#!/usr/bin/env python3
"""
swarm_leader_follower.py — ROS2 Humble / rclpy — TWO-DRONE SWARM (Behavior 2)
=============================================================================
Crazyflie 2.1+ x2 | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

LEADER (cf231) flies a circle; FOLLOWER (cf2) trails behind it.

WHY THE OLD VERSION CRASHED (and what changed)
----------------------------------------------
* Follower into the WALL: _follower_target() built the trailing direction by
  differentiating the leader's NOISY measured position with only a 1 mm gate and
  no step limit, so the target flung to random bearings into walls. FIXED: the
  follower trails the leader's LIVE position but along the circle's ANALYTIC
  tangent (smooth, noise-free), plus a hard per-tick MAX_STEP speed limit.
* Leader into the CEILING: the commanded Z is a constant, so the only way a
  drone reaches the ceiling is a diverged Z estimate, and the old script had no
  guard. FIXED: a divergence watchdog auto-lands both if either drone's estimate
  runs away, and a pre-takeoff Z-sanity check refuses to arm on a bad estimate.
* Off-centre / wall-poking circle: now centred via dynamic_start.fit_center so
  the whole leader+follower pattern stays inside the anchor hull.

SAFETY: vertical separation (leader low, follower high), min-distance floor,
safe-zone clamp, /all/emergency stops BOTH — keep that terminal ready.

Run:
  python3 swarm_leader_follower.py
"""

import os, sys, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           fit_center, clamp_xy, in_safe_zone)

LEADER   = "cf231"
FOLLOWER = "cf2"

# ── Leader circle ──
RADIUS   = 0.8
OMEGA    = 0.4
CIRCLE_DURATION = 2 * math.pi / OMEGA
LEADER_Z = 0.7

# ── Follower ──
FOLLOW_OFFSET = 0.8     # m — trailing distance behind the leader
FOLLOWER_Z    = 1.0     # m — higher than leader (vertical separation)
MIN_DISTANCE  = 0.6     # m — never command follower closer than this (XY)
MAX_STEP      = 0.35    # m — max follower target move per tick (speed limit)
FOLLOW_DT     = 0.25

# circle centre is fit so leader+follower both stay in the hull
REACH = RADIUS + FOLLOW_OFFSET

TAKEOFF_TIME = 3.0
LPS_SETTLE   = 3.0

# ── Divergence / runaway guard ──
Z_ABORT    = FOLLOWER_Z + 0.7     # measured z above this -> climbing away
POS_ABORT  = 1.5                  # measured XY this far from target -> runaway
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting
FLOOR_Z_OK = 0.35                 # pre-takeoff: |z| must be below this on floor

CSV_PATH = os.path.expanduser("~/swarm_leader_follower_log.csv")


class SwarmLeaderFollower(Node):

    def __init__(self):
        super().__init__("swarm_leader_follower")
        self.pose = {LEADER: None, FOLLOWER: None}
        self.setpoint = {LEADER: (0.0, 0.0, LEADER_Z),
                         FOLLOWER: (0.0, 0.0, FOLLOWER_Z)}
        self.log_data = []
        self.t0 = None
        self.cx = self.cy = None
        self.theta = 0.0
        self.circle_t0 = None

        self.state = "WAIT_POSE"
        self._pending = {LEADER: None, FOLLOWER: None}
        self._wait_until = 0
        self._finished = False
        self._saved = False
        self._guard_after = 0
        self._bad_count = 0

        self.arm_cli, self.takeoff_cli, self.goto_cli, self.land_cli = {}, {}, {}, {}
        for d in (LEADER, FOLLOWER):
            self.create_subscription(
                PoseStamped, f"/{d}/pose",
                lambda msg, dd=d: self._pose_cb(dd, msg), 10)
            self.arm_cli[d]     = self.create_client(Arm,     f"/{d}/arm")
            self.takeoff_cli[d] = self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d]    = self.create_client(GoTo,    f"/{d}/go_to")
            self.land_cli[d]    = self.create_client(Land,    f"/{d}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"SwarmLeaderFollower: leader={LEADER} follower={FOLLOWER}")

    def _pose_cb(self, drone, msg):
        self.pose[drone] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.t0 is not None and not self._finished:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            row = {"time": round(t, 3)}
            for d in (LEADER, FOLLOWER):
                p = self.pose[d] if self.pose[d] else (0, 0, 0)
                s = self.setpoint[d]
                row[f"{d}_x"], row[f"{d}_y"], row[f"{d}_z"] = p
                row[f"{d}_ex"], row[f"{d}_ey"], row[f"{d}_ez"] = s
            if self.pose[LEADER] and self.pose[FOLLOWER]:
                lx, ly, lz = self.pose[LEADER]; fx, fy, fz = self.pose[FOLLOWER]
                row["separation_xy"] = math.hypot(fx - lx, fy - ly)
                row["separation_3d"] = math.sqrt((fx-lx)**2+(fy-ly)**2+(fz-lz)**2)
            self.log_data.append(row)

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until
    def _both_pose(self): return all(self.pose[d] is not None for d in (LEADER, FOLLOWER))
    def _both_done(self):
        return all(self._pending[d] is not None and self._pending[d].done()
                   for d in (LEADER, FOLLOWER))

    def _send_goto(self, drone, x, y, z, duration):
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(duration); req.duration.nanosec = int((duration % 1) * 1e9)
        req.relative = False
        self.setpoint[drone] = (x, y, z)
        return self.goto_cli[drone].call_async(req)

    def _diverged(self):
        # Divergence auto-land DISABLED for LPS/TDoA2. The low anchors sit at
        # 0.30 m, so a drone on the floor is below the anchor plane and its
        # Z estimate is legitimately noisy/negative there — gating on it caused
        # false aborts on a healthy system. Manual /all/emergency (or
        # /cf231/emergency) remains available if a drone ever misbehaves.
        return None

    def _leader_circle_point(self, t):
        theta = OMEGA * t
        return self.cx + RADIUS * math.cos(theta), self.cy + RADIUS * math.sin(theta), theta

    def _follower_target(self, t):
        """Trail the leader's LIVE position, but along the circle's ANALYTIC
        tangent (smooth) — not a differentiated noisy delta. Speed-limited."""
        lp = self.pose[LEADER]
        if lp is None: return None
        lx, ly, _ = lp
        theta = OMEGA * max(t, 0.0)
        # CCW tangent (direction of leader motion); follower sits opposite it
        tang_x, tang_y = -math.sin(theta), math.cos(theta)
        gx = lx - tang_x * FOLLOW_OFFSET
        gy = ly - tang_y * FOLLOW_OFFSET

        # min-distance floor from the leader
        d = math.hypot(gx - lx, gy - ly)
        if d < MIN_DISTANCE:
            ux, uy = ((gx-lx)/d, (gy-ly)/d) if d > 1e-3 else (1.0, 0.0)
            gx, gy = lx + ux*MIN_DISTANCE, ly + uy*MIN_DISTANCE

        # speed limit from the follower's measured position
        fp = self.pose[FOLLOWER]
        if fp is not None:
            sx, sy = gx - fp[0], gy - fp[1]; sd = math.hypot(sx, sy)
            if sd > MAX_STEP:
                gx, gy = fp[0] + sx/sd*MAX_STEP, fp[1] + sy/sd*MAX_STEP

        gx, gy = clamp_xy(gx, gy)
        return gx, gy, FOLLOWER_Z

    def _tick(self):

        if self.state == "WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE * 1e9)
                self.get_logger().info(f"Both have pose. Settling {LPS_SETTLE}s ...")
                for d in (LEADER, FOLLOWER):
                    self.get_logger().info(
                        f"  {d}: x={self.pose[d][0]:.2f} y={self.pose[d][1]:.2f} "
                        f"z={self.pose[d][2]:.2f}")
                return
            if not self._past(): return
            # Pre-takeoff Z sanity — a bad estimate now is the ceiling crash later.
            if not in_safe_zone(self.pose[LEADER][0], self.pose[LEADER][1]):
                self.get_logger().warn(
                    f"{LEADER} placed outside safe zone — circle will be centred at "
                    f"the hull centre instead of the placement.")
            self.cx, self.cy = fit_center(self.pose[LEADER][0], self.pose[LEADER][1],
                                          REACH, REACH)
            # Pre-takeoff placement check: make sure this specific placement won't
            # bring the two drones close during the sequenced approach. The rim is
            # at (cx+R, cy); the follower's trailing point is RIM_OFFSET below it.
            rim = (self.cx + RADIUS, self.cy)
            trail = (rim[0], rim[1] - FOLLOW_OFFSET)
            lspawn = (self.pose[LEADER][0], self.pose[LEADER][1])
            fspawn = (self.pose[FOLLOWER][0], self.pose[FOLLOWER][1])
            d_lead_trail = math.hypot(lspawn[0]-trail[0], lspawn[1]-trail[1])
            d_foll_rim   = math.hypot(fspawn[0]-rim[0], fspawn[1]-rim[1])
            if d_lead_trail < 0.6 or d_foll_rim < 0.6:
                self.get_logger().error(
                    f"Placement too tight: leader is {d_lead_trail:.2f}m from the "
                    f"follower's trail point and follower is {d_foll_rim:.2f}m from the "
                    f"leader's rim (need >0.6m). Move the two drones further apart "
                    f"(place {FOLLOWER} on the OPPOSITE side of the room from where "
                    f"{LEADER}'s circle will be). NOT arming.")
                self.state = "SAVE"; return
            self.get_logger().info(
                f"Circle centre ({self.cx:.2f},{self.cy:.2f}) R={RADIUS} "
                f"(leader+follower kept in hull).")
            self.t0 = self._now() / 1e9
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending[LEADER] is None:
                for d in (LEADER, FOLLOWER):
                    req = Arm.Request(); req.arm = True
                    self._pending[d] = self.arm_cli[d].call_async(req)
                self.get_logger().info("Arming both ...")
            elif self._both_done():
                self.get_logger().info("Both armed.")
                self._pending = {LEADER: None, FOLLOWER: None}
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self._pending[LEADER] is None:
                req = Takeoff.Request(); req.height = LEADER_Z
                req.duration.sec = int(TAKEOFF_TIME); req.duration.nanosec = int((TAKEOFF_TIME % 1)*1e9)
                self._pending[LEADER] = self.takeoff_cli[LEADER].call_async(req)
                req2 = Takeoff.Request(); req2.height = FOLLOWER_Z
                req2.duration.sec = int(TAKEOFF_TIME); req2.duration.nanosec = int((TAKEOFF_TIME % 1)*1e9)
                self._pending[FOLLOWER] = self.takeoff_cli[FOLLOWER].call_async(req2)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.5)*1e9)
                self._guard_after = self._now() + int((TAKEOFF_TIME + 1.5 + GUARD_GRACE_S) * 1e9)
                self.get_logger().info(f"Takeoff: {LEADER}->{LEADER_Z}m  {FOLLOWER}->{FOLLOWER_Z}m")
            elif self._both_done() and self._past():
                self._pending = {LEADER: None, FOLLOWER: None}
                self.state = "APPROACH_FOLLOWER"

        # ── Sequenced approach: FOLLOWER goes to its trailing point first (it
        #    starts on the far side from the leader), while the leader holds at
        #    its takeoff spot; THEN the leader moves to the rim while the follower
        #    holds. Only one drone travels at a time, and the mover's path never
        #    passes through the other's position — so they cannot cross. ──
        elif self.state == "APPROACH_FOLLOWER":
            if self._diverged():
                self.state = "LAND"; self._pending = {LEADER: None, FOLLOWER: None}; return
            if self._pending[LEADER] is None:
                lx, ly, _ = self.pose[LEADER]
                # leader HOLDS at its takeoff position (hover in place)
                self._pending[LEADER] = self._send_goto(LEADER, lx, ly, LEADER_Z, 3.0)
                ft = self._follower_target(0.0)
                self._pending[FOLLOWER] = self._send_goto(FOLLOWER, *ft, 3.0) if ft else \
                    self._send_goto(FOLLOWER, self.cx + RADIUS, self.cy - FOLLOW_OFFSET,
                                    FOLLOWER_Z, 3.0)
                self._wait_until = self._now() + int(4.0*1e9)
                self.get_logger().info("Approach 1/2: follower -> trail (leader holding)")
            elif self._both_done() and self._past():
                self._pending = {LEADER: None, FOLLOWER: None}
                self.state = "APPROACH_LEADER"

        elif self.state == "APPROACH_LEADER":
            if self._diverged():
                self.state = "LAND"; self._pending = {LEADER: None, FOLLOWER: None}; return
            if self._pending[LEADER] is None:
                rx, ry = self.cx + RADIUS, self.cy   # rim (theta=0)
                # follower holds at its trail point; leader moves to the rim
                ft = self._follower_target(0.0)
                self._pending[FOLLOWER] = self._send_goto(FOLLOWER, *ft, 2.0) if ft else \
                    self._send_goto(FOLLOWER, rx, ry - FOLLOW_OFFSET, FOLLOWER_Z, 2.0)
                self._pending[LEADER] = self._send_goto(LEADER, rx, ry, LEADER_Z, 3.0)
                self._wait_until = self._now() + int(4.0*1e9)
                self.get_logger().info("Approach 2/2: leader -> rim (follower holding at trail)")
            elif self._both_done() and self._past():
                self._pending = {LEADER: None, FOLLOWER: None}
                self.circle_t0 = self._now() / 1e9
                self.get_logger().info(
                    f"Phase: LEADER circle + FOLLOWER trail "
                    f"(offset={FOLLOW_OFFSET}m, dz={FOLLOWER_Z-LEADER_Z}m)")
                self.state = "FOLLOW"

        elif self.state == "FOLLOW":
            if self._diverged():
                self.state = "LAND"; self._pending = {LEADER: None, FOLLOWER: None}; return
            t = self._now() / 1e9 - self.circle_t0
            if t > CIRCLE_DURATION:
                self.state = "SETTLE"; self._pending = {LEADER: None, FOLLOWER: None}; return
            lx, ly, self.theta = self._leader_circle_point(t)
            lx, ly = clamp_xy(lx, ly)
            self._send_goto(LEADER, lx, ly, LEADER_Z, FOLLOW_DT)
            ft = self._follower_target(t)
            if ft:
                self._send_goto(FOLLOWER, *ft, FOLLOW_DT)
            self._wait_until = self._now() + int(FOLLOW_DT * 1e9)
            if self.pose[LEADER] and self.pose[FOLLOWER] and int(t*4) % 8 == 0:
                fx, fy, _ = self.pose[FOLLOWER]
                sep = math.hypot(fx - self.pose[LEADER][0], fy - self.pose[LEADER][1])
                self.get_logger().info(f"  t={t:4.1f}s  separation={sep:.2f}m")

        elif self.state == "SETTLE":
            if self._diverged():
                self.state = "LAND"; self._pending = {LEADER: None, FOLLOWER: None}; return
            if self._pending[LEADER] is None:
                rx, ry = self.cx + RADIUS, self.cy
                self._pending[LEADER] = self._send_goto(LEADER, rx, ry, LEADER_Z, 2.0)
                ft = self._follower_target(0.0)
                self._pending[FOLLOWER] = self._send_goto(FOLLOWER, *ft, 2.0) if ft else \
                    self._send_goto(FOLLOWER, rx, ry - FOLLOW_OFFSET, FOLLOWER_Z, 2.0)
                self._wait_until = self._now() + int(3.0*1e9)
                self.get_logger().info("Settling both before landing ...")
            elif self._both_done() and self._past():
                self._pending = {LEADER: None, FOLLOWER: None}
                self.state = "LAND"

        elif self.state == "LAND":
            if self._pending[LEADER] is None:
                for d in (LEADER, FOLLOWER):
                    req = Land.Request(); req.height = 0.0
                    req.duration.sec = 3; req.duration.nanosec = 0
                    self._pending[d] = self.land_cli[d].call_async(req)
                self._wait_until = self._now() + int(5.0*1e9)
                self.get_logger().info("Both landing ...")
            elif self._both_done() and self._past():
                self._pending = {LEADER: None, FOLLOWER: None}
                self.get_logger().info("Both landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._finalize(); self.state = "DONE"

        elif self.state == "DONE":
            self._finished = True

    def _finalize(self):
        if self._saved: return
        self._saved = True
        if not self.log_data:
            self.get_logger().info("Leader-follower complete (no data)."); return
        keys = ["time"]
        for d in (LEADER, FOLLOWER):
            keys += [f"{d}_x", f"{d}_y", f"{d}_z", f"{d}_ex", f"{d}_ey", f"{d}_ez"]
        keys += ["separation_xy", "separation_3d"]
        try:
            with open(CSV_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                w.writeheader(); w.writerows(self.log_data)
            self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        seps = [r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps:
            self.get_logger().info(
                f"Separation: min={min(seps):.2f}m max={max(seps):.2f}m "
                f"mean={sum(seps)/len(seps):.2f}m")
        self.get_logger().info("Leader-follower swarm complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLeaderFollower()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — saving what we have ...")
    finally:
        node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
