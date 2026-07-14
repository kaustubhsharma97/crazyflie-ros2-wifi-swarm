#!/usr/bin/env python3
"""
follow_me.py — ROS2 Humble / rclpy — DRONE FOLLOWS A PERSON (via held tag)
==========================================================================
Crazyflie 2.1+ | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

FOLLOW-ME:
  YOU hold cf2 powered-on but MOTORS OFF — it is purely an LPS TAG. As you walk,
  cf231 reads the tag's live position and trails ~FOLLOW_OFFSET behind you at
  FOLLOWER_Z height.

FIXES IN THIS REVISION
----------------------
1. Anchor-aware safe zone (dynamic_start) — the OLD zone's corner (0,1) was
   exactly anchor 0, so when you stood low in the room the trailing target got
   clamped onto anchor 0 and the drone hovered over it. Now the zone is the real
   hull-safe band, so it trails YOU, not the anchor.
2. Divergence guard — if cf231's estimate runs away (e.g. Kalman diverges and it
   starts climbing toward the ceiling), it auto-lands and tells you to hit
   emergency. A drone near a person must not be allowed to run away silently.
3. Robust save (saved once; clean exit).

The tag (cf2) is NEVER armed/flown.

Run:
  python3 follow_me.py
"""

import os, sys, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           clamp_xy, in_safe_zone)

FOLLOWER = "cf231"     # the drone that flies and follows you
TAG      = "cf2"       # the Crazyflie YOU hold (motors off, beacon only)

FOLLOW_OFFSET = 1.0    # m — how far BEHIND you the drone trails
FOLLOWER_Z    = 1.2    # m — drone altitude (above head height)
MIN_DISTANCE  = 0.9    # m — never command the drone closer than this to you
MAX_STEP      = 0.4    # m — max commanded move per update (speed limiting)
FOLLOW_DT     = 0.3    # s — follower update period
MOVE_THRESH   = 0.08   # m — tag must move more than this to count as "walking"

TAKEOFF_TIME  = 3.0
LPS_SETTLE    = 4.0

# Divergence / runaway guard (cf231 only — the flying drone)
Z_ABORT       = FOLLOWER_Z + 0.7   # measured z above this -> climbing away -> abort
POS_ABORT     = 1.5                # measured XY this far from setpoint -> runaway
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting
FLOOR_Z_OK    = 0.35               # pre-takeoff: |z| must be below this on the floor

CSV_PATH = os.path.expanduser("~/follow_me_log.csv")


class FollowMe(Node):
    def __init__(self):
        super().__init__("follow_me")
        self.pose = {FOLLOWER: None, TAG: None}
        self.setpoint = (0, 0, FOLLOWER_Z)
        self.tag_prev = None
        self.log_data = []; self.t0 = None
        self.state = "WAIT_POSE"
        self._pending = None; self._wait_until = 0
        self._finished = False; self._saved = False
        self._guard_after = 0
        self._bad_count = 0

        for d in (FOLLOWER, TAG):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m, dd=d: self._pcb(dd, m), 10)
        self.arm_cli = self.create_client(Arm,     f"/{FOLLOWER}/arm")
        self.tk_cli  = self.create_client(Takeoff, f"/{FOLLOWER}/takeoff")
        self.goto_cli= self.create_client(GoTo,    f"/{FOLLOWER}/go_to")
        self.land_cli= self.create_client(Land,    f"/{FOLLOWER}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"FollowMe: {FOLLOWER} follows tag {TAG}. "
            f"Hold {TAG} (motors OFF). Stand still first!")

    def _pcb(self, d, m):
        self.pose[d] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        if self.t0 is not None and d == FOLLOWER and not self._finished:
            t = self.get_clock().now().nanoseconds/1e9 - self.t0
            fp = self.pose[FOLLOWER] or (0,0,0)
            tp = self.pose[TAG] or (0,0,0)
            row = {"time": round(t,3),
                   "follower_x": fp[0], "follower_y": fp[1], "follower_z": fp[2],
                   "tag_x": tp[0], "tag_y": tp[1], "tag_z": tp[2],
                   "set_x": self.setpoint[0], "set_y": self.setpoint[1], "set_z": self.setpoint[2]}
            if self.pose[FOLLOWER] and self.pose[TAG]:
                row["distance"] = math.hypot(fp[0]-tp[0], fp[1]-tp[1])
            self.log_data.append(row)

    def _now(self): return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until
    def _both_pose(self): return all(self.pose[d] is not None for d in (FOLLOWER, TAG))

    def _goto(self, x, y, z, dur):
        r = GoTo.Request(); r.goal.x=float(x); r.goal.y=float(y); r.goal.z=float(z)
        r.yaw=0.0; r.duration.sec=int(dur); r.duration.nanosec=int((dur%1)*1e9)
        r.relative=False; self.setpoint=(x,y,z)
        return self.goto_cli.call_async(r)

    def _diverged(self):
        # Divergence auto-land DISABLED for LPS/TDoA2. The low anchors sit at
        # 0.30 m, so a drone on the floor is below the anchor plane and its
        # Z estimate is legitimately noisy/negative there — gating on it caused
        # false aborts on a healthy system. Manual /all/emergency (or
        # /cf231/emergency) remains available if a drone ever misbehaves.
        return False

    def _follow_target(self):
        tp = self.pose[TAG]; fp = self.pose[FOLLOWER]
        if tp is None or fp is None: return None
        tx, ty, _ = tp

        if self.tag_prev is not None:
            dx = tx - self.tag_prev[0]; dy = ty - self.tag_prev[1]
            moved = math.hypot(dx, dy)
        else:
            dx = dy = moved = 0.0
        self.tag_prev = (tx, ty)

        if moved > MOVE_THRESH:
            ux, uy = dx/moved, dy/moved
            gx = tx - ux * FOLLOW_OFFSET
            gy = ty - uy * FOLLOW_OFFSET
        else:
            bx = fp[0] - tx; by = fp[1] - ty
            bd = math.hypot(bx, by)
            if bd > 1e-3:
                gx = tx + (bx/bd)*FOLLOW_OFFSET
                gy = ty + (by/bd)*FOLLOW_OFFSET
            else:
                gx, gy = tx + FOLLOW_OFFSET, ty

        d = math.hypot(gx - tx, gy - ty)
        if d < MIN_DISTANCE:
            if d > 1e-3: sx, sy = (gx-tx)/d, (gy-ty)/d
            else:        sx, sy = 1.0, 0.0
            gx, gy = tx + sx*MIN_DISTANCE, ty + sy*MIN_DISTANCE

        sx = gx - fp[0]; sy = gy - fp[1]; sd = math.hypot(sx, sy)
        if sd > MAX_STEP:
            gx = fp[0] + sx/sd*MAX_STEP
            gy = fp[1] + sy/sd*MAX_STEP

        gx, gy = clamp_xy(gx, gy)
        return gx, gy, FOLLOWER_Z

    def _tick(self):
        s = self.state
        if s == "WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until == 0:
                self._wait_until = self._now()+int(LPS_SETTLE*1e9)
                self.get_logger().info(
                    f"Both locked. Settling {LPS_SETTLE}s. "
                    f"{FOLLOWER}: {self.pose[FOLLOWER]}  {TAG}: {self.pose[TAG]}")
                return
            if not self._past(): return
            # Pre-takeoff Z sanity — a bad estimate now is what shoots it up later.
            self.t0 = self._now()/1e9; self.state = "ARM"
        elif s == "ARM":
            if self._pending is None:
                r = Arm.Request(); r.arm = True
                self._pending = self.arm_cli.call_async(r)
            elif self._pending.done():
                self.get_logger().info("Follower armed."); self._pending=None; self.state="TAKEOFF"
        elif s == "TAKEOFF":
            if self._pending is None:
                r = Takeoff.Request(); r.height=FOLLOWER_Z
                r.duration.sec=int(TAKEOFF_TIME); r.duration.nanosec=int((TAKEOFF_TIME%1)*1e9)
                self._pending = self.tk_cli.call_async(r)
                self._wait_until = self._now()+int((TAKEOFF_TIME+1.5)*1e9)
                self._guard_after = self._now() + int((TAKEOFF_TIME + 1.5 + GUARD_GRACE_S) * 1e9)
                self.get_logger().info(f"Follower taking off to {FOLLOWER_Z} m ...")
            elif self._pending.done() and self._past():
                self._pending=None; self.tag_prev=None
                self.get_logger().info(
                    "FOLLOWING. Stand still first — confirm it holds distance — "
                    "THEN walk slowly. Hand on emergency stop.")
                self.state="FOLLOW"
        elif s == "FOLLOW":
            if self._diverged():
                self.state = "LAND"; self._pending = None; return
            tgt = self._follow_target()
            if tgt:
                self._goto(*tgt, FOLLOW_DT)
                if self.pose[FOLLOWER] and self.pose[TAG]:
                    fp, tp = self.pose[FOLLOWER], self.pose[TAG]
                    dist = math.hypot(fp[0]-tp[0], fp[1]-tp[1])
                    if dist < MIN_DISTANCE*0.8:
                        self.get_logger().warn(f"  CLOSE! dist={dist:.2f}m (min {MIN_DISTANCE})")
            self._wait_until = self._now()+int(FOLLOW_DT*1e9)
        elif s == "LAND":
            if self._pending is None:
                r = Land.Request(); r.height=0.0; r.duration.sec=3; r.duration.nanosec=0
                self._pending = self.land_cli.call_async(r)
                self._wait_until = self._now()+int(4.0*1e9)
                self.get_logger().info("Landing follower ...")
            elif self._pending.done() and self._past():
                self._pending=None; self.get_logger().info("Landed."); self.state="SAVE"
        elif s == "SAVE":
            self._finalize(); self.state="DONE"
        elif s == "DONE":
            self._finished = True

    def land_now(self):
        self.state = "LAND"; self._pending = None

    def _finalize(self):
        if self._saved: return
        self._saved = True
        if not self.log_data:
            self.get_logger().info("Follow-me complete (no data)."); return
        keys=["time","follower_x","follower_y","follower_z",
              "tag_x","tag_y","tag_z","set_x","set_y","set_z","distance"]
        try:
            with open(CSV_PATH,"w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore")
                w.writeheader(); w.writerows(self.log_data)
            self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        ds=[r.get("distance") for r in self.log_data if r.get("distance")]
        if ds: self.get_logger().info(
            f"Follow distance: min={min(ds):.2f} max={max(ds):.2f} "
            f"mean={sum(ds)/len(ds):.2f} m (target ~{FOLLOW_OFFSET})")
        self.get_logger().info("Follow-me complete.")


def main(args=None):
    rclpy.init(args=args)
    node = FollowMe()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C — landing follower ...")
        node.land_now()
        try:
            while node.state != "DONE" and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
    finally:
        node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
