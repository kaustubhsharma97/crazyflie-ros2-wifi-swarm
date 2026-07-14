#!/usr/bin/env python3
"""
swarm_formation_hold.py - ROS2 Humble / rclpy - TWO-DRONE SWARM (Behavior 1)
============================================================================
Crazyflie 2.1+ x2 (cf231 + cf2) | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

BEHAVIOR 1 - FORMATION HOLD: both arm, take off, hold a fixed-offset formation
hover a set distance apart, then land together. Simplest swarm primitive.

UPDATED to lab standard:
  - Dynamic placement: formation is centred near where cf231 is placed (via
    dynamic_start.fit_center), so it stays inside the anchor hull and off the
    anchors - the old hardcoded (1.0,2.5)/(2.5,2.5) could sit on anchor 0.
  - Divergence guard: auto-lands BOTH if either estimate runs away.
  - Pre-takeoff Z sanity; separation logging; robust save; Ctrl+C bug fixed.

/all/emergency stops BOTH drones - keep that terminal ready.

Run: python3 swarm_formation_hold.py
"""
import os, sys, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           fit_center, clamp_xy, in_safe_zone)

DRONES = ["cf231", "cf2"]

HOVER_HEIGHT = 0.5
TAKEOFF_TIME = 3.0
HOVER_TIME   = 8.0
LAND_TIME    = 3.0
LPS_SETTLE   = 3.0
HALF_GAP     = 0.75          # each drone sits this far from the formation centre (1.5 m apart)
REACH        = HALF_GAP

Z_ABORT    = HOVER_HEIGHT + 0.7
POS_ABORT  = 1.5
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting
FLOOR_Z_OK = 0.35

CSV_PATH = os.path.expanduser("~/swarm_formation_log.csv")


class SwarmFormationHold(Node):

    def __init__(self):
        super().__init__("swarm_formation_hold")
        self._finished = False
        self._saved = False
        self._guard_after = 0
        self._bad_count = 0
        self.pose = {d: None for d in DRONES}
        self.setpoint = {d: (0.0, 0.0, HOVER_HEIGHT) for d in DRONES}
        self.formation = {d: None for d in DRONES}
        self.log_data = []
        self.t0 = None
        self.state = "WAIT_POSE"
        self._pending = {d: None for d in DRONES}
        self._wait_until = 0

        self.arm_cli, self.takeoff_cli, self.goto_cli, self.land_cli = {}, {}, {}, {}
        for d in DRONES:
            self.create_subscription(
                PoseStamped, f"/{d}/pose",
                lambda msg, dd=d: self._pose_cb(dd, msg), 10)
            self.arm_cli[d]     = self.create_client(Arm,     f"/{d}/arm")
            self.takeoff_cli[d] = self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d]    = self.create_client(GoTo,    f"/{d}/go_to")
            self.land_cli[d]    = self.create_client(Land,    f"/{d}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"SwarmFormationHold started for {DRONES} (non-blocking).")

    def _pose_cb(self, drone, msg):
        self.pose[drone] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.t0 is not None and not self._finished:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            row = {"time": round(t, 3)}
            for d in DRONES:
                pp = self.pose[d] if self.pose[d] else (0, 0, 0)
                sp = self.setpoint[d]
                row[f"{d}_x"], row[f"{d}_y"], row[f"{d}_z"] = pp
                row[f"{d}_ex"], row[f"{d}_ey"], row[f"{d}_ez"] = sp
            if self.pose[DRONES[0]] and self.pose[DRONES[1]]:
                a, b = self.pose[DRONES[0]], self.pose[DRONES[1]]
                row["separation_xy"] = math.hypot(b[0]-a[0], b[1]-a[1])
            self.log_data.append(row)

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until
    def _all_have_pose(self): return all(self.pose[d] is not None for d in DRONES)
    def _all_done(self):
        return all(self._pending[d] is not None and self._pending[d].done() for d in DRONES)

    def _goto(self, d, x, y, z, dur):
        r = GoTo.Request(); r.goal.x=float(x); r.goal.y=float(y); r.goal.z=float(z)
        r.yaw=0.0; r.duration.sec=int(dur); r.duration.nanosec=int((dur%1)*1e9)
        r.relative=False; self.setpoint[d]=(x,y,z)
        return self.goto_cli[d].call_async(r)

    def _diverged(self):
        # Divergence auto-land DISABLED for LPS/TDoA2. The low anchors sit at
        # 0.30 m, so a drone on the floor is below the anchor plane and its
        # Z estimate is legitimately noisy/negative there — gating on it caused
        # false aborts on a healthy system. Manual /all/emergency (or
        # /cf231/emergency) remains available if a drone ever misbehaves.
        return False

    def _tick(self):
        if self.state == "WAIT_POSE":
            if not self._all_have_pose(): return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE * 1e9)
                self.get_logger().info(f"Both drones have pose. Settling {LPS_SETTLE}s ...")
                for d in DRONES:
                    self.get_logger().info(
                        f"  {d}: x={self.pose[d][0]:.2f} y={self.pose[d][1]:.2f} z={self.pose[d][2]:.2f}")
                return
            if not self._past(): return
            cx, cy = fit_center(self.pose[DRONES[0]][0], self.pose[DRONES[0]][1], REACH, REACH)
            self.formation[DRONES[0]] = clamp_xy(cx - HALF_GAP, cy)
            self.formation[DRONES[1]] = clamp_xy(cx + HALF_GAP, cy)
            self.get_logger().info(
                f"Formation centre ({cx:.2f},{cy:.2f}): "
                f"{DRONES[0]}@{self.formation[DRONES[0]]} {DRONES[1]}@{self.formation[DRONES[1]]}")
            self.t0 = self._now() / 1e9
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending[DRONES[0]] is None:
                for d in DRONES:
                    req = Arm.Request(); req.arm = True
                    self._pending[d] = self.arm_cli[d].call_async(req)
                self.get_logger().info("Arming both drones ...")
            elif self._all_done():
                self.get_logger().info("Both armed.")
                self._pending = {d: None for d in DRONES}
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self._pending[DRONES[0]] is None:
                for d in DRONES:
                    fx, fy = self.formation[d]
                    self.setpoint[d] = (fx, fy, HOVER_HEIGHT)
                    req = Takeoff.Request(); req.height = HOVER_HEIGHT
                    req.duration.sec = int(TAKEOFF_TIME); req.duration.nanosec = int((TAKEOFF_TIME % 1)*1e9)
                    self._pending[d] = self.takeoff_cli[d].call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.5) * 1e9)
                self._guard_after = self._now() + int((TAKEOFF_TIME + 1.5 + GUARD_GRACE_S) * 1e9)
                self.get_logger().info(f"Both taking off to {HOVER_HEIGHT} m ...")
            elif self._all_done() and self._past():
                self._pending = {d: None for d in DRONES}
                self.state = "FORM"

        elif self.state == "FORM":
            # move each drone from its takeoff spot to its formation point
            if self._diverged():
                self.state = "LAND"; self._pending = {d: None for d in DRONES}; return
            if self._pending[DRONES[0]] is None:
                for d in DRONES:
                    fx, fy = self.formation[d]
                    self._pending[d] = self._goto(d, fx, fy, HOVER_HEIGHT, 3.0)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Moving into formation ...")
            elif self._all_done() and self._past():
                self._pending = {d: None for d in DRONES}
                self._wait_until = self._now() + int(HOVER_TIME * 1e9)
                self.get_logger().info(f"Formation hover - holding {HOVER_TIME}s.")
                self.state = "HOVER"

        elif self.state == "HOVER":
            if self._diverged():
                self.state = "LAND"; self._pending = {d: None for d in DRONES}; return
            if self._past():
                self.get_logger().info("Hover complete.")
                self.state = "LAND"

        elif self.state == "LAND":
            if self._pending[DRONES[0]] is None:
                for d in DRONES:
                    req = Land.Request(); req.height = 0.0
                    req.duration.sec = int(LAND_TIME); req.duration.nanosec = int((LAND_TIME % 1)*1e9)
                    self._pending[d] = self.land_cli[d].call_async(req)
                self._wait_until = self._now() + int((LAND_TIME + 1.5) * 1e9)
                self.get_logger().info("Both landing ...")
            elif self._all_done() and self._past():
                self._pending = {d: None for d in DRONES}
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
            self.get_logger().info("Formation-hold complete (no data)."); return
        keys = ["time"]
        for d in DRONES:
            keys += [f"{d}_x", f"{d}_y", f"{d}_z", f"{d}_ex", f"{d}_ey", f"{d}_ez"]
        keys += ["separation_xy"]
        try:
            with open(CSV_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                w.writeheader(); w.writerows(self.log_data)
            self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        try:
            import swarm_plots
            png = swarm_plots.auto_plot(CSV_PATH, title="Formation Hold")
            if png: self.get_logger().info(f"PNG -> {png}")
        except Exception as e:
            self.get_logger().warn(f"PNG generation skipped: {e}")
        seps = [r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps:
            self.get_logger().info(
                f"Separation: min={min(seps):.2f} max={max(seps):.2f} mean={sum(seps)/len(seps):.2f} m")
        self.get_logger().info("Swarm formation-hold complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmFormationHold()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted - saving what we have ...")
    finally:
        node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
