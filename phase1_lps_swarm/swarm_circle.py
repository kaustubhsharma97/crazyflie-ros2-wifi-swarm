#!/usr/bin/env python3
"""
swarm_circle.py — TWO-DRONE SYNCHRONIZED CIRCLE (cf231 + cf2, 180° apart)
=========================================================================
Crazyflie 2.1+ x2 | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

Both drones fly the SAME circle, phase-offset by 180°, so they sit on opposite
sides and chase each other around it. Different altitudes give a guaranteed
vertical safety margin at all times (even though 180° phase already keeps them
a full diameter apart in XY).

Pure rclpy. One non-blocking timer commands BOTH drones. Auto-saves CSV + PNG.
Clean shutdown (returns your prompt). Whole-room safe zone.

Run:
  python3 swarm_circle.py
"""

import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import csv, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

# ── Drone names (must match crazyflies_swarm.yaml) ──
D1, D2 = "cf231", "cf2"

# ── Circle parameters (both drones share this circle) ──
CENTER_X = 1.5          # circle centre X (put it in the middle of your safe area)
CENTER_Y = 2.5          # circle centre Y
RADIUS   = 1.0          # circle radius (m)
OMEGA    = 0.4          # angular speed (rad/s) — slower = cleaner tracking
LAPS     = 1.0          # how many full loops to fly
PHASE    = math.pi      # 180° between the two drones (opposite sides)

# ── Altitudes (different for vertical safety) ──
Z1 = 0.6                # cf231 altitude
Z2 = 0.9                # cf2 altitude (higher)

# ── Timing ──
TAKEOFF_TIME = 3.0
LPS_SETTLE   = 3.0
DT           = 0.25     # setpoint update period during the circle

CIRCLE_DURATION = LAPS * 2 * math.pi / OMEGA

# ── Whole-room safe zone (adjust to your anchor area) ──
SAFE_X = (0.0, 5.0)
SAFE_Y = (0.0, 8.0)

CSV_PATH = os.path.expanduser("~/swarm_circle_log.csv")

def clamp(v, lo, hi): return max(lo, min(hi, v))


class SwarmCircle(Node):
    def __init__(self):
        super().__init__("swarm_circle")
        self._finished = False
        self.pose = {D1: None, D2: None}
        self.setpoint = {D1: (0, 0, Z1), D2: (0, 0, Z2)}
        self.log_data = []
        self.t0 = None
        self.circle_t0 = None
        self.state = "WAIT_POSE"
        self._pending = {D1: None, D2: None}
        self._wait_until = 0

        self.arm_cli = {}; self.tk_cli = {}; self.goto_cli = {}; self.land_cli = {}
        for d in (D1, D2):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m, dd=d: self._pcb(dd, m), 10)
            self.arm_cli[d]  = self.create_client(Arm,     f"/{d}/arm")
            self.tk_cli[d]   = self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d] = self.create_client(GoTo,    f"/{d}/go_to")
            self.land_cli[d] = self.create_client(Land,    f"/{d}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"SwarmCircle: {D1} & {D2} on same circle, 180° apart "
            f"(centre=({CENTER_X},{CENTER_Y}) r={RADIUS} omega={OMEGA})")

    def _pcb(self, d, m):
        self.pose[d] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        if self.t0 is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            row = {"time": round(t, 3)}
            for dd in (D1, D2):
                p = self.pose[dd] or (0, 0, 0); s = self.setpoint[dd]
                row[f"{dd}_x"], row[f"{dd}_y"], row[f"{dd}_z"] = p
                row[f"{dd}_ex"], row[f"{dd}_ey"], row[f"{dd}_ez"] = s
            if self.pose[D1] and self.pose[D2]:
                a, b = self.pose[D1], self.pose[D2]
                row["separation_xy"] = math.hypot(b[0]-a[0], b[1]-a[1])
            self.log_data.append(row)

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until
    def _both_pose(self): return all(self.pose[d] is not None for d in (D1, D2))
    def _both_done(self):
        return all(self._pending[d] is not None and self._pending[d].done()
                   for d in (D1, D2))

    def _goto(self, d, x, y, z, dur):
        r = GoTo.Request()
        r.goal.x = float(x); r.goal.y = float(y); r.goal.z = float(z)
        r.yaw = 0.0
        r.duration.sec = int(dur); r.duration.nanosec = int((dur % 1) * 1e9)
        r.relative = False
        self.setpoint[d] = (x, y, z)
        return self.goto_cli[d].call_async(r)

    def _circle_point(self, t, phase, z):
        th = OMEGA * t + phase
        x = clamp(CENTER_X + RADIUS * math.cos(th), SAFE_X[0]+0.1, SAFE_X[1]-0.1)
        y = clamp(CENTER_Y + RADIUS * math.sin(th), SAFE_Y[0]+0.1, SAFE_Y[1]-0.1)
        return x, y, z

    def _tick(self):
        s = self.state

        if s == "WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE * 1e9)
                self.get_logger().info(f"Both have pose. Settling {LPS_SETTLE}s ...")
                for d in (D1, D2):
                    self.get_logger().info(
                        f"  {d}: x={self.pose[d][0]:.2f} y={self.pose[d][1]:.2f} "
                        f"z={self.pose[d][2]:.2f}")
                return
            if not self._past(): return
            self.t0 = self._now() / 1e9
            self.state = "ARM"

        elif s == "ARM":
            if self._pending[D1] is None:
                for d in (D1, D2):
                    r = Arm.Request(); r.arm = True
                    self._pending[d] = self.arm_cli[d].call_async(r)
                self.get_logger().info("Arming both ...")
            elif self._both_done():
                self.get_logger().info("Both armed.")
                self._pending = {D1: None, D2: None}
                self.state = "TAKEOFF"

        elif s == "TAKEOFF":
            if self._pending[D1] is None:
                for d, z in ((D1, Z1), (D2, Z2)):
                    r = Takeoff.Request(); r.height = z
                    r.duration.sec = int(TAKEOFF_TIME)
                    r.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                    self._pending[d] = self.tk_cli[d].call_async(r)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.5) * 1e9)
                self.get_logger().info(f"Takeoff: {D1}->{Z1}m  {D2}->{Z2}m")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.state = "APPROACH"

        elif s == "APPROACH":
            # move each drone to its circle START point (opposite sides)
            if self._pending[D1] is None:
                self._pending[D1] = self._goto(D1, *self._circle_point(0, 0, Z1), 3.0)
                self._pending[D2] = self._goto(D2, *self._circle_point(0, PHASE, Z2), 3.0)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Approaching circle start (opposite sides) ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.circle_t0 = self._now() / 1e9
                self.get_logger().info(f"Flying synchronized circle ({LAPS} lap/s) ...")
                self.state = "CIRCLE"

        elif s == "CIRCLE":
            t = self._now() / 1e9 - self.circle_t0
            if t > CIRCLE_DURATION:
                self.state = "SETTLE"; self._pending = {D1: None, D2: None}; return
            self._goto(D1, *self._circle_point(t, 0, Z1), DT)
            self._goto(D2, *self._circle_point(t, PHASE, Z2), DT)
            self._wait_until = self._now() + int(DT * 1e9)
            if self.pose[D1] and self.pose[D2]:
                sep = math.hypot(self.pose[D2][0]-self.pose[D1][0],
                                 self.pose[D2][1]-self.pose[D1][1])
                if int(t*4) % 8 == 0:
                    self.get_logger().info(f"  t={t:4.1f}s  separation={sep:.2f}m")

        elif s == "SETTLE":
            if self._pending[D1] is None:
                self._pending[D1] = self._goto(D1, *self._circle_point(0, 0, Z1), 2.0)
                self._pending[D2] = self._goto(D2, *self._circle_point(0, PHASE, Z2), 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)
                self.get_logger().info("Settling before landing ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.state = "LAND"

        elif s == "LAND":
            if self._pending[D1] is None:
                for d in (D1, D2):
                    r = Land.Request(); r.height = 0.0
                    r.duration.sec = 3; r.duration.nanosec = 0
                    self._pending[d] = self.land_cli[d].call_async(r)
                self._wait_until = self._now() + int(5.0 * 1e9)
                self.get_logger().info("Both landing ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.get_logger().info("Both landed.")
                self.state = "SAVE"

        elif s == "SAVE":
            self._save()
            self.get_logger().info("Swarm circle complete.")
            self.state = "DONE"

        elif s == "DONE":
            self._finished = True

    def _save(self):
        if not self.log_data:
            self.get_logger().warn("No data to save."); return
        keys = ["time"]
        for d in (D1, D2):
            keys += [f"{d}_x", f"{d}_y", f"{d}_z", f"{d}_ex", f"{d}_ey", f"{d}_ez"]
        keys += ["separation_xy"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        seps = [r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps:
            self.get_logger().info(
                f"Separation: min={min(seps):.2f} max={max(seps):.2f} "
                f"mean={sum(seps)/len(seps):.2f} m (should stay ~{2*RADIUS:.1f} at 180°)")
        try:
            import swarm_plots
            png = swarm_plots.auto_plot(CSV_PATH, title="Swarm Circle (180° apart)")
            if png:
                self.get_logger().info(f"PNG -> {png}")
        except Exception as e:
            self.get_logger().warn(f"PNG generation skipped: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCircle()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try: node._save()
        except Exception: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
