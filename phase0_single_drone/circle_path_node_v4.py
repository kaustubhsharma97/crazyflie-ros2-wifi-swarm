#!/usr/bin/env python3
"""
circle_path_node_v4.py - ROS2 Humble / rclpy - SINGLE-DRONE CIRCLE (cf231)
===================================================================
Crazyflie 2.1+ | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

Flies cf231 in a clean, visible circle and saves a CSV + 4 PNGs.

Design goals (per your feedback):
  - Makes a REAL circle you can see: the circle is centred so the drone starts
    on the rim (no lunge), radius/omega chosen for smooth LPS tracking.
  - Self-contained: no import of dynamic_start / error_plots — nothing external
    to break. Safe zone + all plotting are built in.
  - NO floor-Z arming gate and NO divergence auto-land. LPS floor-Z is naturally
    noisy (drone sits below the 0.30 m anchor plane on the ground); it reads
    correctly once airborne. Arms and flies regardless of the ground Z reading.
  - Robust shutdown: no terminal freeze (spin_once main loop).

Flow: WAIT_POSE -> ARM -> TAKEOFF -> GOTO_RIM -> CIRCLE -> RETURN -> LAND -> SAVE
Manual stop: keep /cf231/emergency (or /all/emergency) ready.

Run:  python3 circle_path_node_v4.py
"""
import os, csv, math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Drone ─────────────────────────────────────────────────────────────────────
CF_NAME = "cf231"

# ── Circle parameters ─────────────────────────────────────────────────────────
RADIUS          = 1.3      # m
OMEGA           = 0.5      # rad/s (angular speed)
CIRCLE_DURATION = 20.0     # s (one full loop takes 2*pi/OMEGA ~= 12.6s; 20s > that)
FLY_Z           = 0.6      # m cruise altitude (well inside the anchor volume)
DT              = 0.3      # s per waypoint (10 Hz native on real hw; 0.3 is safe)
TAKEOFF_TIME    = 5.0      # s  GENTLE climb (was 2.5 - aggressive on a low floor-Z)
LAND_TIME       = 3.0      # s
LPS_SETTLE_TIME = 3.0      # s to let the estimator settle before arming

# Center the circle so cf231 STARTS on the rim (theta=0) -> no lunge, clean entry.
START_ON_RIM    = True

# ── Safe zone (from the real B-419 anchor hull) ───────────────────────────────
SAFE_X_MIN, SAFE_X_MAX = 0.6, 4.4
SAFE_Y_MIN, SAFE_Y_MAX = 1.2, 5.6

# ── Outputs ───────────────────────────────────────────────────────────────────
CSV_PATH = os.path.expanduser("~/circle_v4_log.csv")
PNG_XY   = os.path.expanduser("~/circle_v4_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/circle_v4_3d.png")
PNG_ERR  = os.path.expanduser("~/circle_v4_error.png")
PNG_Z    = os.path.expanduser("~/circle_v4_z.png")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class CirclePathNodeV4(Node):

    def __init__(self):
        super().__init__("circle_path_node_v4")
        self._finished = False
        self._saved = False

        self.pose = None
        self.setpoint = (0.0, 0.0, FLY_Z)
        self.log_data = []
        self.t0 = None

        self.spawn = None
        self.cx = self.cy = None           # circle centre (set after spawn read)
        self.state = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.step_i = 0
        self.total_steps = int(CIRCLE_DURATION / DT)

        self.create_subscription(PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)
        self.arm_cli     = self.create_client(Arm,     f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(DT, self._tick)
        self.get_logger().info(f"circle_path_node_v4 started (r={RADIUS}, z={FLY_Z}). Non-blocking.")

    # ── pose logging ──────────────────────────────────────────────────────────
    def _pose_cb(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.t0 is not None and not self._finished:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            self.log_data.append({
                "time": round(t, 3),
                "actual_x": self.pose[0], "actual_y": self.pose[1], "actual_z": self.pose[2],
                "expected_x": self.setpoint[0], "expected_y": self.setpoint[1], "expected_z": self.setpoint[2],
            })

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until

    def _goto(self, x, y, z, dur):
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(dur); req.duration.nanosec = int((dur % 1) * 1e9)
        req.relative = False
        self.setpoint = (x, y, z)
        return self.goto_cli.call_async(req)

    def _rim(self):
        # theta=0 point on the circle
        return (self.cx + RADIUS, self.cy)

    def _place_circle(self):
        sx, sy, _ = self.spawn
        if START_ON_RIM:
            # centre so the drone's spawn is the theta=0 rim point (no lunge)
            cx, cy = sx - RADIUS, sy
        else:
            cx, cy = sx, sy
        # pull the centre in so the whole circle stays strictly inside the safe
        # zone (small margin M avoids the ring just touching the boundary)
        M = 0.05
        cx = clamp(cx, SAFE_X_MIN + RADIUS + M, SAFE_X_MAX - RADIUS - M)
        cy = clamp(cy, SAFE_Y_MIN + RADIUS + M, SAFE_Y_MAX - RADIUS - M)
        self.cx, self.cy = cx, cy

    # ── state machine ─────────────────────────────────────────────────────────
    def _tick(self):
        s = self.state

        if s == "WAIT_POSE":
            if self.pose is None:
                return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(f"Have pose. Settling {LPS_SETTLE_TIME}s "
                                       f"(floor z may read off - that's normal for LPS) ...")
                return
            if not self._past():
                return
            self.spawn = self.pose
            self._place_circle()
            self.t0 = self._now() / 1e9
            self.get_logger().info(
                f"Spawn ({self.spawn[0]:.2f},{self.spawn[1]:.2f},z={self.spawn[2]:.2f}); "
                f"circle centre ({self.cx:.2f},{self.cy:.2f}) r={RADIUS}.")
            self.state = "ARM"

        elif s == "ARM":
            if self._pending is None:
                req = Arm.Request(); req.arm = True
                self._pending = self.arm_cli.call_async(req)
                self.get_logger().info("Arming ...")
            elif self._pending.done():
                self.get_logger().info("Armed.")
                self._pending = None
                self.state = "TAKEOFF"

        elif s == "TAKEOFF":
            if self._pending is None:
                self.setpoint = (self.spawn[0], self.spawn[1], FLY_Z)
                req = Takeoff.Request(); req.height = FLY_Z
                req.duration.sec = int(TAKEOFF_TIME); req.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.5) * 1e9)
                self.get_logger().info(f"Taking off to {FLY_Z} m ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "STABILIZE"

        elif s == "STABILIZE":
            # Hold position at altitude for a few seconds so the estimate settles
            # in the anchor volume (where Z is accurate) BEFORE any horizontal
            # motion. This is what keeps the start smooth instead of jumpy.
            if self._pending is None:
                px, py = (self.pose[0], self.pose[1]) if self.pose else (self.spawn[0], self.spawn[1])
                self._pending = self._goto(px, py, FLY_Z, 2.0)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Stabilising at altitude (settling estimate) ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "GOTO_RIM"

        elif s == "GOTO_RIM":
            if self._pending is None:
                rx, ry = self._rim()
                dist = math.hypot(rx - self.spawn[0], ry - self.spawn[1])
                travel = max(3.0, dist / 0.4)   # gentle approach for a clean settle
                self._pending = self._goto(rx, ry, FLY_Z, travel)
                self._wait_until = self._now() + int((travel + 1.5) * 1e9)
                self.get_logger().info(f"To rim start ({rx:.2f},{ry:.2f}) dist={dist:.2f}m t={travel:.1f}s")
            elif self._pending.done() and self._past():
                self._pending = None
                self.step_i = 0
                self.get_logger().info(f"Circle: r={RADIUS} omega={OMEGA} for {CIRCLE_DURATION}s ...")
                self.state = "CIRCLE"

        elif s == "CIRCLE":
            if self.step_i >= self.total_steps:
                self.state = "RETURN"
                self._pending = None
                return
            theta = OMEGA * (self.step_i * DT)
            wx = self.cx + RADIUS * math.cos(theta)
            wy = self.cy + RADIUS * math.sin(theta)
            self._goto(wx, wy, FLY_Z, DT)
            self.step_i += 1

        elif s == "RETURN":
            if self._pending is None:
                rx, ry = self._rim()
                self._pending = self._goto(rx, ry, FLY_Z, 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)
                self.get_logger().info("Circle complete. Settling before landing ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "LAND"

        elif s == "LAND":
            if self._pending is None:
                req = Land.Request(); req.height = 0.0
                req.duration.sec = int(LAND_TIME); req.duration.nanosec = int((LAND_TIME % 1) * 1e9)
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int((LAND_TIME + 1.5) * 1e9)
                self.get_logger().info("Landing ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif s == "SAVE":
            self._finalize()
            self.state = "DONE"

        elif s == "DONE":
            self._finished = True

    # ── save CSV + PNGs ───────────────────────────────────────────────────────
    def _finalize(self):
        if self._saved:
            return
        self._saved = True
        if not self.log_data:
            self.get_logger().info("Done (no data logged).")
            return
        try:
            self._save_csv()
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        try:
            self._save_plots()
        except Exception as e:
            self.get_logger().error(f"Plot save failed: {e}")
        for p in (CSV_PATH, PNG_XY, PNG_3D, PNG_ERR, PNG_Z):
            self.get_logger().info(f"  [{'OK  ' if os.path.exists(p) else 'MISS'}] {p}")
        self.get_logger().info("circle_path_node_v4 complete.")

    def _save_csv(self):
        keys = ["time", "actual_x", "actual_y", "actual_z", "expected_x", "expected_y", "expected_z"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")

    def _save_plots(self):
        d = {k: np.array([r[k] for r in self.log_data])
             for k in ("time", "actual_x", "actual_y", "actual_z",
                       "expected_x", "expected_y", "expected_z")}
        # keep only samples taken while circling (on the ring), for clean figures
        on = np.hypot(d["expected_x"] - self.cx, d["expected_y"] - self.cy) > (RADIUS * 0.5)
        if not np.any(on):
            on = np.ones(len(d["time"]), dtype=bool)
        th = np.linspace(0, 2 * math.pi, 400)
        ix, iy = self.cx + RADIUS * np.cos(th), self.cy + RADIUS * np.sin(th)

        # 1) XY top-down
        plt.figure(figsize=(7.5, 7.5))
        plt.plot([SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN],
                 [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN],
                 "--", color="grey", alpha=0.4, label="Safe zone")
        plt.plot(ix, iy, "b:", lw=2, label=f"Ideal circle r={RADIUS}")
        plt.scatter(d["actual_x"][on], d["actual_y"][on], s=10, color="green", alpha=0.8, label="Actual")
        plt.plot(self.cx, self.cy, "k+", ms=13, mew=2, label=f"Centre ({self.cx:.2f},{self.cy:.2f})")
        plt.title("cf231 circle - XY top-down (LPS)"); plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8); plt.grid(alpha=0.3); plt.axis("equal")
        plt.tight_layout(); plt.savefig(PNG_XY, dpi=120); plt.close()

        # 2) 3D
        fig = plt.figure(figsize=(8, 6.5)); a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, FLY_Z * np.ones_like(ix), "b:", lw=2, label="Ideal")
        a3.scatter(d["actual_x"][on], d["actual_y"][on], d["actual_z"][on], s=8,
                   color="green", alpha=0.7, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("cf231 circle - 3D"); a3.legend(fontsize=8)
        plt.tight_layout(); plt.savefig(PNG_3D, dpi=120); plt.close()

        # 3) Error analysis (radial + 3D error over time)
        r_actual = np.hypot(d["actual_x"] - self.cx, d["actual_y"] - self.cy)
        radial_err = (r_actual - RADIUS) * 100.0
        err3d = np.sqrt((d["actual_x"] - d["expected_x"])**2 +
                        (d["actual_y"] - d["expected_y"])**2 +
                        (d["actual_z"] - d["expected_z"])**2) * 100.0
        plt.figure(figsize=(9, 4.5))
        plt.plot(d["time"][on], err3d[on], color="tab:red", lw=1, label="3D tracking error")
        plt.plot(d["time"][on], radial_err[on], color="tab:blue", lw=1, alpha=0.7, label="Radial error")
        if np.any(on):
            plt.axhline(float(np.mean(err3d[on])), ls="--", color="grey", alpha=0.7,
                        label=f"mean 3D {np.mean(err3d[on]):.1f}cm")
        plt.title("cf231 circle - tracking error"); plt.xlabel("time (s)"); plt.ylabel("error (cm)")
        plt.legend(fontsize=8); plt.grid(alpha=0.3)
        plt.tight_layout(); plt.savefig(PNG_ERR, dpi=120); plt.close()

        # 4) Z theoretical vs real
        fig, (axz, axe) = plt.subplots(2, 1, figsize=(9, 6), sharex=True,
                                       gridspec_kw={"height_ratios": [2, 1]})
        axz.axhline(FLY_Z, ls="--", color="k", label=f"Theoretical z={FLY_Z}")
        axz.plot(d["time"], d["actual_z"], color="tab:green", lw=1, label="Actual z")
        axz.set_ylabel("Z (m)"); axz.set_title("cf231 - altitude theoretical vs real")
        axz.set_ylim(0, FLY_Z + 0.4); axz.legend(fontsize=8); axz.grid(alpha=0.3)
        zerr = (d["actual_z"] - d["expected_z"]) * 100.0
        axe.plot(d["time"], zerr, color="tab:red", lw=1, label="Z error")
        axe.axhline(float(np.mean(zerr)), ls="--", color="grey", alpha=0.7, label=f"mean {np.mean(zerr):.1f}cm")
        axe.set_xlabel("time (s)"); axe.set_ylabel("Z error (cm)"); axe.legend(fontsize=8); axe.grid(alpha=0.3)
        plt.tight_layout(); plt.savefig(PNG_Z, dpi=120); plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = CirclePathNodeV4()
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
