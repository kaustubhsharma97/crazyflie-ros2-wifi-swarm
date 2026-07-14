#!/usr/bin/env python3
"""
spiral_node.py — ROS2 Humble / rclpy
====================================
Crazyflie 2.1 + LPS (TDoA2, 8 anchors) | Kaustubh Sharma | IIIT-Delhi, Lab B-419

DYNAMIC: Archimedean spiral centred near wherever you place the drone (via
dynamic_start.fit_center) so the outer turns stay inside the anchor hull.
Robust save (shutdown out of the timer callback, saved once, [OK]/[MISS] log).

  r = R_MAX*(theta/theta_max),  x = cx + r cos,  y = cy + r sin,  z climbs.

Outputs (in $HOME): spiral_trajectory_log.csv, _xy_topdown.png, _3d.png
"""

import os
import sys
import csv
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           fit_center, clamp_xy, in_safe_zone)

CF_NAME = "cf231"

TAKEOFF_TIME    = 2.5
LPS_SETTLE_TIME = 3.0
GOTO_SPEED      = 0.5

R_MAX    = 1.0
TURNS    = 3
Z_START  = 0.4
Z_END    = 0.8
TOTAL_TIME = 24.0
DT       = 0.3
REACH = R_MAX

THETA_MAX = TURNS * 2 * math.pi

CSV_PATH = os.path.expanduser("~/spiral_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/spiral_trajectory_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/spiral_trajectory_3d.png")


def spiral_point(frac, cx, cy):
    theta = frac * THETA_MAX
    r = R_MAX * (theta / THETA_MAX)
    x = cx + r * math.cos(theta)
    y = cy + r * math.sin(theta)
    z = Z_START + (Z_END - Z_START) * (theta / THETA_MAX)
    return x, y, z


class SpiralNode(Node):

    def __init__(self):
        super().__init__("spiral_node")
        self.current_pose = None
        self.setpoint     = (0.0, 0.0, Z_START)
        self.log_data     = []
        self.t0           = None
        self.spawn_x = self.spawn_y = self.spawn_z = None
        self.cx = self.cy = None

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.step_i = 0
        self.total_steps = int(TOTAL_TIME / DT)
        self._finished = False
        self._saved = False

        self.create_subscription(PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)
        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(DT, self._tick)
        self.get_logger().info("SpiralNode started (non-blocking, dynamic).")

    def _pose_cb(self, msg):
        x = msg.pose.position.x; y = msg.pose.position.y; z = msg.pose.position.z
        self.current_pose = (x, y, z)
        if self.t0 is not None and not self._finished:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            self.log_data.append({
                "time": round(t, 3), "actual_x": x, "actual_y": y, "actual_z": z,
                "expected_x": self.setpoint[0], "expected_y": self.setpoint[1],
                "expected_z": self.setpoint[2]})

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until

    def _send_goto(self, x, y, z, duration):
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(duration); req.duration.nanosec = int((duration % 1) * 1e9)
        req.relative = False
        self.setpoint = (x, y, z)
        return self.goto_cli.call_async(req)

    def _tick(self):
        if self.state == "WAIT_POSE":
            if self.current_pose is None: return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(f"Waiting {LPS_SETTLE_TIME}s for LPS lock ...")
                return
            if not self._past(): return
            self.spawn_x, self.spawn_y, self.spawn_z = self.current_pose
            self.t0 = self._now() / 1e9
            if not in_safe_zone(self.spawn_x, self.spawn_y):
                self.get_logger().error(
                    f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) outside safe "
                    f"zone X[{SAFE_X_MIN},{SAFE_X_MAX}] Y[{SAFE_Y_MIN},{SAFE_Y_MAX}]. "
                    f"Aborting (will still save).")
                self.state = "SAVE"; return
            self.cx, self.cy = fit_center(self.spawn_x, self.spawn_y, REACH, REACH)
            self.get_logger().info(
                f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) -> spiral "
                f"centre ({self.cx:.2f},{self.cy:.2f}), R_max={R_MAX}, in hull.")
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending is None:
                req = Arm.Request(); req.arm = True
                self._pending = self.arm_cli.call_async(req)
            elif self._pending.done():
                self.get_logger().info("Armed."); self._pending = None
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self._pending is None:
                self.setpoint = (self.spawn_x, self.spawn_y, Z_START)
                req = Takeoff.Request(); req.height = Z_START
                req.duration.sec = int(TAKEOFF_TIME); req.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.0) * 1e9)
                self.get_logger().info(f"Taking off to {Z_START} m ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.state = "APPROACH"

        elif self.state == "APPROACH":
            if self._pending is None:
                self._pending = self._send_goto(self.cx, self.cy, Z_START, 3.0)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Moving to spiral start (centre) ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.step_i = 0
                self.get_logger().info(
                    f"Tracing spiral: R_max={R_MAX} turns={TURNS} climb {Z_START}->{Z_END} m")
                self.state = "SPIRAL"

        elif self.state == "SPIRAL":
            if self.step_i >= self.total_steps:
                self.state = "RETURN_CENTER"; self._pending = None; return
            frac = self.step_i / self.total_steps
            wx, wy, wz = spiral_point(frac, self.cx, self.cy)
            cwx, cwy = clamp_xy(wx, wy)
            self._send_goto(cwx, cwy, wz, DT)
            if self.step_i % 20 == 0:
                self.get_logger().info(f"Spiral {self.step_i}/{self.total_steps}")
            self.step_i += 1

        elif self.state == "RETURN_CENTER":
            if self._pending is None:
                self._pending = self._send_goto(self.cx, self.cy, Z_START, 2.5)
                self._wait_until = self._now() + int(3.5 * 1e9)
                self.get_logger().info("Returning to centre to settle ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.state = "LAND"

        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request(); req.height = 0.0
                req.duration.sec = 3; req.duration.nanosec = 0
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Landing ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._finalize(); self.state = "DONE"

        elif self.state == "DONE":
            self._finished = True

    def _finalize(self):
        if self._saved: return
        self._saved = True
        self.get_logger().info("Finalising: saving CSV + plots ...")
        try: self._save_csv()
        except Exception as e: self.get_logger().error(f"CSV save failed: {e}")
        try: self._save_plots()
        except Exception as e: self.get_logger().error(f"plot save failed: {e}")
        try:
            from tracking_error import compute_tracking_error, format_report
            m = compute_tracking_error(self.log_data, fly_z=(Z_START + Z_END) / 2)
            if m: self.get_logger().info("\n" + format_report(m, "(spiral)"))
        except Exception as e:
            self.get_logger().warn(f"tracking_error metrics skipped: {e}")
        for p in (CSV_PATH, PNG_XY, PNG_3D):
            self.get_logger().info(f"  [{'OK  ' if os.path.exists(p) else 'MISS'}] {p}")
        self.get_logger().info("Sequence complete.")

    def _save_csv(self):
        if not self.log_data:
            self.get_logger().warn("No data to save."); return
        keys = ["time", "actual_x", "actual_y", "actual_z",
                "expected_x", "expected_y", "expected_z"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV  -> {CSV_PATH} ({len(self.log_data)} rows)")

    def _save_plots(self):
        if not self.log_data or self.cx is None: return
        ax = np.array([d["actual_x"] for d in self.log_data])
        ay = np.array([d["actual_y"] for d in self.log_data])
        az = np.array([d["actual_z"] for d in self.log_data])
        ex = np.array([d["expected_x"] for d in self.log_data])
        ey = np.array([d["expected_y"] for d in self.log_data])

        ff = np.linspace(0, 1, 600)
        ipts = np.array([spiral_point(f, self.cx, self.cy) for f in ff])
        ix, iy, iz = ipts[:, 0], ipts[:, 1], ipts[:, 2]

        r_exp = np.hypot(ex - self.cx, ey - self.cy)
        on_spiral = r_exp > 0.1
        if not np.any(on_spiral): on_spiral = np.ones(len(ex), dtype=bool)
        sax, say, saz = ax[on_spiral], ay[on_spiral], az[on_spiral]

        bx = [SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN]
        by = [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN]

        plt.figure(figsize=(7, 7))
        plt.plot(bx, by, "r--", linewidth=1.0, alpha=0.6, label="Safety boundary")
        plt.plot(ix, iy, "b:", linewidth=2.5, label="Ideal spiral")
        plt.plot(sax, say, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(sax, say, s=14, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        if self.spawn_x is not None:
            plt.plot(self.spawn_x, self.spawn_y, "ko", markersize=9,
                     label=f"Start ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.plot(self.cx, self.cy, "m+", markersize=14, markeredgewidth=2.5,
                 label=f"Centre ({self.cx:.2f},{self.cy:.2f})", zorder=6)
        plt.title("Spiral — Ideal vs Actual (XY top-down, LPS coords)")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout(); plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, iz, "b:", linewidth=2.0, label="Ideal")
        a3.scatter(sax, say, saz, s=10, color="green", alpha=0.8, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Spiral — 3D (note the climb)"); a3.legend()
        plt.tight_layout(); plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = SpiralNode()
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
