#!/usr/bin/env python3
"""
figure8_node_sim.py — ROS2 Humble / rclpy (sim-compatible, streamed go_to)
==========================================================================
Crazyflie 2.1 + LPS (TDoA2, 8 anchors) | Kaustubh Sharma | IIIT-Delhi, Lab B-419

DYNAMIC: the figure-8 (lemniscate of Gerono) is centred near wherever you place
the drone, via dynamic_start.fit_center, so it stays inside the anchor hull.
Robust save (shutdown out of the timer callback, saved once, [OK]/[MISS] log).

  x(t) = cx + A*sin(t),  y(t) = cy + B*sin(t)*cos(t),  t: 0 -> 2*pi

Outputs (in $HOME): figure8_trajectory_log.csv, _xy_topdown.png, _3d.png
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

FLY_Z           = 0.6
TAKEOFF_TIME    = 2.5
LPS_SETTLE_TIME = 3.0

A        = 1.0     # X amplitude -> 2 m wide
B        = 1.0     # Y amplitude -> lobes reach +/-0.5 m
FIG8_DURATION = 24.0
DT       = 0.3
GOTO_SPEED = 0.5
REACH_X = A         # x-extent of the lemniscate
REACH_Y = 0.5 * B   # y-extent (max of sin*cos is 0.5)

CSV_PATH = os.path.expanduser("~/figure8_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/figure8_trajectory_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/figure8_trajectory_3d.png")


def fig8_point(t, cx, cy):
    return cx + A * math.sin(t), cy + B * math.sin(t) * math.cos(t)


class Figure8Node(Node):

    def __init__(self):
        super().__init__("figure8_node")
        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None
        self.spawn_x = self.spawn_y = self.spawn_z = None
        self.cx = self.cy = None

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.step_i = 0
        self.total_steps = int(FIG8_DURATION / DT)
        self._finished = False
        self._saved = False

        self.create_subscription(PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)
        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(DT, self._tick)
        self.get_logger().info("Figure8Node started (non-blocking, dynamic).")

    def _pose_cb(self, msg: PoseStamped):
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
            self.cx, self.cy = fit_center(self.spawn_x, self.spawn_y, REACH_X, REACH_Y)
            self.get_logger().info(
                f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) -> figure-8 "
                f"centre ({self.cx:.2f},{self.cy:.2f}), in hull.")
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
                self.setpoint = (self.spawn_x, self.spawn_y, FLY_Z)
                req = Takeoff.Request(); req.height = FLY_Z
                req.duration.sec = int(TAKEOFF_TIME); req.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.0) * 1e9)
                self.get_logger().info(f"Taking off to {FLY_Z} m ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.state = "APPROACH"

        elif self.state == "APPROACH":
            if self._pending is None:
                ex, ey = fig8_point(0.0, self.cx, self.cy)   # entry = centre (t=0)
                dist = math.hypot(ex - self.spawn_x, ey - self.spawn_y)
                dur = max(2.0, dist / GOTO_SPEED)
                self._pending = self._send_goto(ex, ey, FLY_Z, dur)
                self._wait_until = self._now() + int((dur + 1.0) * 1e9)
                self.get_logger().info("Approaching figure-8 entry point ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.step_i = 0
                self.get_logger().info(f"Tracing figure-8: A={A} B={B} {FIG8_DURATION}s")
                self.state = "FIG8"

        elif self.state == "FIG8":
            if self.step_i >= self.total_steps:
                self.state = "RETURN_CENTER"; self._pending = None; return
            t = (self.step_i / self.total_steps) * (2 * math.pi)
            wx, wy = fig8_point(t, self.cx, self.cy)
            cwx, cwy = clamp_xy(wx, wy)
            self._send_goto(cwx, cwy, FLY_Z, DT)
            self.step_i += 1

        elif self.state == "RETURN_CENTER":
            if self._pending is None:
                self._pending = self._send_goto(self.cx, self.cy, FLY_Z, 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)
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
            m = compute_tracking_error(self.log_data, fly_z=FLY_Z)
            if m: self.get_logger().info("\n" + format_report(m, "(figure-8)"))
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

        tt = np.linspace(0, 2*math.pi, 600)
        ix = self.cx + A * np.sin(tt)
        iy = self.cy + B * np.sin(tt) * np.cos(tt)

        ideal_pts = np.column_stack([ix, iy])
        on_fig8 = np.array([
            np.min(np.hypot(ideal_pts[:, 0] - exx, ideal_pts[:, 1] - eyy)) < 0.15
            for exx, eyy in zip(ex, ey)])
        if not np.any(on_fig8): on_fig8 = np.ones(len(ex), dtype=bool)
        fax, fay = ax[on_fig8], ay[on_fig8]

        bx = [SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN]
        by = [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN]

        plt.figure(figsize=(7, 7))
        plt.plot(bx, by, "r--", linewidth=1.0, alpha=0.6, label="Safety boundary")
        plt.plot(ix, iy, "b:", linewidth=2.5, label="Ideal figure-8")
        plt.plot(fax, fay, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(fax, fay, s=14, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        if self.spawn_x is not None:
            plt.plot(self.spawn_x, self.spawn_y, "ko", markersize=9,
                     label=f"Start ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.plot(self.cx, self.cy, "m+", markersize=14, markeredgewidth=2.5,
                 label=f"Centre ({self.cx:.2f},{self.cy:.2f})", zorder=6)
        plt.title("Figure-8 — Ideal vs Actual (XY top-down, LPS coords)")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout(); plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, FLY_Z*np.ones_like(ix), "b:", linewidth=2.0, label="Ideal")
        a3.scatter(fax, fay, az[on_fig8], s=10, color="green", alpha=0.8, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Figure-8 — 3D"); a3.legend()
        plt.tight_layout(); plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Node()
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
