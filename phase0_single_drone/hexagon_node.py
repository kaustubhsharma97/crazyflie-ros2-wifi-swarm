#!/usr/bin/env python3
"""
hexagon_node.py — ROS2 Humble / rclpy
=====================================
Crazyflie 2.1 + LPS (TDoA2, 8 anchors) | Kaustubh Sharma | IIIT-Delhi, Lab B-419

DYNAMIC: reads where you place the drone and centres the hexagon near that
point via dynamic_start.fit_center, so the whole hexagon stays inside the
anchor hull wherever you set it down. Robust save (shutdown out of the timer
callback, saved exactly once, [OK]/[MISS] confirmation).

Outputs (in $HOME): hexagon_trajectory_log.csv, _xy_topdown.png, _3d.png
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
GOTO_SPEED      = 0.5

RADIUS   = 1.0            # circumradius (centre-to-vertex)
N_SIDES  = 6
POINTS_PER_EDGE = 12
EDGE_TIME = 2.0
START_ANGLE = math.pi / 6.0
REACH = RADIUS           # max centre-to-vertex distance -> fit guarantee

CSV_PATH = os.path.expanduser("~/hexagon_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/hexagon_trajectory_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/hexagon_trajectory_3d.png")


def hexagon_corners(cx, cy):
    corners = []
    for k in range(N_SIDES):
        ang = START_ANGLE + k * (2 * math.pi / N_SIDES)
        corners.append((cx + RADIUS * math.cos(ang), cy + RADIUS * math.sin(ang)))
    corners.append(corners[0])
    return corners


def build_edge_waypoints(corners):
    wps = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]; x1, y1 = corners[i + 1]
        for k in range(POINTS_PER_EDGE):
            f = k / POINTS_PER_EDGE
            wps.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    wps.append(corners[-1])
    return wps


class HexagonNode(Node):

    def __init__(self):
        super().__init__("hexagon_node")
        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None
        self.spawn_x = self.spawn_y = self.spawn_z = None
        self.cx = self.cy = None
        self.corners = self.waypoints = None
        self.wp_i = 0
        self.wp_dt = EDGE_TIME / POINTS_PER_EDGE

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self._finished = False
        self._saved = False

        self.create_subscription(PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)
        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info("HexagonNode started (non-blocking, dynamic).")

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
            self.corners = hexagon_corners(self.cx, self.cy)
            self.waypoints = build_edge_waypoints(self.corners)
            self.get_logger().info(
                f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) -> hexagon "
                f"centre ({self.cx:.2f},{self.cy:.2f}), R={RADIUS}, in hull.")
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
                ex, ey = self.corners[0]
                dist = math.hypot(ex - self.spawn_x, ey - self.spawn_y)
                dur = max(2.0, dist / GOTO_SPEED)
                self._pending = self._send_goto(ex, ey, FLY_Z, dur)
                self._wait_until = self._now() + int((dur + 1.0) * 1e9)
                self.get_logger().info("Approaching hexagon start vertex ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.wp_i = 0
                self.get_logger().info(f"Tracing hexagon: R={RADIUS} {N_SIDES} sides")
                self.state = "HEX"

        elif self.state == "HEX":
            if self.wp_i >= len(self.waypoints):
                self.state = "RETURN_CENTER"; self._pending = None; return
            if self._pending is None:
                wx, wy = self.waypoints[self.wp_i]
                cwx, cwy = clamp_xy(wx, wy)
                self._pending = self._send_goto(cwx, cwy, FLY_Z, self.wp_dt)
                self._wait_until = self._now() + int(self.wp_dt * 1e9)
                if self.wp_i % POINTS_PER_EDGE == 0:
                    self.get_logger().info(f"Edge {self.wp_i // POINTS_PER_EDGE + 1}/{N_SIDES}")
            elif self._pending.done() and self._past():
                self._pending = None; self.wp_i += 1

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
            if m: self.get_logger().info("\n" + format_report(m, "(hexagon)"))
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
        if not self.log_data or self.corners is None: return
        ax = np.array([d["actual_x"] for d in self.log_data])
        ay = np.array([d["actual_y"] for d in self.log_data])
        az = np.array([d["actual_z"] for d in self.log_data])
        ex = np.array([d["expected_x"] for d in self.log_data])
        ey = np.array([d["expected_y"] for d in self.log_data])

        on_hex = np.hypot(ex - self.cx, ey - self.cy) > (RADIUS * 0.5)
        if not np.any(on_hex): on_hex = np.ones(len(ex), dtype=bool)
        hax, hay = ax[on_hex], ay[on_hex]

        ix = [c[0] for c in self.corners]; iy = [c[1] for c in self.corners]
        bx = [SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN]
        by = [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN]

        plt.figure(figsize=(7, 7))
        plt.plot(bx, by, "r--", linewidth=1.0, alpha=0.6, label="Safety boundary")
        plt.plot(ix, iy, "b:", linewidth=2.5, label="Ideal hexagon")
        plt.plot(hax, hay, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(hax, hay, s=14, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        if self.spawn_x is not None:
            plt.plot(self.spawn_x, self.spawn_y, "ko", markersize=9,
                     label=f"Start ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.plot(self.cx, self.cy, "m+", markersize=14, markeredgewidth=2.5,
                 label=f"Centre ({self.cx:.2f},{self.cy:.2f})", zorder=6)
        plt.title("Hexagon — Ideal vs Actual (XY top-down, LPS coords)")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout(); plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, [FLY_Z]*len(ix), "b:", linewidth=2.0, label="Ideal")
        a3.scatter(hax, hay, az[on_hex], s=10, color="green", alpha=0.8, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Hexagon — 3D"); a3.legend()
        plt.tight_layout(); plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = HexagonNode()
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
