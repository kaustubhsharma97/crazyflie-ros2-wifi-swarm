#!/usr/bin/env python3
"""
square_launch_node.py — ROS2 Humble / rclpy
===========================================
Crazyflie 2.1 + Loco Positioning System (TDoA2, 8 anchors)
Kaustubh Sharma | Summer Intern, IIIT Delhi (Prof. Sanjit Kaul)
Lab: B-419, IRAS Hub (Robotics Lab), IIIT-Delhi

Pure rclpy + crazyflie_interfaces services (NO cflib).
Non-blocking state machine. Works UNCHANGED in Gazebo sim and on the real drone.

WHAT CHANGED IN THIS REVISION
-----------------------------
The old version hardcoded the first corner at (0.0, 1.0) — which is EXACTLY
anchor 0's position — and flew straight to it, driving the drone into the
anchor / the X=0 wall. This version is DYNAMIC: it reads where you place the
drone and builds a 2 m x 2 m square centred near that point, pulled inward via
dynamic_start.fit_center so the whole square always stays inside the anchor
hull. No absolute corners, no flying at an anchor.

Square geometry (side = 2 m) is unchanged from Raghav — only its placement is
now relative + hull-safe.

Outputs (in $HOME):
  square_trajectory_log.csv
  square_trajectory_xy_topdown.png
  square_trajectory_3d.png
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

FLY_Z           = 0.5
TAKEOFF_TIME    = 2.5
LPS_SETTLE_TIME = 3.0

# Edge interpolation (smooth continuous motion along each side)
POINTS_PER_EDGE = 15
EDGE_TIME       = 6.0

# ── Square geometry, defined RELATIVE to its own centre (0,0) ─────────────────
# 2 m side -> corners at +/-1 m. reach_x = reach_y = 1.0 (max |x|,|y| of a corner)
HALF = 1.0
LOCAL_CORNERS = [
    (-HALF, -HALF),   # C1
    ( HALF, -HALF),   # C2
    ( HALF,  HALF),   # C3
    (-HALF,  HALF),   # C4
    (-HALF, -HALF),   # close
]
REACH_X = REACH_Y = HALF

GOTO_SPEED = 0.5

CSV_PATH = os.path.expanduser("~/square_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/square_trajectory_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/square_trajectory_3d.png")


def build_edge_waypoints(corners, n_per_edge):
    wps = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        for k in range(n_per_edge):
            f = k / n_per_edge
            wps.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    wps.append(corners[-1])
    return wps


class SquareLaunchNode(Node):

    def __init__(self):
        super().__init__("square_launch_node")

        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None

        self.spawn_x = self.spawn_y = self.spawn_z = None
        self.cx = self.cy = None
        self.corners   = None      # world corners (filled in after spawn)
        self.waypoints = None

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.wp_i = 0
        self.wp_dt = EDGE_TIME / POINTS_PER_EDGE

        self._finished = False
        self._saved    = False

        self.create_subscription(
            PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)

        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info("SquareLaunchNode started (non-blocking, dynamic).")

    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.current_pose = (x, y, z)
        if self.t0 is not None and not self._finished:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            self.log_data.append({
                "time":       round(t, 3),
                "actual_x":   x, "actual_y": y, "actual_z": z,
                "expected_x": self.setpoint[0],
                "expected_y": self.setpoint[1],
                "expected_z": self.setpoint[2],
            })

    def _now(self):
        return self.get_clock().now().nanoseconds

    def _past(self):
        return self._now() > self._wait_until

    def _send_goto(self, x, y, z, duration):
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(duration)
        req.duration.nanosec = int((duration % 1) * 1e9)
        req.relative = False
        self.setpoint = (x, y, z)
        return self.goto_cli.call_async(req)

    def _tick(self):

        # ── Phase 0: wait for LPS, read placement, build the square around it ──
        if self.state == "WAIT_POSE":
            if self.current_pose is None:
                return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(
                    f"Initializing... waiting {LPS_SETTLE_TIME}s for LPS lock ...")
                return
            if not self._past():
                return

            self.spawn_x, self.spawn_y, self.spawn_z = self.current_pose
            self.t0 = self._now() / 1e9

            if not in_safe_zone(self.spawn_x, self.spawn_y):
                self.get_logger().error(
                    f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) is outside "
                    f"the safe zone X[{SAFE_X_MIN},{SAFE_X_MAX}] "
                    f"Y[{SAFE_Y_MIN},{SAFE_Y_MAX}]. Aborting (will still save).")
                self.state = "SAVE"
                return

            # Centre the square near the placement, pulled inward to fit the hull.
            self.cx, self.cy = fit_center(self.spawn_x, self.spawn_y,
                                          REACH_X, REACH_Y)
            self.corners = [(self.cx + lx, self.cy + ly)
                            for (lx, ly) in LOCAL_CORNERS]
            self.waypoints = build_edge_waypoints(self.corners, POINTS_PER_EDGE)
            self.get_logger().info(
                f"Placement ({self.spawn_x:.2f},{self.spawn_y:.2f}) -> square "
                f"centre ({self.cx:.2f},{self.cy:.2f}), 2 m side, fully in hull.")
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending is None:
                req = Arm.Request(); req.arm = True
                self._pending = self.arm_cli.call_async(req)
            elif self._pending.done():
                self.get_logger().info("Armed.")
                self._pending = None
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self._pending is None:
                self.setpoint = (self.spawn_x, self.spawn_y, FLY_Z)
                req = Takeoff.Request()
                req.height = FLY_Z
                req.duration.sec = int(TAKEOFF_TIME)
                req.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.0) * 1e9)
                self.get_logger().info(f"Taking off to {FLY_Z} m ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "APPROACH"

        # ── Phase 1.5: short hop from takeoff point to the first corner ──
        elif self.state == "APPROACH":
            if self._pending is None:
                sx, sy = self.corners[0]
                dist = math.hypot(sx - self.spawn_x, sy - self.spawn_y)
                dur = max(2.0, dist / GOTO_SPEED)
                self._pending = self._send_goto(sx, sy, FLY_Z, dur)
                self._wait_until = self._now() + int((dur + 1.0) * 1e9)
                self.get_logger().info(
                    f"To square start corner ({sx:.2f},{sy:.2f}) dist={dist:.2f}m")
            elif self._pending.done() and self._past():
                self._pending = None
                self.wp_i = 0
                self.state = "SQUARE"

        # ── Phase 2: trace the square ──
        elif self.state == "SQUARE":
            if self.wp_i >= len(self.waypoints):
                self.state = "SETTLE_FINAL"
                self._pending = None
                return
            if self._pending is None:
                wx, wy = self.waypoints[self.wp_i]
                cwx, cwy = clamp_xy(wx, wy)
                self._pending = self._send_goto(cwx, cwy, FLY_Z, self.wp_dt)
                self._wait_until = self._now() + int(self.wp_dt * 1e9)
                if self.wp_i % POINTS_PER_EDGE == 0:
                    side = self.wp_i // POINTS_PER_EDGE
                    tgt = self.corners[min(side + 1, len(self.corners) - 1)]
                    self.get_logger().info(
                        f"Edge {side+1}: toward corner ({tgt[0]:.2f},{tgt[1]:.2f})")
            elif self._pending.done() and self._past():
                self._pending = None
                self.wp_i += 1

        elif self.state == "SETTLE_FINAL":
            if self._pending is None:
                fx, fy = self.corners[-1]
                self._pending = self._send_goto(fx, fy, FLY_Z, 1.5)
                self._wait_until = self._now() + int(2.5 * 1e9)
                self.get_logger().info("Settling at final corner before landing ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "LAND"

        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request()
                req.height = 0.0
                req.duration.sec = 3; req.duration.nanosec = 0
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Mission complete. Landing.")
            elif self._pending.done() and self._past():
                self._pending = None
                self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._finalize()
            self.state = "DONE"

        elif self.state == "DONE":
            self._finished = True

    # ── OUTPUT (saved exactly once, each piece guarded) ─────────────────────────
    def _finalize(self):
        if self._saved:
            return
        self._saved = True
        self.get_logger().info("Finalising: saving CSV + plots ...")
        try:
            self._save_csv()
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        try:
            self._save_plots()
        except Exception as e:
            self.get_logger().error(f"plot save failed: {e}")
        try:
            from tracking_error import compute_tracking_error, format_report
            m = compute_tracking_error(self.log_data, fly_z=FLY_Z)
            if m:
                self.get_logger().info("\n" + format_report(m, "(square)"))
        except Exception as e:
            self.get_logger().warn(f"tracking_error metrics skipped: {e}")
        for p in (CSV_PATH, PNG_XY, PNG_3D):
            tag = "OK  " if os.path.exists(p) else "MISS"
            self.get_logger().info(f"  [{tag}] {p}")
        self.get_logger().info("Sequence complete.")

    def _save_csv(self):
        if not self.log_data:
            self.get_logger().warn("No data to save.")
            return
        keys = ["time", "actual_x", "actual_y", "actual_z",
                "expected_x", "expected_y", "expected_z"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV  -> {CSV_PATH} ({len(self.log_data)} rows)")

    def _save_plots(self):
        if not self.log_data or self.corners is None:
            self.get_logger().warn("No data/geometry — skipping plots.")
            return
        ax = np.array([d["actual_x"] for d in self.log_data])
        ay = np.array([d["actual_y"] for d in self.log_data])
        az = np.array([d["actual_z"] for d in self.log_data])
        ex = np.array([d["expected_x"] for d in self.log_data])
        ey = np.array([d["expected_y"] for d in self.log_data])

        ix = [c[0] for c in self.corners]
        iy = [c[1] for c in self.corners]

        # Keep samples whose expected setpoint lies on the square perimeter.
        xmin, xmax = self.cx - HALF, self.cx + HALF
        ymin, ymax = self.cy - HALF, self.cy + HALF
        def on_perimeter(exx, eyy):
            on_vert  = (abs(exx - xmin) < 0.08 or abs(exx - xmax) < 0.08) and (ymin - 0.08 <= eyy <= ymax + 0.08)
            on_horiz = (abs(eyy - ymin) < 0.08 or abs(eyy - ymax) < 0.08) and (xmin - 0.08 <= exx <= xmax + 0.08)
            return on_vert or on_horiz
        mask = np.array([on_perimeter(a, b) for a, b in zip(ex, ey)])
        if not np.any(mask):
            mask = np.ones(len(ex), dtype=bool)
        sax, say, saz = ax[mask], ay[mask], az[mask]

        bx = [SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN]
        by = [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN]

        plt.figure(figsize=(7, 7))
        plt.plot(bx, by, "r--", linewidth=1.0, alpha=0.6, label="Safety boundary")
        plt.plot(ix, iy, "b:", linewidth=2.5, label="Ideal square")
        plt.plot(sax, say, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(sax, say, s=14, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        for i, (cx, cy) in enumerate(self.corners[:-1]):
            plt.plot(cx, cy, "bs", markersize=7, zorder=6)
            plt.annotate(f"C{i+1}", (cx, cy), textcoords="offset points",
                         xytext=(8, 8), fontsize=9)
        if self.spawn_x is not None:
            plt.plot(self.spawn_x, self.spawn_y, "ko", markersize=9,
                     label=f"Start ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.title("Square — Ideal vs Actual (XY top-down, LPS coords)")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout()
        plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, FLY_Z * np.ones(len(ix)), "b:", linewidth=2.0, label="Ideal")
        a3.scatter(sax, say, saz, s=10, color="green", alpha=0.8, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Square — 3D")
        a3.legend()
        plt.tight_layout()
        plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = SquareLaunchNode()
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
