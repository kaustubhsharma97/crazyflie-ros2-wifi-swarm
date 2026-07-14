#!/usr/bin/env python3
"""
parabola_trajectory_node.py — ROS2 Humble / rclpy
=================================================
Crazyflie 2.1 + LPS (TDoA2, 8 anchors) | Kaustubh Sharma | IIIT-Delhi, Lab B-419

Parabolic arc from A (where you place the drone) to B.

DYNAMIC + SAFE: the arc now travels DISTANCE metres toward the anchor-volume
centre (dynamic_start.inward_unit), not a fixed +X — so from any placement it
heads INTO the hull, never at a wall. Rises PEAK_HEIGHT at the midpoint, returns
to cruise height at B. Robust save (shutdown out of timer callback, saved once).

Outputs (in $HOME): parabola_trajectory_log.csv, _xz_side.png, _3d.png
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

from dynamic_start import inward_unit, clamp_xy, in_safe_zone, \
    SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX

CF_NAME = "cf231"

FLY_Z        = 0.5
DISTANCE     = 1.5      # metres travelled (now toward the hull centre)
PEAK_HEIGHT  = 0.5
STEPS        = 60
SEG_TIME     = 0.25
LPS_SETTLE_TIME = 3.0

CSV_PATH = os.path.expanduser("~/parabola_trajectory_log.csv")
PNG_XZ   = os.path.expanduser("~/parabola_trajectory_xz_side.png")
PNG_3D   = os.path.expanduser("~/parabola_trajectory_3d.png")


class ParabolaNode(Node):

    def __init__(self):
        super().__init__("parabola_trajectory_node")
        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None
        self.start_x = self.start_y = self.start_z = 0.0
        self.ux, self.uy = 1.0, 0.0       # travel direction (filled after spawn)

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.step_i = 0
        self._finished = False
        self._saved = False

        self.create_subscription(PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)
        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        self.create_timer(0.1, self._tick)
        self.get_logger().info("ParabolaNode started (non-blocking, dynamic).")

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

    def _ideal_point(self, t):
        """(x, y, z) at normalised t in [0,1] along the inward travel direction."""
        x = self.start_x + t * DISTANCE * self.ux
        y = self.start_y + t * DISTANCE * self.uy
        z = self.start_z + PEAK_HEIGHT * 4.0 * t * (1.0 - t)
        return x, y, z

    def _tick(self):
        if self.state == "WAIT_POSE":
            if self.current_pose is None: return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(
                    f"Waiting {LPS_SETTLE_TIME}s for LPS estimator to settle ...")
                return
            if not self._past(): return
            self.start_x, self.start_y, _ = self.current_pose
            self.start_z = FLY_Z
            self.t0 = self._now() / 1e9
            if not in_safe_zone(self.start_x, self.start_y):
                self.get_logger().error(
                    f"Placement ({self.start_x:.2f},{self.start_y:.2f}) outside safe "
                    f"zone X[{SAFE_X_MIN},{SAFE_X_MAX}] Y[{SAFE_Y_MIN},{SAFE_Y_MAX}]. "
                    f"Aborting (will still save).")
                self.state = "SAVE"; return
            self.ux, self.uy = inward_unit(self.start_x, self.start_y)
            bx = self.start_x + DISTANCE * self.ux
            by = self.start_y + DISTANCE * self.uy
            self.get_logger().info(
                f"Point A: ({self.start_x:.2f},{self.start_y:.2f})  "
                f"Point B: ({bx:.2f},{by:.2f}) [toward hull centre]")
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
                self.setpoint = (self.start_x, self.start_y, FLY_Z)
                req = Takeoff.Request(); req.height = FLY_Z
                req.duration.sec = 2; req.duration.nanosec = 500_000_000
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int(3.5 * 1e9)
                self.get_logger().info(f"Taking off to {FLY_Z} m ...")
            elif self._pending.done() and self._past():
                self._pending = None
                if self.current_pose:
                    self.start_x, self.start_y, _ = self.current_pose
                    # re-derive direction from the actual hover point
                    self.ux, self.uy = inward_unit(self.start_x, self.start_y)
                self.step_i = 0
                self.get_logger().info("Phase: Parabola A -> B ...")
                self.state = "PARABOLA"

        elif self.state == "PARABOLA":
            if self.step_i > STEPS:
                self.state = "SETTLE_B"; self._pending = None; return
            t = self.step_i / STEPS
            wx, wy, wz = self._ideal_point(t)
            cwx, cwy = clamp_xy(wx, wy)
            self._send_goto(cwx, cwy, wz, SEG_TIME)
            if self.step_i % 10 == 0:
                self.get_logger().info(
                    f"Step {self.step_i:02d}/{STEPS} x={cwx:.2f} y={cwy:.2f} z={wz:.2f}")
            self.step_i += 1

        elif self.state == "SETTLE_B":
            if self._pending is None:
                bx = self.start_x + DISTANCE * self.ux
                by = self.start_y + DISTANCE * self.uy
                bx, by = clamp_xy(bx, by)
                self._pending = self._send_goto(bx, by, FLY_Z, 1.5)
                self._wait_until = self._now() + int(2.5 * 1e9)
                self.get_logger().info("Settling at point B ...")
            elif self._pending.done() and self._past():
                self._pending = None; self.state = "LAND"

        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request(); req.height = 0.0
                req.duration.sec = 3; req.duration.nanosec = 0
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Landing at B ...")
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
            if m: self.get_logger().info("\n" + format_report(m, "(parabola)"))
        except Exception as e:
            self.get_logger().warn(f"tracking_error metrics skipped: {e}")
        for p in (CSV_PATH, PNG_XZ, PNG_3D):
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
        if not self.log_data: return
        ax = np.array([d["actual_x"] for d in self.log_data])
        ay = np.array([d["actual_y"] for d in self.log_data])
        az = np.array([d["actual_z"] for d in self.log_data])
        ez = np.array([d["expected_z"] for d in self.log_data])

        on_arc = ez > (FLY_Z + 0.02)
        # horizontal distance travelled from A (works for any travel direction)
        dist_act = np.hypot(ax - self.start_x, ay - self.start_y)
        cad, caz = dist_act[on_arc], az[on_arc]

        tt = np.linspace(0, 1, 300)
        idist = tt * DISTANCE
        iz = self.start_z + PEAK_HEIGHT * 4.0 * tt * (1.0 - tt)

        plt.figure(figsize=(8, 5))
        plt.plot(idist, iz, "b:", linewidth=2.5, label="Ideal parabola")
        plt.plot(cad, caz, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(cad, caz, s=16, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        plt.plot(0.0, self.start_z, "ko", markersize=9, label="A (start)", zorder=6)
        plt.plot(DISTANCE, self.start_z, "ms", markersize=9, label="B (end)", zorder=6)
        plt.title("Parabola A -> B — Ideal vs Actual (side profile)")
        plt.xlabel("Distance travelled (m)"); plt.ylabel("Z (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3)
        plt.tight_layout(); plt.savefig(PNG_XZ, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XZ}")

        bx = self.start_x + DISTANCE * self.ux
        by = self.start_y + DISTANCE * self.uy
        ix = self.start_x + tt * DISTANCE * self.ux
        iy = self.start_y + tt * DISTANCE * self.uy
        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, iz, "b:", linewidth=2.0, label="Ideal")
        a3.scatter(ax[on_arc], ay[on_arc], caz, s=10, color="green", alpha=0.8,
                   label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Parabola — 3D"); a3.legend()
        plt.tight_layout(); plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = ParabolaNode()
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
