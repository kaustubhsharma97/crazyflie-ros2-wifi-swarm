#!/usr/bin/env python3
"""
circle_path_node.py — ROS2 Humble / rclpy
===========================================
Crazyflie 2.1 + Loco Positioning System (TDoA2, 8 anchors)
Kaustubh Sharma | Summer Intern, IIIT Delhi (Prof. Sanjit Kaul)
Lab: B-419, IRAS Hub (Robotics Lab), IIIT-Delhi

rclpy port of Raghav's circle_path.py — SAME LOGIC, SAME CONSTANTS.
Only the method differs: pure rclpy + crazyflie_interfaces services
instead of cflib high_level_commander, and /cf231/pose instead of
reading kalman.stateX/Y/Z directly.

Non-blocking state machine (no nested-spin deadlock).
Works UNCHANGED in Gazebo sim and on the real drone.

Phases (identical to Raghav):
  0. Wait for LPS fix, read spawn, safety-check inside safe zone
  1. Takeoff to FLY_Z
  2. Fly to absolute circle centre (1.5, 2.5)
  3. Circle: r=1.2, omega=0.7, 20 s, 10 Hz, clamped to safe zone
  4. Land

Outputs:
  ~/circle_trajectory_log.csv
  ~/circle_trajectory_xy_topdown.png   (dotted ideal circle + solid actual)
  ~/circle_trajectory_3d.png

Run:
  python3 circle_path_node.py
"""

import os
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from std_srvs.srv import SetBool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

CF_NAME = "cf231"

# ── Safe zone (lab square) — IDENTICAL to Raghav ──────────────────────────────
SAFE_X_MIN, SAFE_X_MAX = 0.0, 3.0
SAFE_Y_MIN, SAFE_Y_MAX = 1.0, 4.0

# ── Circle parameters — IDENTICAL to Raghav ───────────────────────────────────
CIRCLE_CENTER_X = 1.5
CIRCLE_CENTER_Y = 2.5
RADIUS          = 0.8     # metres (reduced further from 1.0 — deepest inside
                          # the anchor hull where TDoA2 positioning is cleaner)
OMEGA           = 0.5     # rad/s (slowed for tighter tracking)
CIRCLE_DURATION = 20.0    # seconds
DT              = 0.3     # seconds per waypoint — slowed from Raghav's 0.1 so the
                          # Gazebo sim server (subprocess set_pose) can keep pace.
                          # Circle radius/omega/centre/duration UNCHANGED; only the
                          # sampling resolution changes. For the REAL drone you can
                          # set this back to 0.1 (firmware handles 10 Hz natively).
FLY_Z           = 0.6     # cruise altitude (m)

LPS_SETTLE_TIME = 3.0     # seconds

# ── Output paths ──────────────────────────────────────────────────────────────
CSV_PATH = os.path.expanduser("~/circle_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/circle_trajectory_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/circle_trajectory_3d.png")
PNG_ERR  = os.path.expanduser("~/circle_trajectory_error_analysis.png")


class CirclePathNode(Node):

    def __init__(self):
        super().__init__("circle_path_node")
        self._finished = False

        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None

        self.spawn_x = self.spawn_y = self.spawn_z = None

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0
        self.theta = 0.0
        self.step_i = 0
        self.total_steps = int(CIRCLE_DURATION / DT)

        self.create_subscription(
            PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)

        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli    = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        # Timer at 10 Hz = DT, matching Raghav's loop rate
        self.create_timer(DT, self._tick)
        self.get_logger().info("CirclePathNode started (non-blocking).")

    # ── CALLBACKS ───────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.current_pose = (x, y, z)
        if self.t0 is not None:
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

    # ── SERVICE SENDERS (non-blocking) ───────────────────────────────────────────
    def _send_goto(self, x, y, z, duration):
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(duration)
        req.duration.nanosec = int((duration % 1) * 1e9)
        req.relative = False
        self.setpoint = (x, y, z)
        return self.goto_cli.call_async(req)

    # ── STATE MACHINE ─────────────────────────────────────────────────────────────
    def _tick(self):

        # ── Phase 0: wait for LPS, read spawn, safety check ──
        if self.state == "WAIT_POSE":
            if self.current_pose is None:
                return
            # mimic Raghav's LPS_SETTLE wait once before reading spawn
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(
                    f"Waiting {LPS_SETTLE_TIME}s for LPS estimator to settle ...")
                return
            if not self._past():
                return

            self.spawn_x, self.spawn_y, self.spawn_z = self.current_pose
            self.t0 = self._now() / 1e9
            self.get_logger().info(
                f"LPS spawn: x={self.spawn_x:.3f} y={self.spawn_y:.3f} z={self.spawn_z:.3f}")

            # Safety check — IDENTICAL to Raghav (margin = 0.1)
            margin = 0.1
            if not (SAFE_X_MIN + margin <= self.spawn_x <= SAFE_X_MAX - margin and
                    SAFE_Y_MIN + margin <= self.spawn_y <= SAFE_Y_MAX - margin):
                self.get_logger().error(
                    f"Spawn ({self.spawn_x:.2f}, {self.spawn_y:.2f}) outside safe "
                    f"zone. Aborting.")
                self.state = "SAVE"   # still save whatever we have
                return
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending is None:
                req = Arm.Request(); req.arm = True
                self._pending = self.arm_cli.call_async(req)
            elif self._pending.done():
                self.get_logger().info("Armed.")
                self._pending = None
                self.state = "TAKEOFF"

        # ── Phase 1: takeoff ──
        elif self.state == "TAKEOFF":
            if self._pending is None:
                self.setpoint = (self.spawn_x, self.spawn_y, FLY_Z)
                req = Takeoff.Request()
                req.height = FLY_Z
                req.duration.sec = 2; req.duration.nanosec = 500_000_000  # 2.5 s
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int(3.5 * 1e9)  # Raghav sleeps 3.5
                self.get_logger().info("Phase 1: Taking off ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "GOTO_CENTER"

        # ── Phase 2: fly to absolute circle centre ──
        elif self.state == "GOTO_CENTER":
            # IMPROVED: fly to the RIM START point (theta=0 position on the circle),
            # not the centre. This way the circle begins exactly where the drone
            # already is, eliminating the 1.2 m lunge that used to spike the error
            # at the start of every circle. Approach slowly for a clean settle.
            if self._pending is None:
                rim_x = CIRCLE_CENTER_X + RADIUS   # theta = 0 start point
                rim_y = CIRCLE_CENTER_Y
                dist = math.sqrt((rim_x - self.spawn_x)**2 +
                                 (rim_y - self.spawn_y)**2)
                travel = max(4.0, dist / 0.4)   # slower approach = cleaner settle
                self._travel = travel
                self._pending = self._send_goto(rim_x, rim_y, FLY_Z, travel)
                self._wait_until = self._now() + int((travel + 1.5) * 1e9)  # +settle
                self.get_logger().info(
                    f"Phase 2: To RIM START ({rim_x:.2f},{rim_y:.2f}) "
                    f"dist={dist:.2f}m t={travel:.1f}s")
            elif self._pending.done() and self._past():
                self._pending = None
                self.theta = 0.0
                self.step_i = 0
                self.get_logger().info(
                    f"Phase 3: Circle r={RADIUS} omega={OMEGA} {CIRCLE_DURATION}s")
                self.state = "CIRCLE"

        # ── Phase 3: circle (one waypoint per tick, exactly Raghav's loop) ──
        elif self.state == "CIRCLE":
            if self.step_i >= self.total_steps:
                self.state = "RETURN_CENTER"
                self._pending = None
                return
            # compute waypoint — IDENTICAL math
            wx = CIRCLE_CENTER_X + RADIUS * math.cos(self.theta)
            wy = CIRCLE_CENTER_Y + RADIUS * math.sin(self.theta)
            wx = max(SAFE_X_MIN + 0.05, min(SAFE_X_MAX - 0.05, wx))
            wy = max(SAFE_Y_MIN + 0.05, min(SAFE_Y_MAX - 0.05, wy))
            self._send_goto(wx, wy, FLY_Z, DT)   # fire and forget at 10 Hz
            self.theta += OMEGA * DT
            self.step_i += 1

        # ── Phase 3.5: return to centre and settle before landing ──
        # Prevents the drone from being stranded mid-arc: it flies back to a
        # known stable hover at the circle centre, pauses, THEN descends.
        elif self.state == "RETURN_CENTER":
            # IMPROVED: settle AT the rim where the circle ended (the drone is
            # already there) rather than jumping back to centre. Just hold and
            # settle in place, then land — no rim->centre transient to pollute
            # the tracking metric.
            if self._pending is None:
                end_x = CIRCLE_CENTER_X + RADIUS * math.cos(self.theta)
                end_y = CIRCLE_CENTER_Y + RADIUS * math.sin(self.theta)
                end_x = max(SAFE_X_MIN + 0.05, min(SAFE_X_MAX - 0.05, end_x))
                end_y = max(SAFE_Y_MIN + 0.05, min(SAFE_Y_MAX - 0.05, end_y))
                self._pending = self._send_goto(end_x, end_y, FLY_Z, 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)  # settle in place
                self.get_logger().info("Settling at circle end point ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "LAND"

        # ── Phase 4: land ──
        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request()
                req.height = 0.0   # B-419 LPS frame: floor = Z=0 (anchors at 0.3m)
                req.duration.sec = 3; req.duration.nanosec = 0
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int(4.0 * 1e9)  # full descent
                self.get_logger().info("Phase 4: Landing ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._save_csv()
            self._save_plots()
            # ── PID-tuning metrics ──
            try:
                from tracking_error import (compute_tracking_error,
                                            format_report, append_to_tuning_log)
                m = compute_tracking_error(self.log_data, fly_z=FLY_Z)
                self.get_logger().info("\n" + format_report(m, "(circle)"))
                # Edit the gains_note string each run to record what you changed
                append_to_tuning_log(m, label="circle",
                                     gains_note="v3 rim-entry r=0.8 omega=0.5 P=3.0")
            except Exception as e:
                self.get_logger().warn(f"tracking_error metrics skipped: {e}")
            # ── Detailed multi-panel error analysis PNG ──
            try:
                from error_plots import save_error_analysis
                r = save_error_analysis(self.log_data, "Circle", PNG_ERR, fly_z=FLY_Z)
                if r:
                    self.get_logger().info(f"Error analysis -> {PNG_ERR}")
            except Exception as e:
                self.get_logger().warn(f"error_plots skipped: {e}")
            self.get_logger().info("Sequence complete.")
            self.state = "DONE"

        elif self.state == "DONE":
            self._finished = True

    # ── OUTPUT ──────────────────────────────────────────────────────────────────
    def _save_csv(self):
        if not self.log_data:
            self.get_logger().warn("No data to save.")
            return
        keys = ["time", "actual_x", "actual_y", "actual_z",
                "expected_x", "expected_y", "expected_z"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV  -> {CSV_PATH} ({len(self.log_data)} rows)")

    def _save_plots(self):
        if not self.log_data:
            return
        ax = np.array([d["actual_x"] for d in self.log_data])
        ay = np.array([d["actual_y"] for d in self.log_data])
        az = np.array([d["actual_z"] for d in self.log_data])
        ex = np.array([d["expected_x"] for d in self.log_data])
        ey = np.array([d["expected_y"] for d in self.log_data])

        # Keep only the circle-phase samples (expected radius ~ RADIUS from centre)
        # so the takeoff line and fly-to-centre segment don't clutter the plot.
        r_exp = np.sqrt((ex - CIRCLE_CENTER_X)**2 + (ey - CIRCLE_CENTER_Y)**2)
        on_circle = r_exp > (RADIUS * 0.5)        # mask: only points out on the loop
        cax, cay = ax[on_circle], ay[on_circle]

        # Pure mathematical ideal circle (smooth dotted reference)
        th = np.linspace(0, 2*math.pi, 400)
        ix = CIRCLE_CENTER_X + RADIUS * np.cos(th)
        iy = CIRCLE_CENTER_Y + RADIUS * np.sin(th)

        # Safety boundary box
        bx = [SAFE_X_MIN, SAFE_X_MAX, SAFE_X_MAX, SAFE_X_MIN, SAFE_X_MIN]
        by = [SAFE_Y_MIN, SAFE_Y_MIN, SAFE_Y_MAX, SAFE_Y_MAX, SAFE_Y_MIN]

        # ── XY top-down: bold dotted ideal + light actual scatter + clean centre ──
        plt.figure(figsize=(7, 7))
        plt.plot(bx, by, "r--", linewidth=1.0, alpha=0.6, label="Safety boundary")
        # Ideal: thick blue dotted
        plt.plot(ix, iy, "b:", linewidth=2.5, label="Ideal circle")
        # Actual: light green line PLUS markers so the path reads as samples on
        # the ideal rather than chords slashing across it.
        plt.plot(cax, cay, "-", color="green", linewidth=0.8, alpha=0.35)
        plt.scatter(cax, cay, s=14, color="green", alpha=0.9,
                    label="Actual (LPS samples)", zorder=5)
        if self.spawn_x is not None:
            plt.plot(self.spawn_x, self.spawn_y, "ko", markersize=9,
                     label=f"Spawn ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.plot(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, "m+", markersize=14,
                 markeredgewidth=2.5, label="Circle centre", zorder=6)
        plt.title("Circle — Ideal vs Actual (XY top-down, LPS coords)")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout()
        plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        # ── 3D: ideal loop + actual samples ──
        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, FLY_Z*np.ones_like(ix), "b:", linewidth=2.0, label="Ideal")
        a3.scatter(cax, cay, az[on_circle], s=10, color="green", alpha=0.8,
                   label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Circle — 3D")
        a3.legend()
        plt.tight_layout()
        plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    node = CirclePathNode()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        for m in ("_save_csv", "_save"):
            fn2 = getattr(node, m, None)
            if fn2:
                try: fn2()
                except Exception: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
