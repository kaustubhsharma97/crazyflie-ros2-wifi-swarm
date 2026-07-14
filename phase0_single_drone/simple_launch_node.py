#!/usr/bin/env python3
"""
simple_launch_node.py — ROS2 Humble / rclpy
=============================================
Crazyflie 2.1 | Takeoff → Hover → Land
Kaustubh Sharma | Summer Intern, IIIT Delhi (Prof. Sanjit Kaul)

Pure rclpy + crazyflie_interfaces. No crazyflie_py / cflib in this script.
Non-blocking state machine (no nested spin deadlock).

Works UNCHANGED in:
  - Gazebo sim  (launch_sim.sh + cf_sim_server.py)
  - Real drone  (ros2 launch crazyflie launch.py backend:=cflib mocap:=False)

Outputs on completion:
  ~/simple_launch_log.csv
  ~/simple_launch_xy_topdown.png
  ~/simple_launch_z_time.png

Run:
  python3 simple_launch_node.py
"""

import os
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, Arm
from std_srvs.srv import SetBool

import matplotlib
matplotlib.use("Agg")            # headless — no display needed
import matplotlib.pyplot as plt

# ─── CONFIG ─────────────────────────────────────────────────────────────────────
CF_NAME        = "cf231"
HOVER_HEIGHT   = 0.5             # metres
TAKEOFF_TIME   = 2.5            # seconds to rise
HOVER_TIME     = 5.0            # seconds to hold hover
LAND_TIME      = 3.0           # seconds to descend

CSV_PATH = os.path.expanduser("~/simple_launch_log.csv")
PNG_XY   = os.path.expanduser("~/simple_launch_xy_topdown.png")
PNG_Z    = os.path.expanduser("~/simple_launch_z_time.png")


class SimpleLaunchNode(Node):

    def __init__(self):
        super().__init__("simple_launch_node")

        self.current_pose = None
        self.setpoint     = (0.0, 0.0, HOVER_HEIGHT)
        self.log_data     = []          # collected in memory → CSV + PNG
        self.t0           = None        # mission start time (for elapsed column)

        self.state    = "WAIT_POSE"
        self.start_x  = self.start_y = self.start_z = 0.0
        self._pending = None
        self._wait_until = 0

        # Pose subscriber (same topic in sim and real)
        self.create_subscription(
            PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)

        # Service clients
        self.arm_cli     = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.land_cli    = self.create_client(Land,    f"/{CF_NAME}/land")

        # Single timer drives the state machine
        self.create_timer(0.1, self._tick)
        self.get_logger().info("SimpleLaunchNode started (non-blocking).")

    # ── CALLBACKS ───────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.current_pose = (x, y, z)

        # Log every pose once the mission clock has started
        if self.t0 is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            self.log_data.append({
                "time":       round(t, 3),
                "actual_x":   x,
                "actual_y":   y,
                "actual_z":   z,
                "expected_x": self.setpoint[0],
                "expected_y": self.setpoint[1],
                "expected_z": self.setpoint[2],
            })

    # ── HELPERS ─────────────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds

    def _elapsed_past(self):
        return self._now() > self._wait_until

    # ── STATE MACHINE ─────────────────────────────────────────────────────────────
    def _tick(self):

        if self.state == "WAIT_POSE":
            if self.current_pose is None:
                return
            self.start_x, self.start_y, self.start_z = self.current_pose
            self.t0 = self._now() / 1e9         # start logging clock
            self.setpoint = (self.start_x, self.start_y, HOVER_HEIGHT)
            self.get_logger().info(
                f"Start pose  x={self.start_x:.2f} y={self.start_y:.2f} z={self.start_z:.2f}"
            )
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
                req = Takeoff.Request()
                req.height = HOVER_HEIGHT
                req.duration.sec = int(TAKEOFF_TIME)
                req.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                self._pending = self.takeoff_cli.call_async(req)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.0) * 1e9)
                self.get_logger().info(f"Taking off to {HOVER_HEIGHT} m ...")
            elif self._pending.done() and self._elapsed_past():
                self._pending = None
                self.get_logger().info("Reached hover height.")
                self._wait_until = self._now() + int(HOVER_TIME * 1e9)
                self.state = "HOVER"

        elif self.state == "HOVER":
            # Just hold — pose logger keeps recording. Wait out the hover time.
            if self._elapsed_past():
                self.get_logger().info("Hover complete.")
                self.state = "LAND"

        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request()
                req.height = 0.0   # B-419 LPS frame: floor = Z=0 (anchors at 0.3m)
                req.duration.sec = int(LAND_TIME)
                req.duration.nanosec = int((LAND_TIME % 1) * 1e9)
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int((LAND_TIME + 1.0) * 1e9)
                self.get_logger().info("Landing ...")
            elif self._pending.done() and self._elapsed_past():
                self._pending = None
                self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._save_csv()
            self._save_plots()
            self.get_logger().info("Mission complete.")
            self.state = "DONE"

        elif self.state == "DONE":
            rclpy.shutdown()

    # ── OUTPUT ──────────────────────────────────────────────────────────────────
    def _save_csv(self):
        if not self.log_data:
            self.get_logger().warn("No pose data logged — skipping CSV.")
            return
        keys = ["time", "actual_x", "actual_y", "actual_z",
                "expected_x", "expected_y", "expected_z"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.log_data)
        self.get_logger().info(f"CSV  -> {CSV_PATH}  ({len(self.log_data)} rows)")

    def _save_plots(self):
        if not self.log_data:
            return
        t  = [d["time"]       for d in self.log_data]
        ax = [d["actual_x"]   for d in self.log_data]
        ay = [d["actual_y"]   for d in self.log_data]
        az = [d["actual_z"]   for d in self.log_data]
        ez = [d["expected_z"] for d in self.log_data]

        # XY top-down
        plt.figure()
        plt.plot(ax, ay, label="Actual path")
        plt.plot(self.start_x, self.start_y, "ko", label="Start")
        plt.title("Simple Launch — XY top-down")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.axis("equal"); plt.grid(True); plt.legend()
        plt.savefig(PNG_XY); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        # Z vs time
        plt.figure()
        plt.plot(t, az, label="Actual Z")
        plt.plot(t, ez, "--", label="Expected Z")
        plt.title("Simple Launch — Height vs Time")
        plt.xlabel("Time (s)"); plt.ylabel("Z (m)")
        plt.grid(True); plt.legend()
        plt.savefig(PNG_Z); plt.close()
        self.get_logger().info(f"Plot -> {PNG_Z}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaunchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._save_csv()
        node._save_plots()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
