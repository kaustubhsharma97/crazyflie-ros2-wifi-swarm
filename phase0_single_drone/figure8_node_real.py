#!/usr/bin/env python3
"""
figure8_node_real.py — ROS2 Humble / rclpy  (OPTION B: real-drone onboard polynomial)
=======================================================================================
Crazyflie 2.1 + Loco Positioning System (TDoA2, 8 anchors)
Kaustubh Sharma | Summer Intern, IIIT Delhi (Prof. Sanjit Kaul)
Lab: B-419, IRAS Hub (Robotics Lab), IIIT-Delhi

This is the REAL-DRONE figure-8: instead of streaming go_to waypoints
(Option A / sim), it uploads a piecewise-polynomial trajectory to the
Crazyflie's ONBOARD memory and runs it natively with start_trajectory.
The firmware then interpolates the polynomial at high rate, giving a
smooth continuous figure-8 — the same mechanism Raghav's figure-8.py uses.

  *** This will NOT work in the custom Gazebo cf_sim_server (it has no
      upload_trajectory / start_trajectory). Run it on the REAL drone:
        ros2 launch crazyflie launch.py backend:=cflib mocap:=False
      For sim, use figure8_node_sim.py (Option A). ***

Trajectory: lemniscate of Gerono, A=1.0 B=1.0, centre (1.5, 2.5), Z=0.6,
fit to 8 degree-7 polynomial pieces -> figure8_poly.csv (must sit beside
this script, or pass its path as argv[1]).

crazyswarm2 polynomial CSV format, one row per piece:
  duration, x[0..7], y[0..7], z[0..7], yaw[0..7]   (33 columns)

Outputs:
  ~/figure8_real_trajectory_log.csv
  ~/figure8_real_xy_topdown.png
  ~/figure8_real_3d.png

Run (real drone, after crazyswarm2 server is up):
  python3 figure8_node_real.py [path/to/figure8_poly.csv]
"""

import os
import csv
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, UploadTrajectory, StartTrajectory, Arm
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from std_srvs.srv import SetBool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

CF_NAME = "cf231"

# Must match the curve the polynomial CSV was generated from
CENTER_X, CENTER_Y = 1.5, 2.5
A, B = 1.0, 1.0
FLY_Z = 0.6
TAKEOFF_TIME    = 2.5
LPS_SETTLE_TIME = 3.0
GOTO_SPEED      = 0.5
TRAJECTORY_ID   = 1          # onboard slot to store the trajectory in

SAFE_X_MIN, SAFE_X_MAX = 0.0, 3.0
SAFE_Y_MIN, SAFE_Y_MAX = 1.0, 4.0

# Default polynomial CSV path: beside this script, else ~/figure8_poly.csv
_DEFAULT_POLY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "figure8_poly.csv")

CSV_PATH = os.path.expanduser("~/figure8_real_trajectory_log.csv")
PNG_XY   = os.path.expanduser("~/figure8_real_xy_topdown.png")
PNG_3D   = os.path.expanduser("~/figure8_real_3d.png")


def fig8_point(t):
    x = CENTER_X + A * math.sin(t)
    y = CENTER_Y + B * math.sin(t) * math.cos(t)
    return x, y


def load_poly_pieces(path):
    """Load the polynomial CSV into a list of TrajectoryPolynomialPiece msgs."""
    pieces = []
    total_duration = 0.0
    with open(path, newline="") as f:
        for row in csv.reader(f):
            vals = [float(v) for v in row if v.strip() != ""]
            if len(vals) != 33:
                raise ValueError(
                    f"Expected 33 columns per piece, got {len(vals)}")
            piece = TrajectoryPolynomialPiece()
            dur = vals[0]
            piece.poly_x   = vals[1:9]
            piece.poly_y   = vals[9:17]
            piece.poly_z   = vals[17:25]
            piece.poly_yaw = vals[25:33]
            # duration as builtin_interfaces/Duration
            piece.duration.sec = int(dur)
            piece.duration.nanosec = int((dur % 1) * 1e9)
            pieces.append(piece)
            total_duration += dur
    return pieces, total_duration


class Figure8RealNode(Node):

    def __init__(self, poly_path):
        super().__init__("figure8_real_node")

        self.poly_path = poly_path
        self.current_pose = None
        self.setpoint     = (0.0, 0.0, FLY_Z)
        self.log_data     = []
        self.t0           = None
        self.spawn_x = self.spawn_y = self.spawn_z = None

        self.state    = "WAIT_POSE"
        self._pending = None
        self._wait_until = 0

        # Load polynomial pieces up front so we fail early if the CSV is bad
        try:
            self.pieces, self.traj_duration = load_poly_pieces(poly_path)
            self.get_logger().info(
                f"Loaded {len(self.pieces)} polynomial pieces "
                f"({self.traj_duration:.1f}s) from {poly_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load polynomial CSV: {e}")
            raise

        self.create_subscription(
            PoseStamped, f"/{CF_NAME}/pose", self._pose_cb, 10)

        self.arm_cli      = self.create_client(Arm, f"/{CF_NAME}/arm")
        self.takeoff_cli  = self.create_client(Takeoff, f"/{CF_NAME}/takeoff")
        self.goto_cli     = self.create_client(GoTo,    f"/{CF_NAME}/go_to")
        self.land_cli     = self.create_client(Land,    f"/{CF_NAME}/land")
        self.upload_cli   = self.create_client(UploadTrajectory, f"/{CF_NAME}/upload_trajectory")
        self.start_cli    = self.create_client(StartTrajectory,  f"/{CF_NAME}/start_trajectory")

        self.create_timer(0.1, self._tick)
        self.get_logger().info("Figure8RealNode (Option B, onboard polynomial) started.")

    # ── CALLBACKS ─────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x; y = msg.pose.position.y; z = msg.pose.position.z
        self.current_pose = (x, y, z)
        if self.t0 is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            self.log_data.append({
                "time": round(t, 3),
                "actual_x": x, "actual_y": y, "actual_z": z,
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

    # ── STATE MACHINE ─────────────────────────────────────────────────────────
    def _tick(self):

        if self.state == "WAIT_POSE":
            if self.current_pose is None:
                return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(LPS_SETTLE_TIME * 1e9)
                self.get_logger().info(f"Waiting {LPS_SETTLE_TIME}s for LPS lock ...")
                return
            if not self._past():
                return
            self.spawn_x, self.spawn_y, self.spawn_z = self.current_pose
            self.t0 = self._now() / 1e9
            self.get_logger().info(f"Spawn: x={self.spawn_x:.2f} y={self.spawn_y:.2f}")
            self.state = "WAIT_SERVICES"

        elif self.state == "WAIT_SERVICES":
            # Make sure the onboard-trajectory services exist (real drone only)
            if not self.upload_cli.service_is_ready():
                self.get_logger().warn(
                    "upload_trajectory service not available — are you on the "
                    "REAL drone (backend:=cflib)? This node will not work in the "
                    "custom Gazebo sim. Waiting ...", throttle_duration_sec=5.0)
                return
            if not self.start_cli.service_is_ready():
                return
            self.state = "ARM"

        elif self.state == "ARM":
            if self._pending is None:
                req = Arm.Request(); req.arm = True
                self._pending = self.arm_cli.call_async(req)
            elif self._pending.done():
                self.get_logger().info("Armed.")
                self._pending = None
                self.state = "UPLOAD"

        # ── Upload the polynomial trajectory to onboard memory ──
        elif self.state == "UPLOAD":
            if self._pending is None:
                req = UploadTrajectory.Request()
                req.trajectory_id = TRAJECTORY_ID
                req.piece_offset = 0
                req.pieces = self.pieces
                self._pending = self.upload_cli.call_async(req)
                self.get_logger().info(
                    f"Uploading {len(self.pieces)} pieces to slot {TRAJECTORY_ID} ...")
            elif self._pending.done():
                self._pending = None
                self.get_logger().info("Trajectory uploaded.")
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

        # ── Fly to the figure-8 entry point (t=0 -> the centre) ──
        elif self.state == "APPROACH":
            if self._pending is None:
                ex, ey = fig8_point(0.0)
                dist = math.sqrt((ex - self.spawn_x)**2 + (ey - self.spawn_y)**2)
                dur = max(2.0, dist / GOTO_SPEED)
                self._pending = self._send_goto(ex, ey, FLY_Z, dur)
                self._wait_until = self._now() + int((dur + 1.0) * 1e9)
                self.get_logger().info("Approaching figure-8 entry point ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "START_TRAJ"

        # ── Run the onboard trajectory natively (the smooth part) ──
        elif self.state == "START_TRAJ":
            if self._pending is None:
                req = StartTrajectory.Request()
                req.trajectory_id = TRAJECTORY_ID
                req.timescale = 1.0
                req.relative = False
                req.reversed = False
                self._pending = self.start_cli.call_async(req)
                # wait the trajectory's full duration (+margin) while it flies
                self._wait_until = self._now() + int((self.traj_duration + 1.5) * 1e9)
                self.get_logger().info(
                    f"Running onboard figure-8 ({self.traj_duration:.1f}s) ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "RETURN_CENTER"

        # ── Settle at centre before landing ──
        elif self.state == "RETURN_CENTER":
            if self._pending is None:
                self._pending = self._send_goto(CENTER_X, CENTER_Y, FLY_Z, 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)
                self.get_logger().info("Returning to centre to settle ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.state = "LAND"

        elif self.state == "LAND":
            if self._pending is None:
                req = Land.Request()
                req.height = 0.0   # B-419 LPS frame: floor = Z=0 (anchors at 0.3m), land on floor
                req.duration.sec = 3; req.duration.nanosec = 0
                self._pending = self.land_cli.call_async(req)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Landing ...")
            elif self._pending.done() and self._past():
                self._pending = None
                self.get_logger().info("Landed.")
                self.state = "SAVE"

        elif self.state == "SAVE":
            self._save_csv()
            self._save_plots()
            self.get_logger().info("Sequence complete.")
            self.state = "DONE"

        elif self.state == "DONE":
            rclpy.shutdown()

    # ── OUTPUT ────────────────────────────────────────────────────────────────
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

        # Ideal figure-8 (smooth dotted reference)
        tt = np.linspace(0, 2*math.pi, 600)
        ix = CENTER_X + A * np.sin(tt)
        iy = CENTER_Y + B * np.sin(tt) * np.cos(tt)

        # Keep samples while the drone is actually near the 8 (drop takeoff/approach)
        ideal_pts = np.column_stack([ix, iy])
        keep = []
        for x, y in zip(ax, ay):
            d = np.min(np.hypot(ideal_pts[:,0]-x, ideal_pts[:,1]-y))
            keep.append(d < 0.25)
        keep = np.array(keep)
        fax, fay, faz = ax[keep], ay[keep], az[keep]

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
                     label=f"Spawn ({self.spawn_x:.2f},{self.spawn_y:.2f})", zorder=6)
        plt.plot(CENTER_X, CENTER_Y, "m+", markersize=14, markeredgewidth=2.5,
                 label="Centre", zorder=6)
        plt.title("Figure-8 (onboard polynomial) — Ideal vs Actual")
        plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right", fontsize=8)
        plt.grid(True, alpha=0.3); plt.axis("equal")
        plt.tight_layout()
        plt.savefig(PNG_XY, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_XY}")

        fig = plt.figure(figsize=(7, 6))
        a3 = fig.add_subplot(111, projection="3d")
        a3.plot(ix, iy, FLY_Z*np.ones_like(ix), "b:", linewidth=2.0, label="Ideal")
        a3.scatter(fax, fay, faz, s=10, color="green", alpha=0.8, label="Actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Figure-8 (onboard polynomial) — 3D")
        a3.legend()
        plt.tight_layout()
        plt.savefig(PNG_3D, dpi=120); plt.close()
        self.get_logger().info(f"Plot -> {PNG_3D}")


def main(args=None):
    rclpy.init(args=args)
    poly_path = sys.argv[1] if len(sys.argv) > 1 else _DEFAULT_POLY
    if not os.path.exists(poly_path):
        print(f"ERROR: polynomial CSV not found: {poly_path}")
        print("Place figure8_poly.csv beside this script or pass its path.")
        return
    node = Figure8RealNode(poly_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._save_csv()
        node._save_plots()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
