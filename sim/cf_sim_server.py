#!/usr/bin/env python3
"""
cf_sim_server.py — Crazyflie Sim Server for Gazebo
No cffirmware needed. Uses ign service set_pose.
"""
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import SetBool
try:
    from crazyflie_interfaces.srv import Arm   # real-drone arm service
    _HAVE_ARM = True
except Exception:
    _HAVE_ARM = False
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from geometry_msgs.msg import PoseStamped

CF_NAME = "cf231"
WORLD   = "cf_lab"

def set_pose(x, y, z):
    cmd = (
        f'ign service -s /world/{WORLD}/set_pose '
        f'--reqtype ignition.msgs.Pose '
        f'--reptype ignition.msgs.Boolean '
        f'--timeout 2000 '
        f'--req "name: \'{CF_NAME}\', '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}"'
    )
    subprocess.run(cmd, shell=True, capture_output=True)

class CfSimServer(Node):

    def __init__(self):
        super().__init__("cf_sim_server")
        self.x = 1.5
        self.y = 2.0
        self.z = 0.0145    # start resting on the ground plane
        self.armed = False
        self._lock = threading.Lock()
        self._gen = 0          # motion generation: a new command supersedes the old

        self.pose_pub = self.create_publisher(
            PoseStamped, f"/{CF_NAME}/pose", 10)
        self.create_timer(0.05, self._publish_pose)

        # Services
        # Arm: match the REAL crazyswarm2 server (crazyflie_interfaces/srv/Arm)
        # so the same patched trajectory scripts run in sim AND on the real drone.
        # Fall back to SetBool only if crazyflie_interfaces isn't importable.
        if _HAVE_ARM:
            self.create_service(Arm, f"/{CF_NAME}/arm", self._arm_cb)
        else:
            self.create_service(SetBool, f"/{CF_NAME}/arm", self._arm_cb_setbool)
        self.create_service(Takeoff, f"/{CF_NAME}/takeoff", self._takeoff_cb)
        self.create_service(GoTo,    f"/{CF_NAME}/go_to",   self._goto_cb)
        self.create_service(Land,    f"/{CF_NAME}/land",    self._land_cb)

        self.get_logger().info("✅ cf_sim_server ready!")
        self.get_logger().info(
            f"   Services: arm ({'Arm' if _HAVE_ARM else 'SetBool'}) "
            f"| takeoff | go_to | land")

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        with self._lock:
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    # ── Arm service (crazyflie_interfaces/srv/Arm) — field is req.arm ──
    # Arm.Response has no success/message fields, so just return res.
    def _arm_cb(self, req, res):
        self.armed = bool(req.arm)
        self.get_logger().info(
            f"{'🟢 Armed' if self.armed else '🔴 Disarmed'}"
        )
        return res

    # ── Fallback: SetBool arm (field req.data, has success/message) ──
    def _arm_cb_setbool(self, req, res):
        self.armed = req.data
        self.get_logger().info(
            f"{'🟢 Armed' if self.armed else '🔴 Disarmed'}"
        )
        res.success = True
        res.message = "Armed" if self.armed else "Disarmed"
        return res

    # ── Takeoff, Land, GoTo do NOT have success field ──
    def _takeoff_cb(self, req, res):
        if not self.armed:
            self.get_logger().warn("❌ Not armed! Call /cf231/arm first.")
            return res
        target_z = req.height
        duration = max(req.duration.sec + req.duration.nanosec / 1e9, 1.0)
        self.get_logger().info(
            f"🚁 Taking off to {target_z}m in {duration:.1f}s..."
        )
        def _fly():
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sx, sy, sz = self.x, self.y, self.z
            # Wall-clock interpolation: position depends on REAL elapsed time,
            # so slow set_pose calls can't stretch the takeoff out.
            t_start = time.time()
            while True:
                if my_gen != self._gen:        # superseded by a newer command
                    return
                frac = (time.time() - t_start) / duration
                if frac >= 1.0:
                    break
                cz = sz + (target_z - sz) * frac
                with self._lock:
                    self.z = cz
                    cx, cy = self.x, self.y
                set_pose(cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.z = target_z          # snap exactly to target
                cx, cy = self.x, self.y
            set_pose(cx, cy, target_z)
            self.get_logger().info(f"✅ Reached {target_z}m!")
        threading.Thread(target=_fly, daemon=True).start()
        return res

    def _goto_cb(self, req, res):
        duration = max(req.duration.sec + req.duration.nanosec / 1e9, 0.1)
        with self._lock:
            if req.relative:
                tx = self.x + req.goal.x
                ty = self.y + req.goal.y
                tz = self.z + req.goal.z
            else:
                tx = req.goal.x
                ty = req.goal.y
                tz = req.goal.z
        self.get_logger().info(
            f"➡️  GoTo ({tx:.2f}, {ty:.2f}, {tz:.2f}) in {duration:.1f}s"
        )
        def _fly():
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sx, sy, sz = self.x, self.y, self.z
            t_start = time.time()
            while True:
                if my_gen != self._gen:        # superseded by a newer command
                    return
                frac = (time.time() - t_start) / duration
                if frac >= 1.0:
                    break
                with self._lock:
                    self.x = sx + (tx - sx) * frac
                    self.y = sy + (ty - sy) * frac
                    self.z = sz + (tz - sz) * frac
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.x, self.y, self.z = tx, ty, tz
            set_pose(tx, ty, tz)
        threading.Thread(target=_fly, daemon=True).start()
        return res

    def _land_cb(self, req, res):
        duration = max(req.duration.sec + req.duration.nanosec / 1e9, 1.0)
        self.get_logger().info(f"⬇️  Landing in {duration:.1f}s...")
        def _fly():
            target_z = 0.0145   # rest the body on the ground plane (sim visual)
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sx, sy, sz = self.x, self.y, self.z
            t_start = time.time()
            while True:
                if my_gen != self._gen:        # superseded by a newer command
                    return
                frac = (time.time() - t_start) / duration
                if frac >= 1.0:
                    break
                cz = sz + (target_z - sz) * frac
                with self._lock:
                    self.z = cz
                    cx, cy = self.x, self.y
                set_pose(cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.z = target_z
                self.armed = False
                cx, cy = self.x, self.y
            set_pose(cx, cy, target_z)
            self.get_logger().info("✅ Landed!")
        threading.Thread(target=_fly, daemon=True).start()
        return res

def main(args=None):
    rclpy.init(args=args)
    node = CfSimServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
