#!/usr/bin/env python3
"""
cf_sim_server_swarm.py — TWO-DRONE Crazyflie Sim Server for Gazebo
==================================================================
Kaustubh Sharma | IIIT-Delhi | Crazyflie 2.1+ swarm sim

Extends the single-drone cf_sim_server.py to serve BOTH cf231 and cf2 so the
swarm scripts (formation_hold, leader_follower, synchronized, formation_flight)
and follow_me can be tested in Gazebo before/alongside real flights.

Each drone is an independent CfSimDrone with its own:
  - state (x, y, z, armed)
  - /<name>/pose publisher
  - /<name>/arm, /takeoff, /go_to, /land services
All driven by wall-clock interpolation (same proven method as the single-drone
server). No cffirmware needed; uses ign service set_pose per drone.

Run (instead of the single-drone server):
  python3 cf_sim_server_swarm.py
"""
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import SetBool
try:
    from crazyflie_interfaces.srv import Arm
    _HAVE_ARM = True
except Exception:
    _HAVE_ARM = False
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from geometry_msgs.msg import PoseStamped

WORLD = "cf_lab"

# Two drones, each with a distinct Gazebo model name + spawn position.
# Spawn them apart so formations start safely separated.
DRONES = {
    "cf231": {"spawn": (1.0, 2.5, 0.0145), "model": "cf231"},
    "cf2":   {"spawn": (2.5, 2.5, 0.0145), "model": "cf2"},
}


def set_pose(model, x, y, z):
    cmd = (
        f'ign service -s /world/{WORLD}/set_pose '
        f'--reqtype ignition.msgs.Pose '
        f'--reptype ignition.msgs.Boolean '
        f'--timeout 2000 '
        f'--req "name: \'{model}\', '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}"'
    )
    subprocess.run(cmd, shell=True, capture_output=True)


class CfSimDrone:
    """One simulated Crazyflie: own state, pose pub, and 4 services."""
    def __init__(self, node: Node, name: str, spawn, model: str):
        self.node = node
        self.name = name
        self.model = model
        self.x, self.y, self.z = spawn
        self.armed = False
        self._lock = threading.Lock()
        self._gen = 0          # motion generation: a new command supersedes the old

        self.pose_pub = node.create_publisher(PoseStamped, f"/{name}/pose", 10)

        if _HAVE_ARM:
            node.create_service(Arm, f"/{name}/arm", self._arm_cb)
        else:
            node.create_service(SetBool, f"/{name}/arm", self._arm_cb_setbool)
        node.create_service(Takeoff, f"/{name}/takeoff", self._takeoff_cb)
        node.create_service(GoTo,    f"/{name}/go_to",   self._goto_cb)
        node.create_service(Land,    f"/{name}/land",    self._land_cb)

        # place the model at its spawn at startup
        set_pose(self.model, self.x, self.y, self.z)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        with self._lock:
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def _log(self, m): self.node.get_logger().info(f"[{self.name}] {m}")
    def _warn(self, m): self.node.get_logger().warn(f"[{self.name}] {m}")

    def _arm_cb(self, req, res):
        self.armed = bool(req.arm)
        self._log("🟢 Armed" if self.armed else "🔴 Disarmed")
        return res

    def _arm_cb_setbool(self, req, res):
        self.armed = req.data
        self._log("🟢 Armed" if self.armed else "🔴 Disarmed")
        res.success = True; res.message = "ok"
        return res

    def _takeoff_cb(self, req, res):
        if not self.armed:
            self._warn(f"❌ Not armed! Call /{self.name}/arm first.")
            return res
        target_z = req.height
        duration = max(req.duration.sec + req.duration.nanosec/1e9, 1.0)
        self._log(f"🚁 Takeoff to {target_z}m in {duration:.1f}s")
        def _fly():
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sz = self.z
            t0 = time.time()
            while True:
                if my_gen != self._gen:
                    return
                frac = (time.time()-t0)/duration
                if frac >= 1.0: break
                with self._lock:
                    self.z = sz + (target_z - sz)*frac
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(self.model, cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.z = target_z; cx, cy = self.x, self.y
            set_pose(self.model, cx, cy, target_z)
            self._log(f"✅ Reached {target_z}m")
        threading.Thread(target=_fly, daemon=True).start()
        return res

    def _goto_cb(self, req, res):
        duration = max(req.duration.sec + req.duration.nanosec/1e9, 0.1)
        with self._lock:
            if req.relative:
                tx, ty, tz = self.x+req.goal.x, self.y+req.goal.y, self.z+req.goal.z
            else:
                tx, ty, tz = req.goal.x, req.goal.y, req.goal.z
        def _fly():
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sx, sy, sz = self.x, self.y, self.z
            t0 = time.time()
            while True:
                if my_gen != self._gen:
                    return
                frac = (time.time()-t0)/duration
                if frac >= 1.0: break
                with self._lock:
                    self.x = sx + (tx-sx)*frac
                    self.y = sy + (ty-sy)*frac
                    self.z = sz + (tz-sz)*frac
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(self.model, cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.x, self.y, self.z = tx, ty, tz
            set_pose(self.model, tx, ty, tz)
        threading.Thread(target=_fly, daemon=True).start()
        return res

    def _land_cb(self, req, res):
        duration = max(req.duration.sec + req.duration.nanosec/1e9, 1.0)
        self._log(f"⬇️  Landing in {duration:.1f}s")
        def _fly():
            target_z = 0.0145
            with self._lock:
                self._gen += 1
                my_gen = self._gen
                sz = self.z
            t0 = time.time()
            while True:
                if my_gen != self._gen:
                    return
                frac = (time.time()-t0)/duration
                if frac >= 1.0: break
                with self._lock:
                    self.z = sz + (target_z - sz)*frac
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(self.model, cx, cy, cz)
            if my_gen != self._gen:
                return
            with self._lock:
                self.z = target_z; self.armed = False; cx, cy = self.x, self.y
            set_pose(self.model, cx, cy, target_z)
            self._log("✅ Landed")
        threading.Thread(target=_fly, daemon=True).start()
        return res


class CfSimServerSwarm(Node):
    def __init__(self):
        super().__init__("cf_sim_server_swarm")
        self.drones = []
        for name, cfg in DRONES.items():
            self.drones.append(CfSimDrone(self, name, cfg["spawn"], cfg["model"]))
        # one timer publishes all drones' poses
        self.create_timer(0.05, self._publish_all)
        self.get_logger().info(
            f"✅ cf_sim_server_swarm ready! Drones: {list(DRONES.keys())} "
            f"| arm ({'Arm' if _HAVE_ARM else 'SetBool'}) | takeoff | go_to | land")

    def _publish_all(self):
        for d in self.drones:
            d.publish_pose()


def main(args=None):
    rclpy.init(args=args)
    node = CfSimServerSwarm()
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
