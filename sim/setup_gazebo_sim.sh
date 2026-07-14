#!/bin/bash
# ============================================================
# setup_gazebo_sim.sh
# Complete Gazebo Simulation Setup for Crazyflie 2.1
# Kaustubh Sharma | IIIT Delhi | Prof. Sanjit Kaul
# Run once: bash setup_gazebo_sim.sh
# ============================================================

echo "======================================"
echo " Crazyflie Gazebo Sim Setup"
echo "======================================"

# Create directories
mkdir -p ~/gazebo_sim/worlds
mkdir -p ~/gazebo_sim/scripts

# ── 1. World SDF ──────────────────────────────────────────────
cat > ~/gazebo_sim/worlds/cf_lab.sdf << 'SDFEOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="cf_lab">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <!-- Use a single directional light with unique name -->
    <light name="directional_light_1" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Floor -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Crazyflie cf231 -->
    <!-- static=false + gravity=false = kinematic body -->
    <!-- set_pose works AND gravity doesn't pull it down -->
    <model name="cf231">
      <pose>1.5 2.0 0.1 0 0 0</pose>
      <static>false</static>
      <link name="base_link">
        <inertial>
          <mass>0.027</mass>
          <inertia>
            <ixx>1.657e-5</ixx><iyy>1.657e-5</iyy><izz>2.9e-5</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>

        <!-- Disable gravity on this link -->
        <gravity>false</gravity>

        <!-- Centre body - dark grey -->
        <visual name="body">
          <pose>0 0 0 0 0 0</pose>
          <geometry><box><size>0.032 0.032 0.016</size></box></geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
          </material>
        </visual>

        <!-- Arm along X axis - blue -->
        <visual name="arm_x">
          <pose>0 0 0 0 0 0</pose>
          <geometry><box><size>0.095 0.007 0.003</size></box></geometry>
          <material>
            <ambient>0.05 0.05 0.7 1</ambient>
            <diffuse>0.1 0.1 0.9 1</diffuse>
          </material>
        </visual>

        <!-- Arm along Y axis - blue -->
        <visual name="arm_y">
          <pose>0 0 0 0 0 0</pose>
          <geometry><box><size>0.007 0.095 0.003</size></box></geometry>
          <material>
            <ambient>0.05 0.05 0.7 1</ambient>
            <diffuse>0.1 0.1 0.9 1</diffuse>
          </material>
        </visual>

        <!-- Motor FL (+x+y) - green = FRONT -->
        <visual name="motor_fl">
          <pose>0.0335 0.0335 0.005 0 0 0</pose>
          <geometry><cylinder><radius>0.010</radius><length>0.007</length></cylinder></geometry>
          <material><ambient>0.0 0.7 0.0 1</ambient><diffuse>0.0 0.9 0.0 1</diffuse></material>
        </visual>

        <!-- Propeller FL -->
        <visual name="prop_fl">
          <pose>0.0335 0.0335 0.010 0 0 0</pose>
          <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 0.5</ambient><diffuse>0.8 0.8 0.8 0.5</diffuse></material>
        </visual>

        <!-- Motor FR (+x-y) - green = FRONT -->
        <visual name="motor_fr">
          <pose>0.0335 -0.0335 0.005 0 0 0</pose>
          <geometry><cylinder><radius>0.010</radius><length>0.007</length></cylinder></geometry>
          <material><ambient>0.0 0.7 0.0 1</ambient><diffuse>0.0 0.9 0.0 1</diffuse></material>
        </visual>

        <!-- Propeller FR -->
        <visual name="prop_fr">
          <pose>0.0335 -0.0335 0.010 0 0 0</pose>
          <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 0.5</ambient><diffuse>0.8 0.8 0.8 0.5</diffuse></material>
        </visual>

        <!-- Motor RL (-x+y) - red = REAR -->
        <visual name="motor_rl">
          <pose>-0.0335 0.0335 0.005 0 0 0</pose>
          <geometry><cylinder><radius>0.010</radius><length>0.007</length></cylinder></geometry>
          <material><ambient>0.8 0.0 0.0 1</ambient><diffuse>0.9 0.0 0.0 1</diffuse></material>
        </visual>

        <!-- Propeller RL -->
        <visual name="prop_rl">
          <pose>-0.0335 0.0335 0.010 0 0 0</pose>
          <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 0.5</ambient><diffuse>0.8 0.8 0.8 0.5</diffuse></material>
        </visual>

        <!-- Motor RR (-x-y) - red = REAR -->
        <visual name="motor_rr">
          <pose>-0.0335 -0.0335 0.005 0 0 0</pose>
          <geometry><cylinder><radius>0.010</radius><length>0.007</length></cylinder></geometry>
          <material><ambient>0.8 0.0 0.0 1</ambient><diffuse>0.9 0.0 0.0 1</diffuse></material>
        </visual>

        <!-- Propeller RR -->
        <visual name="prop_rr">
          <pose>-0.0335 -0.0335 0.010 0 0 0</pose>
          <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 0.5</ambient><diffuse>0.8 0.8 0.8 0.5</diffuse></material>
        </visual>

        <!-- Collision box -->
        <collision name="collision">
          <geometry><box><size>0.092 0.092 0.029</size></box></geometry>
        </collision>

      </link>

      <!-- Pose publisher plugin -->
      <plugin filename="ignition-gazebo-pose-publisher-system"
              name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <update_rate>50</update_rate>
      </plugin>

    </model>

  </world>
</sdf>
SDFEOF
echo "✅ World SDF created"

# ── 2. Sim Server ─────────────────────────────────────────────
cat > ~/gazebo_sim/scripts/cf_sim_server.py << 'PYEOF'
#!/usr/bin/env python3
"""
cf_sim_server.py
Provides ROS2 services for Crazyflie and moves it in Gazebo.
No cffirmware needed.
"""
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
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
        self.z = 0.1
        self.armed = False
        self._lock = threading.Lock()

        self.pose_pub = self.create_publisher(
            PoseStamped, f"/{CF_NAME}/pose", 10)
        self.create_timer(0.05, self._publish_pose)

        self.create_service(SetBool, f"/{CF_NAME}/arm",     self._arm_cb)
        self.create_service(Takeoff, f"/{CF_NAME}/takeoff", self._takeoff_cb)
        self.create_service(GoTo,    f"/{CF_NAME}/go_to",   self._goto_cb)
        self.create_service(Land,    f"/{CF_NAME}/land",    self._land_cb)

        self.get_logger().info("✅ cf_sim_server ready!")
        self.get_logger().info("   Services: arm | takeoff | go_to | land")

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

    def _arm_cb(self, req, res):
        self.armed = req.data
        self.get_logger().info(f"{'🟢 Armed' if self.armed else '🔴 Disarmed'}")
        res.success = True
        res.message = "Armed" if self.armed else "Disarmed"
        return res

    def _takeoff_cb(self, req, res):
        if not self.armed:
            self.get_logger().warn("❌ Not armed! Call /cf231/arm first.")
            res.success = False
            return res
        target_z = req.height
        duration = max(req.duration.sec + req.duration.nanosec / 1e9, 1.0)
        self.get_logger().info(f"🚁 Taking off to {target_z}m in {duration:.1f}s...")

        def _fly():
            steps = 30
            with self._lock:
                start_z = self.z
            dz = (target_z - start_z) / steps
            for i in range(steps):
                with self._lock:
                    self.z += dz
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(cx, cy, cz)
                time.sleep(duration / steps)
            with self._lock:
                self.z = target_z
            set_pose(self.x, self.y, self.z)
            self.get_logger().info(f"✅ Reached {target_z}m!")

        threading.Thread(target=_fly, daemon=True).start()
        res.success = True
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
            steps = max(10, int(duration / 0.05))
            with self._lock:
                sx, sy, sz = self.x, self.y, self.z
            dx = (tx - sx) / steps
            dy = (ty - sy) / steps
            dz = (tz - sz) / steps
            for _ in range(steps):
                with self._lock:
                    self.x += dx
                    self.y += dy
                    self.z += dz
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(cx, cy, cz)
                time.sleep(duration / steps)
            with self._lock:
                self.x, self.y, self.z = tx, ty, tz
            set_pose(tx, ty, tz)

        threading.Thread(target=_fly, daemon=True).start()
        res.success = True
        return res

    def _land_cb(self, req, res):
        duration = max(req.duration.sec + req.duration.nanosec / 1e9, 1.0)
        self.get_logger().info(f"⬇️  Landing in {duration:.1f}s...")

        def _fly():
            steps = 30
            target_z = 0.05
            with self._lock:
                start_z = self.z
            dz = (target_z - start_z) / steps
            for _ in range(steps):
                with self._lock:
                    self.z += dz
                    cx, cy, cz = self.x, self.y, self.z
                set_pose(cx, cy, cz)
                time.sleep(duration / steps)
            with self._lock:
                self.z = target_z
                self.armed = False
            set_pose(self.x, self.y, target_z)
            self.get_logger().info("✅ Landed!")

        threading.Thread(target=_fly, daemon=True).start()
        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = CfSimServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
PYEOF
echo "✅ Sim server created"

# ── 3. Pose Relay ─────────────────────────────────────────────
cat > ~/gazebo_sim/scripts/pose_relay.py << 'PYEOF'
#!/usr/bin/env python3
"""
pose_relay.py
Bridges /world/cf_lab/dynamic_pose/info → /cf231/pose
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped

class PoseRelay(Node):
    def __init__(self):
        super().__init__("pose_relay")
        self.pub = self.create_publisher(PoseStamped, "/cf231/pose", 10)
        self.sub = self.create_subscription(
            PoseArray,
            "/world/cf_lab/dynamic_pose/info",
            self._cb, 10
        )
        self.get_logger().info("✅ Pose relay: Gazebo → /cf231/pose")

    def _cb(self, msg):
        if not msg.poses:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "world"
        ps.pose = msg.poses[0]
        self.pub.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PoseRelay())

if __name__ == "__main__":
    main()
PYEOF
echo "✅ Pose relay created"

# ── 4. Master launch script ───────────────────────────────────
cat > ~/gazebo_sim/launch_sim.sh << 'SHEOF'
#!/bin/bash
# ============================================================
# launch_sim.sh — Launch complete Crazyflie simulation
# Usage: bash ~/gazebo_sim/launch_sim.sh
# Then in another terminal: python3 your_trajectory_script.py
# ============================================================

source /opt/ros/humble/setup.bash
source ~/crazyswarm2_ws/install/setup.bash

echo ""
echo "======================================"
echo " Crazyflie Gazebo Simulation"
echo "======================================"
echo ""
echo "Starting Gazebo..."
export QT_QPA_PLATFORM=xcb

# Clear old cache to prevent sun light duplicate crash
rm -rf ~/.ignition/gazebo/cache 2>/dev/null

# Launch Gazebo in background
ign gazebo ~/gazebo_sim/worlds/cf_lab.sdf -r --force-version 6 &
GAZEBO_PID=$!
echo "✅ Gazebo PID: $GAZEBO_PID"
sleep 5

# Launch ROS-GZ bridge in background
echo "Starting ROS-GZ bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/world/cf_lab/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V" &
BRIDGE_PID=$!
echo "✅ Bridge PID: $BRIDGE_PID"
sleep 2

# Launch sim server in background
echo "Starting Crazyflie sim server..."
python3 ~/gazebo_sim/scripts/cf_sim_server.py &
SERVER_PID=$!
echo "✅ Sim server PID: $SERVER_PID"
sleep 2

echo ""
echo "======================================"
echo " ✅ Simulation ready!"
echo "======================================"
echo ""
echo " Run your trajectory script in a new terminal:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/crazyswarm2_ws/install/setup.bash"
echo "   python3 ~/gazebo_sim/scripts/parabola_trajectory_node.py"
echo ""
echo " Press Ctrl+C to stop everything"
echo "======================================"

# Wait and cleanup on exit
trap "kill $GAZEBO_PID $BRIDGE_PID $SERVER_PID 2>/dev/null; echo 'Simulation stopped.'" EXIT
wait $GAZEBO_PID
SHEOF
chmod +x ~/gazebo_sim/launch_sim.sh
echo "✅ Launch script created"

# ── 5. Copy parabola script ───────────────────────────────────
cp ~/gazebo_sim/parabola_trajectory_node.py \
   ~/gazebo_sim/scripts/parabola_trajectory_node.py 2>/dev/null || true
echo "✅ Setup complete!"
echo ""
echo "======================================"
echo " To start simulation:"
echo "   bash ~/gazebo_sim/launch_sim.sh"
echo ""
echo " Then in a NEW terminal run:"
echo "   python3 ~/gazebo_sim/scripts/parabola_trajectory_node.py"  
echo "======================================"
