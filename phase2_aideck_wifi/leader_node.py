#!/usr/bin/env python3
"""
leader_node.py — Option A leader (pure rclpy + crazyflie_interfaces, no cflib)

AI-deck leader-follower, Option A: the ROS2 graph is the comms link.
The leader flies a pre-planned trajectory and broadcasts every commanded
setpoint on /leader/setpoint (PoseStamped) and its mission phase on
/leader/phase (String). The follower subscribes, trails at a fixed
world-frame offset, and lands itself if the stream goes stale.

Trajectories: circle (rim-entry start), square, triangle (equilateral).

State machine (non-blocking, ticked at `rate` Hz):
    WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> TRAJECTORY -> LAND -> DONE

Carried-over conventions:
  * gentle takeoff ramp (default 5 s) + stabilize hover
  * rim-entry circle start: circle center is placed one radius behind the
    hover point so the trajectory begins exactly where the drone already is
  * no floor-Z arming gates / divergence guards
  * tuned circle values kept as defaults: radius 0.8 m, omega 0.5 rad/s

Run (defaults: cf231, circle, 2 laps):
    python3 leader_node.py
    python3 leader_node.py --ros-args -p trajectory:=square -p side:=1.2
    python3 leader_node.py --ros-args -p trajectory:=triangle -p laps:=3

Start the FOLLOWER FIRST — it takes off, hovers, and waits for this node.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm

GROUND_Z = 0.06        # commanded z at touchdown [m]
DONE_GRACE_S = 1.5     # keep publishing the ground setpoint this long after landing


class LeaderNode(Node):

    def __init__(self):
        super().__init__('leader_node')

        # ---------------- parameters ----------------
        self.declare_parameter('drone_ns', 'cf231')
        self.declare_parameter('trajectory', 'circle')   # circle | square | triangle
        self.declare_parameter('radius', 0.8)            # circle radius [m]
        self.declare_parameter('omega', 0.5)             # circle angular speed [rad/s]
        self.declare_parameter('side', 1.0)              # polygon side length [m]
        self.declare_parameter('speed', 0.25)            # polygon edge speed [m/s]
        self.declare_parameter('height', 1.0)            # cruise height [m]
        self.declare_parameter('laps', 2)
        self.declare_parameter('takeoff_time', 5.0)      # gentle takeoff ramp [s]
        self.declare_parameter('stabilize_time', 3.0)    # hover before trajectory [s]
        self.declare_parameter('land_time', 4.0)         # landing ramp [s]
        self.declare_parameter('rate', 20.0)             # control loop [Hz]
        self.declare_parameter('arm_timeout', 5.0)       # continue w/o arm service (sim)

        g = self.get_parameter
        self.ns = g('drone_ns').value
        self.traj = str(g('trajectory').value).lower()
        self.radius = float(g('radius').value)
        self.omega = float(g('omega').value)
        self.side = float(g('side').value)
        self.speed = float(g('speed').value)
        self.height = float(g('height').value)
        self.laps = int(g('laps').value)
        self.takeoff_time = float(g('takeoff_time').value)
        self.stabilize_time = float(g('stabilize_time').value)
        self.land_time = float(g('land_time').value)
        self.rate = float(g('rate').value)
        self.arm_timeout = float(g('arm_timeout').value)

        if self.traj not in ('circle', 'square', 'triangle'):
            self.get_logger().error(f"unknown trajectory '{self.traj}' — using circle")
            self.traj = 'circle'

        # ---------------- pubs / subs / clients ----------------
        self.cmd_pub = self.create_publisher(Position, f'/{self.ns}/cmd_position', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/leader/setpoint', 10)
        self.phase_pub = self.create_publisher(String, '/leader/phase', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.ns}/pose', self.pose_cb, 10)
        self.arm_client = self.create_client(Arm, f'/{self.ns}/arm')

        # ---------------- state ----------------
        self.state = 'WAIT_POSE'
        self.home = None                 # (x, y, z) captured on first pose
        self.cmd = [0.0, 0.0, 0.0]       # current commanded position
        self.phase_t0 = 0.0              # start time of current state
        self.arm_future = None
        self.arm_wait_t0 = None
        self.center = None               # circle center (rim entry)
        self.segments = None             # polygon segments [((x0,y0),(x1,y1),L), ...]
        self.perimeter = 0.0
        self.total_time = 0.0            # circle duration
        self.total_s = 0.0               # polygon total path length
        self.land_z0 = 0.0
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"leader up: ns={self.ns} traj={self.traj} laps={self.laps} "
            f"height={self.height:.2f} m — start the follower first")

    # ---------------- helpers ----------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_cb(self, msg: PoseStamped):
        if self.home is None:
            p = msg.pose.position
            self.home = (p.x, p.y, p.z)

    def set_state(self, new, now):
        self.get_logger().info(f'{self.state} -> {new}')
        self.state = new
        self.phase_t0 = now

    def publish_phase(self):
        m = String()
        m.data = self.state
        self.phase_pub.publish(m)

    def push(self):
        """Publish cmd_position to the drone + broadcast setpoint/phase to the follower."""
        m = Position()
        m.x, m.y, m.z = float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])
        m.yaw = 0.0
        self.cmd_pub.publish(m)

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'world'
        sp.pose.position.x = float(self.cmd[0])
        sp.pose.position.y = float(self.cmd[1])
        sp.pose.position.z = float(self.cmd[2])
        sp.pose.orientation.w = 1.0
        self.setpoint_pub.publish(sp)

        self.publish_phase()

    def disarm(self):
        if self.arm_client.service_is_ready():
            req = Arm.Request()
            req.arm = False
            self.arm_client.call_async(req)

    # ---------------- trajectory construction ----------------

    def build_trajectory(self):
        """Called once, at the hover point, when entering TRAJECTORY."""
        bx, by = self.cmd[0], self.cmd[1]

        if self.traj == 'circle':
            # Rim entry: center one radius behind the hover point, theta starts at 0,
            # so position(t=0) == hover point. Largest single tracking-error win.
            self.center = (bx - self.radius, by)
            self.total_time = self.laps * 2.0 * math.pi / self.omega
            self.get_logger().info(
                f'circle: center=({self.center[0]:.2f},{self.center[1]:.2f}) '
                f'R={self.radius} omega={self.omega} duration={self.total_time:.1f}s')
            return

        L = self.side
        if self.traj == 'square':
            verts = [(bx, by), (bx + L, by), (bx + L, by + L), (bx, by + L), (bx, by)]
        else:  # equilateral triangle
            verts = [(bx, by), (bx + L, by), (bx + L / 2.0, by + L * math.sqrt(3) / 2.0),
                     (bx, by)]

        self.segments = []
        self.perimeter = 0.0
        for a, b in zip(verts[:-1], verts[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            self.segments.append((a, b, seg_len))
            self.perimeter += seg_len
        self.total_s = self.perimeter * self.laps
        self.get_logger().info(
            f'{self.traj}: side={L} perimeter={self.perimeter:.2f} m '
            f'duration={self.total_s / self.speed:.1f}s')

    def polygon_point(self, s):
        """Point at arc length s along the closed polygon (s already wrapped)."""
        for (x0, y0), (x1, y1), seg_len in self.segments:
            if s <= seg_len:
                f = s / seg_len if seg_len > 0.0 else 0.0
                return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            s -= seg_len
        return self.segments[-1][1]  # numerical edge: end of last segment

    # ---------------- main state machine ----------------

    def tick(self):
        now = self.now_s()

        if self.state == 'WAIT_POSE':
            if self.home is not None:
                self.cmd = [self.home[0], self.home[1], self.home[2]]
                self.get_logger().info(
                    f'home: ({self.home[0]:.2f}, {self.home[1]:.2f}, {self.home[2]:.2f})')
                self.set_state('ARM', now)
            else:
                self.get_logger().info('waiting for pose...', throttle_duration_sec=2.0)
            self.publish_phase()
            return

        if self.state == 'ARM':
            if self.arm_future is not None:
                if self.arm_future.done():
                    self.get_logger().info('armed')
                    self.set_state('TAKEOFF', now)
            elif self.arm_client.service_is_ready():
                req = Arm.Request()
                req.arm = True
                self.arm_future = self.arm_client.call_async(req)
                self.get_logger().info('arming...')
            else:
                if self.arm_wait_t0 is None:
                    self.arm_wait_t0 = now
                if now - self.arm_wait_t0 > self.arm_timeout:
                    self.get_logger().warn('arm service unavailable — continuing (sim?)')
                    self.set_state('TAKEOFF', now)
            self.publish_phase()
            return

        if self.state == 'TAKEOFF':
            f = min(1.0, (now - self.phase_t0) / self.takeoff_time)
            self.cmd[2] = self.home[2] + (self.height - self.home[2]) * f
            self.push()
            if f >= 1.0:
                self.set_state('STABILIZE', now)
            return

        if self.state == 'STABILIZE':
            self.push()
            if now - self.phase_t0 >= self.stabilize_time:
                self.build_trajectory()
                self.set_state('TRAJECTORY', now)
            return

        if self.state == 'TRAJECTORY':
            t = now - self.phase_t0
            finished = False
            if self.traj == 'circle':
                if t >= self.total_time:
                    finished = True
                else:
                    th = self.omega * t
                    self.cmd[0] = self.center[0] + self.radius * math.cos(th)
                    self.cmd[1] = self.center[1] + self.radius * math.sin(th)
            else:
                s = self.speed * t
                if s >= self.total_s:
                    finished = True
                else:
                    x, y = self.polygon_point(s % self.perimeter)
                    self.cmd[0], self.cmd[1] = x, y
            self.cmd[2] = self.height
            self.push()
            if finished:
                self.land_z0 = self.cmd[2]
                self.set_state('LAND', now)
            return

        if self.state == 'LAND':
            f = min(1.0, (now - self.phase_t0) / self.land_time)
            self.cmd[2] = self.land_z0 + (GROUND_Z - self.land_z0) * f
            self.push()
            if now - self.phase_t0 > self.land_time + DONE_GRACE_S:
                self.disarm()
                self.set_state('DONE', now)
            return

        if self.state == 'DONE':
            # keep announcing DONE so the follower reliably sees mission end
            self.publish_phase()
            if not self.done_logged:
                self.get_logger().info('mission complete')
                self.done_logged = True
            return


def main():
    rclpy.init()
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
