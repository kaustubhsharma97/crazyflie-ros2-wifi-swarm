#!/usr/bin/env python3
"""
hover_test.py — TWO-DRONE HOVER TEST (AI-deck era, NO preflight checks)
========================================================================
Crazyflie 2.1+ | Wi-Fi transport via crazyswarm2 | Kaustubh Sharma, IIIT-Delhi

Both drones take off gently to `height` (default 0.8 m), hover for
`hover_time` (default 6 s), land gently. cmd_position streaming at `rate` Hz.
Prints hover-stability stats (XY/Z wobble) at the end.

NO FLIGHT-READINESS CHECKS — arms and flies unconditionally.
Keep the kill switch ready in another terminal:
    ros2 service call /all/emergency std_srvs/srv/Empty

Run (server up first: ros2 launch crazyflie launch.py backend:=cflib mocap:=False):
  python3 hover_test.py                                    # both drones
  python3 hover_test.py --ros-args -p drones:=cf2          # one drone only
  python3 hover_test.py --ros-args -p height:=0.6 -p hover_time:=5.0
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm

GROUND_Z = 0.06        # commanded z at touchdown [m]
DONE_GRACE_S = 1.5     # hold ground setpoint this long after landing


class HoverTest(Node):

    def __init__(self):
        super().__init__('hover_test')

        self.declare_parameter('drones', 'cf231,cf2')
        self.declare_parameter('height', 0.8)          # hover height [m]
        self.declare_parameter('hover_time', 6.0)      # hover duration [s]
        self.declare_parameter('takeoff_time', 5.0)    # gentle ramp up [s]
        self.declare_parameter('land_time', 4.0)       # gentle ramp down [s]
        self.declare_parameter('rate', 20.0)           # control loop [Hz]
        self.declare_parameter('arm_timeout', 5.0)

        g = self.get_parameter
        self.drones = [d.strip() for d in str(g('drones').value).split(',') if d.strip()]
        self.height = float(g('height').value)
        self.hover_time = float(g('hover_time').value)
        self.takeoff_time = float(g('takeoff_time').value)
        self.land_time = float(g('land_time').value)
        self.rate = float(g('rate').value)
        self.arm_timeout = float(g('arm_timeout').value)

        self.pose = {d: None for d in self.drones}
        self.home = {d: None for d in self.drones}
        self.cmd = {d: [0.0, 0.0, 0.0] for d in self.drones}
        self.wobble = {d: [] for d in self.drones}     # (dx,dy,dz) during HOVER

        self.cmd_pub = {}; self.arm_cli = {}
        for d in self.drones:
            self.create_subscription(PoseStamped, f'/{d}/pose',
                                     lambda m, dd=d: self._pcb(dd, m), 10)
            self.cmd_pub[d] = self.create_publisher(Position, f'/{d}/cmd_position', 10)
            self.arm_cli[d] = self.create_client(Arm, f'/{d}/arm')

        self.state = 'WAIT_POSE'
        self.phase_t0 = 0.0
        self.arm_futures = None
        self.arm_wait_t0 = None
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'hover test: drones={self.drones} height={self.height} m '
            f'hover={self.hover_time}s — kill switch: '
            f'ros2 service call /all/emergency std_srvs/srv/Empty')

    # ---------------- helpers ----------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pcb(self, d, msg):
        p = msg.pose.position
        self.pose[d] = (p.x, p.y, p.z)
        if self.home[d] is None:
            self.home[d] = (p.x, p.y, p.z)
        if self.state == 'HOVER':
            self.wobble[d].append((p.x - self.cmd[d][0],
                                   p.y - self.cmd[d][1],
                                   p.z - self.cmd[d][2]))

    def set_state(self, new, now):
        self.get_logger().info(f'{self.state} -> {new}')
        self.state = new
        self.phase_t0 = now

    def push_all(self):
        for d in self.drones:
            m = Position()
            m.x, m.y, m.z = (float(self.cmd[d][0]), float(self.cmd[d][1]),
                             float(self.cmd[d][2]))
            m.yaw = 0.0
            self.cmd_pub[d].publish(m)

    def disarm_all(self):
        for d in self.drones:
            if self.arm_cli[d].service_is_ready():
                req = Arm.Request(); req.arm = False
                self.arm_cli[d].call_async(req)

    # ---------------- state machine ----------------

    def tick(self):
        now = self.now_s()

        if self.state == 'WAIT_POSE':
            if all(self.home[d] is not None for d in self.drones):
                for d in self.drones:
                    h = self.home[d]
                    self.cmd[d] = [h[0], h[1], h[2]]
                    self.get_logger().info(
                        f'{d} home: ({h[0]:.2f}, {h[1]:.2f}, {h[2]:.2f})')
                self.set_state('ARM', now)
            else:
                self.get_logger().info('waiting for pose...',
                                       throttle_duration_sec=2.0)
            return

        if self.state == 'ARM':
            if self.arm_futures is not None:
                if all(f.done() for f in self.arm_futures):
                    self.get_logger().info('all armed')
                    self.set_state('TAKEOFF', now)
            elif all(self.arm_cli[d].service_is_ready() for d in self.drones):
                self.arm_futures = []
                for d in self.drones:
                    req = Arm.Request(); req.arm = True
                    self.arm_futures.append(self.arm_cli[d].call_async(req))
                self.get_logger().info('arming...')
            else:
                if self.arm_wait_t0 is None:
                    self.arm_wait_t0 = now
                if now - self.arm_wait_t0 > self.arm_timeout:
                    self.get_logger().warn('arm service unavailable — continuing (sim?)')
                    self.set_state('TAKEOFF', now)
            return

        if self.state == 'TAKEOFF':
            f = min(1.0, (now - self.phase_t0) / self.takeoff_time)
            for d in self.drones:
                self.cmd[d][2] = self.home[d][2] + (self.height - self.home[d][2]) * f
            self.push_all()
            if f >= 1.0:
                self.set_state('HOVER', now)
            return

        if self.state == 'HOVER':
            self.push_all()
            remaining = self.hover_time - (now - self.phase_t0)
            self.get_logger().info(f'hovering... {max(0.0, remaining):.1f}s left',
                                   throttle_duration_sec=1.9)
            if remaining <= 0.0:
                self.set_state('LAND', now)
            return

        if self.state == 'LAND':
            f = min(1.0, (now - self.phase_t0) / self.land_time)
            for d in self.drones:
                self.cmd[d][2] = self.height + (GROUND_Z - self.height) * f
            self.push_all()
            if now - self.phase_t0 > self.land_time + DONE_GRACE_S:
                self.disarm_all()
                self.report()
                self.set_state('DONE', now)
            return

        if self.state == 'DONE':
            if not self.done_logged:
                self.get_logger().info('hover test complete')
                self.done_logged = True
            return

    def report(self):
        """Stability stats from the HOVER phase."""
        for d in self.drones:
            w = self.wobble[d]
            if not w:
                self.get_logger().warn(f'{d}: no hover samples recorded')
                continue
            xy = [math.hypot(dx, dy) for dx, dy, _ in w]
            dz = [abs(z) for _, _, z in w]
            self.get_logger().info(
                f'{d} hover stability ({len(w)} samples): '
                f'XY wobble mean={sum(xy)/len(xy):.3f} m max={max(xy):.3f} m | '
                f'Z wobble mean={sum(dz)/len(dz):.3f} m max={max(dz):.3f} m')


def main():
    rclpy.init()
    node = HoverTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
