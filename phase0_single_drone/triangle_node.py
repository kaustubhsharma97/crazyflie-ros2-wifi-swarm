#!/usr/bin/env python3
"""
triangle_node.py — SINGLE-DRONE EQUILATERAL TRIANGLE (robust build)
====================================================================
Crazyflie 2.1+ | pure rclpy + crazyflie_interfaces (no cflib)
Kaustubh Sharma | IRAS Hub, IIIT-Delhi (Prof. Sanjit Kaul)

Flies an equilateral triangle of side `side` at constant edge speed,
starting at the drone's own takeoff point (first vertex), extending into
+X and +Y. Link-agnostic: works over radio:// or tcp:// — whatever the
crazyswarm2 server's yaml says (drone 04 = cf231).

Robustness kit:
  * Room safe-zone clamp from the 8 LPS anchor positions (commands are
    clamped, never refused).
  * Gentle 5 s takeoff / 4 s landing; lands to home_z + 0.03 (immune to
    the LPS negative-floor artifact).
  * CSV log of setpoint vs measured pose + tracking-error stats on exit
    (also saved on Ctrl-C).

State machine: WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> TRIANGLE
               -> LAND -> DONE   (no arming gates — executes unconditionally)

Run (server up first):
  python3 triangle_node.py
  python3 triangle_node.py --ros-args -p side:=1.2 -p laps:=2
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm

DONE_GRACE_S = 1.5
CSV_PATH = os.path.expanduser('~/triangle_trajectory_log.csv')

# Room safe-zone from the 8 LPS anchors (X 0..5.04, Y -0.8..7.4), with margins
SAFE_X = (0.30, 4.74)
SAFE_Y = (-0.30, 6.90)
SAFE_Z = (-0.50, 2.20)


class TriangleNode(Node):

    def __init__(self):
        super().__init__('triangle_node')

        self.declare_parameter('drone_ns', 'cf231')     # drone 04
        self.declare_parameter('side', 1.0)             # triangle side [m]
        self.declare_parameter('speed', 0.25)           # edge speed [m/s]
        self.declare_parameter('height', 0.8)           # cruise height [m]
        self.declare_parameter('laps', 1)
        self.declare_parameter('takeoff_time', 5.0)
        self.declare_parameter('stabilize_time', 3.0)
        self.declare_parameter('land_time', 4.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('arm_timeout', 5.0)

        g = self.get_parameter
        self.ns = g('drone_ns').value
        self.side = float(g('side').value)
        self.speed = float(g('speed').value)
        self.height = float(g('height').value)
        self.laps = int(g('laps').value)
        self.takeoff_time = float(g('takeoff_time').value)
        self.stabilize_time = float(g('stabilize_time').value)
        self.land_time = float(g('land_time').value)
        self.rate = float(g('rate').value)
        self.arm_timeout = float(g('arm_timeout').value)

        self.cmd_pub = self.create_publisher(Position, f'/{self.ns}/cmd_position', 10)
        self.create_subscription(PoseStamped, f'/{self.ns}/pose', self.pose_cb, 10)
        self.arm_client = self.create_client(Arm, f'/{self.ns}/arm')

        self.state = 'WAIT_POSE'
        self.phase_t0 = 0.0
        self.pose = None
        self.home = None
        self.cmd = [0.0, 0.0, 0.0]
        self.arm_future = None
        self.arm_wait_t0 = None
        self.segments = None
        self.perimeter = 0.0
        self.total_s = 0.0
        self.land_z0 = 0.0
        self.log = []                   # csv rows during flight
        self.t0 = None
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'triangle: ns={self.ns} side={self.side} speed={self.speed} '
            f'laps={self.laps} height={self.height} — extends +X/+Y from '
            f'takeoff point; keep ~{self.side + 0.5:.1f} m clear that side')

    # ---------------- helpers ----------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_cb(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)
        if self.home is None:
            self.home = (p.x, p.y, p.z)
        if self.t0 is not None and self.state in ('TAKEOFF', 'STABILIZE',
                                                  'TRIANGLE', 'LAND'):
            self.log.append({
                'time': round(self.now_s() - self.t0, 3),
                'x': p.x, 'y': p.y, 'z': p.z,
                'ex': self.cmd[0], 'ey': self.cmd[1], 'ez': self.cmd[2],
                'state': self.state})

    def set_state(self, new, now):
        self.get_logger().info(f'{self.state} -> {new}')
        self.state = new
        self.phase_t0 = now

    def clamp(self):
        x, y, z = self.cmd
        cx = min(max(x, SAFE_X[0]), SAFE_X[1])
        cy = min(max(y, SAFE_Y[0]), SAFE_Y[1])
        cz = min(max(z, SAFE_Z[0]), SAFE_Z[1])
        if (cx, cy, cz) != (x, y, z):
            self.get_logger().warn(
                f'command clamped to room safe-zone: ({x:.2f},{y:.2f},{z:.2f})'
                f' -> ({cx:.2f},{cy:.2f},{cz:.2f})', throttle_duration_sec=1.0)
        self.cmd = [cx, cy, cz]

    def push(self):
        self.clamp()
        m = Position()
        m.x, m.y, m.z = float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])
        m.yaw = 0.0
        self.cmd_pub.publish(m)

    def disarm(self):
        if self.arm_client.service_is_ready():
            req = Arm.Request(); req.arm = False
            self.arm_client.call_async(req)

    def build_triangle(self):
        bx, by = self.cmd[0], self.cmd[1]
        L = self.side
        verts = [(bx, by), (bx + L, by),
                 (bx + L / 2.0, by + L * math.sqrt(3) / 2.0), (bx, by)]
        self.segments = []
        self.perimeter = 0.0
        for a, b in zip(verts[:-1], verts[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            self.segments.append((a, b, seg))
            self.perimeter += seg
        self.total_s = self.perimeter * self.laps
        self.get_logger().info(
            f'triangle built: perimeter={self.perimeter:.2f} m, '
            f'duration={self.total_s / self.speed:.1f} s')

    def tri_point(self, s):
        for (x0, y0), (x1, y1), seg in self.segments:
            if s <= seg:
                f = s / seg if seg > 0 else 0.0
                return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            s -= seg
        return self.segments[-1][1]

    # ---------------- state machine ----------------

    def tick(self):
        now = self.now_s()

        if self.state == 'WAIT_POSE':
            if self.home is not None:
                self.cmd = list(self.home)
                self.get_logger().info(
                    f'home: ({self.home[0]:.2f}, {self.home[1]:.2f}, '
                    f'{self.home[2]:.2f})')
                self.t0 = now
                self.set_state('ARM', now)
            else:
                self.get_logger().info('waiting for pose...',
                                       throttle_duration_sec=2.0)
            return

        if self.state == 'ARM':
            if self.arm_future is not None:
                if self.arm_future.done():
                    self.get_logger().info('armed')
                    self.set_state('TAKEOFF', now)
            elif self.arm_client.service_is_ready():
                req = Arm.Request(); req.arm = True
                self.arm_future = self.arm_client.call_async(req)
                self.get_logger().info('arming...')
            else:
                if self.arm_wait_t0 is None:
                    self.arm_wait_t0 = now
                if now - self.arm_wait_t0 > self.arm_timeout:
                    self.get_logger().warn('arm service unavailable — '
                                           'continuing (sim?)')
                    self.set_state('TAKEOFF', now)
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
                self.build_triangle()
                self.set_state('TRIANGLE', now)
            return

        if self.state == 'TRIANGLE':
            s = self.speed * (now - self.phase_t0)
            if s >= self.total_s:
                self.cmd[2] = self.height
                self.push()
                self.land_z0 = self.cmd[2]
                self.set_state('LAND', now)
                return
            x, y = self.tri_point(s % self.perimeter)
            self.cmd[0], self.cmd[1] = x, y
            self.cmd[2] = self.height
            self.push()
            return

        if self.state == 'LAND':
            f = min(1.0, (now - self.phase_t0) / self.land_time)
            land_target = self.home[2] + 0.03
            self.cmd[2] = self.land_z0 + (land_target - self.land_z0) * f
            self.push()
            if now - self.phase_t0 > self.land_time + DONE_GRACE_S:
                self.disarm()
                self.save()
                self.set_state('DONE', now)
            return

        if self.state == 'DONE':
            if not self.done_logged:
                self.get_logger().info('triangle complete')
                self.done_logged = True
            return

    def save(self):
        if not self.log:
            self.get_logger().warn('no flight data recorded')
            return
        with open(CSV_PATH, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(self.log[0].keys()))
            w.writeheader()
            w.writerows(self.log)
        tri = [r for r in self.log if r['state'] == 'TRIANGLE']
        if tri:
            errs = [math.hypot(r['x'] - r['ex'], r['y'] - r['ey']) for r in tri]
            self.get_logger().info(
                f'tracking error (TRIANGLE phase, {len(errs)} samples): '
                f'mean={sum(errs)/len(errs):.3f} m  max={max(errs):.3f} m')
        self.get_logger().info(f'CSV -> {CSV_PATH} ({len(self.log)} rows)')


def main():
    rclpy.init()
    node = TriangleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.save()
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
