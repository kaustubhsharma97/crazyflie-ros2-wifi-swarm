#!/usr/bin/env python3
"""
leader_follower_triangle.py — Option A leader-follower, TRIANGLE trajectory
(pure rclpy + crazyflie_interfaces, no cflib)

One process, two nodes on a single executor:
  * LeaderNode (cf231) flies an EQUILATERAL triangle of side `side` at constant edge speed
    `speed`, starting at the hover point (first corner). Every commanded
    setpoint is broadcast on /leader/setpoint (PoseStamped), mission phase
    on /leader/phase (String).
  * FollowerNode (cf2) trails at a FIXED WORLD-FRAME OFFSET (default 0.8 m
    behind in -X): target = leader_setpoint + offset. It follows only the
    commanded trajectory, never measured position.

Failsafe: if no /leader/setpoint arrives for `watchdog_timeout` s (default
0.5) during FOLLOW, the follower lands in place. Follower motion is
step-limited (`max_step`/tick, default 0.025 m @ 20 Hz = 0.5 m/s).

State machines (non-blocking, ticked at `rate` Hz):
  leader:   WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> TRAJECTORY -> LAND -> DONE
  follower: WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> WAIT_LEADER -> FOLLOW
                -> LAND (leader finished) | FAILSAFE_LAND (leader lost) -> DONE

Placement: follower on the floor roughly at (leader_start + offset).

Run (one terminal):
    python3 leader_follower_triangle.py
    python3 leader_follower_triangle.py --ros-args -p side:=1.2 -p laps:=3

Failsafe demo (leader mutes its broadcast mid-trajectory, keeps flying and
lands itself; follower must failsafe-land):
    python3 leader_follower_triangle.py --ros-args -p simulate_leader_loss_after:=15.0

Optional split across two terminals:
    python3 leader_follower_triangle.py --role follower
    python3 leader_follower_triangle.py --role leader
"""

import argparse
import math
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm

GROUND_Z = 0.06        # commanded z at touchdown [m]
DONE_GRACE_S = 1.5     # keep publishing the ground setpoint this long after landing


# ============================================================ LEADER

class LeaderNode(Node):

    def __init__(self):
        super().__init__('leader_node')

        # ---------------- parameters ----------------
        self.declare_parameter('leader_ns', 'cf231')
        self.declare_parameter('side', 1.0)              # equilateral triangle side [m]
        self.declare_parameter('speed', 0.25)            # edge speed [m/s]
        self.declare_parameter('height', 1.0)            # cruise height [m]
        self.declare_parameter('laps', 2)
        self.declare_parameter('takeoff_time', 5.0)      # gentle takeoff ramp [s]
        self.declare_parameter('stabilize_time', 3.0)    # hover before trajectory [s]
        self.declare_parameter('land_time', 4.0)         # landing ramp [s]
        self.declare_parameter('rate', 20.0)             # control loop [Hz]
        self.declare_parameter('arm_timeout', 5.0)       # continue w/o arm service (sim)
        # failsafe demo: seconds into TRAJECTORY after which the leader stops
        # broadcasting on /leader/* (still flies + lands itself). 0 = disabled.
        self.declare_parameter('simulate_leader_loss_after', 0.0)

        g = self.get_parameter
        self.ns = g('leader_ns').value
        self.side = float(g('side').value)
        self.speed = float(g('speed').value)
        self.height = float(g('height').value)
        self.laps = int(g('laps').value)
        self.takeoff_time = float(g('takeoff_time').value)
        self.stabilize_time = float(g('stabilize_time').value)
        self.land_time = float(g('land_time').value)
        self.rate = float(g('rate').value)
        self.arm_timeout = float(g('arm_timeout').value)
        self.loss_after = float(g('simulate_leader_loss_after').value)

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
        self.phase_t0 = 0.0
        self.arm_future = None
        self.arm_wait_t0 = None
        self.segments = None             # [((x0,y0),(x1,y1),L), ...] closed loop
        self.perimeter = 0.0
        self.total_s = 0.0               # total path length over all laps
        self.land_z0 = 0.0
        self.silent = False              # True after simulated comms loss
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'leader up: ns={self.ns} traj=triangle side={self.side} '
            f'speed={self.speed} laps={self.laps} height={self.height:.2f} m'
            + (f' | comms-loss demo at t={self.loss_after:.0f}s'
               if self.loss_after > 0 else ''))

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
        if self.silent:
            return
        m = String()
        m.data = self.state
        self.phase_pub.publish(m)

    def push(self):
        """cmd_position always goes to the drone; /leader/* only while not silent."""
        m = Position()
        m.x, m.y, m.z = float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])
        m.yaw = 0.0
        self.cmd_pub.publish(m)

        if self.silent:
            return
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
        L = self.side
        # vertices of the closed equilateral triangle, starting at the hover point
        verts = [(bx, by), (bx + L, by),
                 (bx + L / 2.0, by + L * math.sqrt(3) / 2.0), (bx, by)]

        self.segments = []
        self.perimeter = 0.0
        for a, b in zip(verts[:-1], verts[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            self.segments.append((a, b, seg_len))
            self.perimeter += seg_len
        self.total_s = self.perimeter * self.laps
        self.get_logger().info(
            f'triangle: side={L} perimeter={self.perimeter:.2f} m '
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
                    f'leader home: ({self.home[0]:.2f}, {self.home[1]:.2f}, '
                    f'{self.home[2]:.2f})')
                self.set_state('ARM', now)
            else:
                self.get_logger().info('leader waiting for pose...',
                                       throttle_duration_sec=2.0)
            self.publish_phase()
            return

        if self.state == 'ARM':
            if self.arm_future is not None:
                if self.arm_future.done():
                    self.get_logger().info('leader armed')
                    self.set_state('TAKEOFF', now)
            elif self.arm_client.service_is_ready():
                req = Arm.Request()
                req.arm = True
                self.arm_future = self.arm_client.call_async(req)
                self.get_logger().info('leader arming...')
            else:
                if self.arm_wait_t0 is None:
                    self.arm_wait_t0 = now
                if now - self.arm_wait_t0 > self.arm_timeout:
                    self.get_logger().warn(
                        'leader arm service unavailable — continuing (sim?)')
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

            # failsafe demo: mute the broadcast mid-trajectory, keep flying
            if self.loss_after > 0.0 and not self.silent and t >= self.loss_after:
                self.silent = True
                self.get_logger().warn(
                    'SIMULATED COMMS LOSS — /leader/* muted, leader continues '
                    'and lands itself; follower should failsafe-land')

            s = self.speed * t
            if s >= self.total_s:
                self.cmd[2] = self.height
                self.push()
                self.land_z0 = self.cmd[2]
                self.set_state('LAND', now)
                return
            x, y = self.polygon_point(s % self.perimeter)
            self.cmd[0], self.cmd[1] = x, y
            self.cmd[2] = self.height
            self.push()
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
                self.get_logger().info('leader mission complete')
                self.done_logged = True
            return


# ============================================================ FOLLOWER

class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower_node')

        # ---------------- parameters ----------------
        self.declare_parameter('follower_ns', 'cf2')
        self.declare_parameter('offset_x', -0.8)          # world-frame offset [m]
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('height', 1.0)             # hover height while waiting [m]
        self.declare_parameter('watchdog_timeout', 0.5)   # leader-lost threshold [s]
        self.declare_parameter('max_step', 0.025)         # per-tick move limit [m]
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('takeoff_time', 5.0)
        self.declare_parameter('stabilize_time', 3.0)
        self.declare_parameter('land_time', 4.0)
        self.declare_parameter('wait_leader_timeout', 30.0)  # hover budget [s]
        self.declare_parameter('arm_timeout', 5.0)

        g = self.get_parameter
        self.ns = g('follower_ns').value
        self.off = (float(g('offset_x').value),
                    float(g('offset_y').value),
                    float(g('offset_z').value))
        self.height = float(g('height').value)
        self.watchdog = float(g('watchdog_timeout').value)
        self.max_step = float(g('max_step').value)
        self.rate = float(g('rate').value)
        self.takeoff_time = float(g('takeoff_time').value)
        self.stabilize_time = float(g('stabilize_time').value)
        self.land_time = float(g('land_time').value)
        self.wait_leader_timeout = float(g('wait_leader_timeout').value)
        self.arm_timeout = float(g('arm_timeout').value)

        # ---------------- pubs / subs / clients ----------------
        self.cmd_pub = self.create_publisher(Position, f'/{self.ns}/cmd_position', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.ns}/pose', self.pose_cb, 10)
        self.leader_sub = self.create_subscription(
            PoseStamped, '/leader/setpoint', self.leader_cb, 10)
        self.leader_phase_sub = self.create_subscription(
            String, '/leader/phase', self.phase_cb, 10)
        self.arm_client = self.create_client(Arm, f'/{self.ns}/arm')

        # ---------------- state ----------------
        self.state = 'WAIT_POSE'
        self.home = None                 # (x, y, z) captured on first pose
        self.cmd = [0.0, 0.0, 0.0]       # current commanded position
        self.phase_t0 = 0.0
        self.arm_future = None
        self.arm_wait_t0 = None
        self.leader = None               # last leader setpoint (x, y, z)
        self.leader_rx = None            # receive time of last setpoint [s]
        self.leader_phase = ''
        self.land_z0 = 0.0
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'follower up: ns={self.ns} offset=({self.off[0]:+.2f}, '
            f'{self.off[1]:+.2f}, {self.off[2]:+.2f}) '
            f'watchdog={self.watchdog:.2f}s max_step={self.max_step} m/tick')

    # ---------------- helpers ----------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_cb(self, msg: PoseStamped):
        if self.home is None:
            p = msg.pose.position
            self.home = (p.x, p.y, p.z)

    def leader_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.leader = (p.x, p.y, p.z)
        self.leader_rx = self.now_s()    # receive clock, not header stamp

    def phase_cb(self, msg: String):
        self.leader_phase = msg.data

    def set_state(self, new, now):
        self.get_logger().info(f'{self.state} -> {new}')
        self.state = new
        self.phase_t0 = now

    def push(self):
        m = Position()
        m.x, m.y, m.z = float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])
        m.yaw = 0.0
        self.cmd_pub.publish(m)

    def step_toward(self, tx, ty, tz):
        """Move the commanded position toward the target, capped at max_step."""
        dx, dy, dz = tx - self.cmd[0], ty - self.cmd[1], tz - self.cmd[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > self.max_step and dist > 0.0:
            s = self.max_step / dist
            dx, dy, dz = dx * s, dy * s, dz * s
        self.cmd[0] += dx
        self.cmd[1] += dy
        self.cmd[2] += dz

    def begin_land(self, land_state, now):
        self.land_z0 = self.cmd[2]
        self.set_state(land_state, now)

    def disarm(self):
        if self.arm_client.service_is_ready():
            req = Arm.Request()
            req.arm = False
            self.arm_client.call_async(req)

    # ---------------- main state machine ----------------

    def tick(self):
        now = self.now_s()

        if self.state == 'WAIT_POSE':
            if self.home is not None:
                self.cmd = [self.home[0], self.home[1], self.home[2]]
                self.get_logger().info(
                    f'follower home: ({self.home[0]:.2f}, {self.home[1]:.2f}, '
                    f'{self.home[2]:.2f})')
                self.set_state('ARM', now)
            else:
                self.get_logger().info('follower waiting for pose...',
                                       throttle_duration_sec=2.0)
            return

        if self.state == 'ARM':
            if self.arm_future is not None:
                if self.arm_future.done():
                    self.get_logger().info('follower armed')
                    self.set_state('TAKEOFF', now)
            elif self.arm_client.service_is_ready():
                req = Arm.Request()
                req.arm = True
                self.arm_future = self.arm_client.call_async(req)
                self.get_logger().info('follower arming...')
            else:
                if self.arm_wait_t0 is None:
                    self.arm_wait_t0 = now
                if now - self.arm_wait_t0 > self.arm_timeout:
                    self.get_logger().warn(
                        'follower arm service unavailable — continuing (sim?)')
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
                self.set_state('WAIT_LEADER', now)
            return

        if self.state == 'WAIT_LEADER':
            self.push()  # hover in place
            fresh = (self.leader_rx is not None
                     and (now - self.leader_rx) < self.watchdog)
            if fresh and self.leader_phase == 'TRAJECTORY':
                self.get_logger().info('leader trajectory detected — following')
                self.set_state('FOLLOW', now)
            elif now - self.phase_t0 > self.wait_leader_timeout:
                self.get_logger().warn(
                    f'no leader within {self.wait_leader_timeout:.0f}s — landing')
                self.begin_land('LAND', now)
            return

        if self.state == 'FOLLOW':
            if self.leader_phase in ('LAND', 'DONE'):
                self.get_logger().info('leader finished — landing')
                self.begin_land('LAND', now)
                return
            if now - self.leader_rx > self.watchdog:
                self.get_logger().warn(
                    f'leader stream stale > {self.watchdog:.2f}s — FAILSAFE LAND')
                self.begin_land('FAILSAFE_LAND', now)
                return
            self.step_toward(self.leader[0] + self.off[0],
                             self.leader[1] + self.off[1],
                             self.leader[2] + self.off[2])
            self.push()
            return

        if self.state in ('LAND', 'FAILSAFE_LAND'):
            f = min(1.0, (now - self.phase_t0) / self.land_time)
            self.cmd[2] = self.land_z0 + (GROUND_Z - self.land_z0) * f
            self.push()
            if now - self.phase_t0 > self.land_time + DONE_GRACE_S:
                self.disarm()
                self.set_state('DONE', now)
            return

        if self.state == 'DONE':
            if not self.done_logged:
                self.get_logger().info('follower flight complete')
                self.done_logged = True
            return


# ============================================================ MAIN

def main():
    parser = argparse.ArgumentParser(
        description='Option A leader-follower (triangle)')
    parser.add_argument('--role', choices=['both', 'leader', 'follower'],
                        default='both',
                        help='which node(s) to run in this process (default: both)')
    cli, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)

    nodes = []
    if cli.role in ('both', 'follower'):
        nodes.append(FollowerNode())
    if cli.role in ('both', 'leader'):
        nodes.append(LeaderNode())

    executor = SingleThreadedExecutor()
    for n in nodes:
        executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
