#!/usr/bin/env python3
"""
follower_node.py — Option A follower (pure rclpy + crazyflie_interfaces, no cflib)

Fixed WORLD-FRAME offset trailing: target = leader_setpoint + (offset_x,
offset_y, offset_z). Default offset is 0.8 m behind in -X. World-frame
offset keeps the geometry simple and noise-free — no bearing computed from
deltas (the thing that caused the original leader-follower crash).

Failsafe (Prof. Kaul's requirement): if no /leader/setpoint arrives for
`watchdog_timeout` seconds (default 0.5 s) during FOLLOW, the follower
lands in place. Staleness is measured with this node's receive clock, not
the message header stamp, so it is immune to clock skew between machines.

Motion toward the target is step-limited (`max_step` per tick, default
0.025 m @ 20 Hz = 0.5 m/s) so a distant or jumpy target can never fling
the drone — the anti-crash fix from the rebuild, kept here. It also makes
the initial approach a smooth glide instead of a lunge.

State machine (non-blocking, ticked at `rate` Hz):
    WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> WAIT_LEADER -> FOLLOW
        -> LAND (leader finished)  or  FAILSAFE_LAND (leader lost) -> DONE

Placement: put the follower on the floor roughly at (leader_start + offset).

Run (defaults: cf2, offset (-0.8, 0, 0), watchdog 0.5 s):
    python3 follower_node.py
    python3 follower_node.py --ros-args -p offset_x:=-1.2 -p watchdog_timeout:=0.7

Launch order: follower FIRST (takes off and hovers waiting), then leader.
Failsafe test: Ctrl-C the leader mid-trajectory — the follower must land.
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


class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower_node')

        # ---------------- parameters ----------------
        self.declare_parameter('drone_ns', 'cf2')
        self.declare_parameter('offset_x', -0.8)          # world-frame offset [m]
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('height', 1.0)             # hover height while waiting [m]
        self.declare_parameter('watchdog_timeout', 0.5)   # leader-lost threshold [s]
        self.declare_parameter('max_step', 0.025)         # per-tick move limit [m]
        self.declare_parameter('rate', 20.0)              # control loop [Hz]
        self.declare_parameter('takeoff_time', 5.0)       # gentle takeoff ramp [s]
        self.declare_parameter('stabilize_time', 3.0)
        self.declare_parameter('land_time', 4.0)
        self.declare_parameter('wait_leader_timeout', 30.0)  # hover budget [s]
        self.declare_parameter('arm_timeout', 5.0)        # continue w/o arm service (sim)

        g = self.get_parameter
        self.ns = g('drone_ns').value
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
        self.leader_rx = None            # receive time of last setpoint [s, node clock]
        self.leader_phase = ''
        self.land_z0 = 0.0
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'follower up: ns={self.ns} offset=({self.off[0]:+.2f}, {self.off[1]:+.2f}, '
            f'{self.off[2]:+.2f}) watchdog={self.watchdog:.2f}s '
            f'max_step={self.max_step} m/tick')

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
                    f'home: ({self.home[0]:.2f}, {self.home[1]:.2f}, {self.home[2]:.2f})')
                self.set_state('ARM', now)
            else:
                self.get_logger().info('waiting for pose...', throttle_duration_sec=2.0)
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
                self.get_logger().info('flight complete')
                self.done_logged = True
            return


def main():
    rclpy.init()
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
