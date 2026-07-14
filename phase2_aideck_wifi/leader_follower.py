#!/usr/bin/env python3
"""
leader_follower.py — Option A leader-follower in ONE file
(pure rclpy + crazyflie_interfaces, no cflib)

AI-deck leader-follower, Option A: the ROS2 graph is the comms link.
Both nodes run in this one process on a single executor:

  * LeaderNode (cf231) flies a pre-planned trajectory (circle / square /
    triangle) and broadcasts every commanded setpoint on /leader/setpoint
    (PoseStamped) plus its mission phase on /leader/phase (String).
  * FollowerNode (cf2) trails at a FIXED WORLD-FRAME OFFSET (default 0.8 m
    behind in -X): target = leader_setpoint + offset. It never touches the
    leader's measured position — only the commanded trajectory — so
    positioning noise can never create bearings (the original crash cause).

Failsafe (Prof. Kaul's requirement): if no /leader/setpoint arrives for
`watchdog_timeout` s (default 0.5) during FOLLOW, the follower lands in
place. Staleness uses the follower's receive clock, not header stamps.

Follower motion is step-limited (`max_step`/tick, default 0.025 m @ 20 Hz
= 0.5 m/s — slightly faster than the circle's 0.4 m/s rim speed) so a
distant or jumpy target can never fling the drone.

State machines (non-blocking, ticked at `rate` Hz):
  leader:   WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> TRAJECTORY -> LAND -> DONE
  follower: WAIT_POSE -> ARM -> TAKEOFF -> STABILIZE -> WAIT_LEADER -> FOLLOW
                -> LAND (leader finished) | FAILSAFE_LAND (leader lost) -> DONE

Carried-over conventions: gentle 5 s takeoff + stabilize hover, rim-entry
circle start, tuned circle defaults (R=0.8, omega=0.5), no floor-Z gates.

Placement: put the follower on the floor roughly at (leader_start + offset).

Run (one terminal):
    python3 leader_follower.py
    python3 leader_follower.py --ros-args -p trajectory:=square -p side:=1.2
    python3 leader_follower.py --ros-args -p trajectory:=triangle -p laps:=3

Failsafe demo (leader mutes its broadcast 15 s into the trajectory and
keeps flying / lands itself; follower must failsafe-land):
    python3 leader_follower.py --ros-args -p simulate_leader_loss_after:=15.0

Optional split across two terminals (e.g. two laptops later):
    python3 leader_follower.py --role follower
    python3 leader_follower.py --role leader
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

GROUND_Z = 0.06        # legacy touchdown z [m] (landing now uses home_z + 0.03)
DONE_GRACE_S = 1.5     # keep publishing the ground setpoint this long after landing

# Room safe-zone, derived from the 8 LPS anchor positions in B-419
# (anchors span X 0..5.04, Y -0.8..7.4, high plane ~2.5-2.6 m), with margins.
# Extra margin on high-Y where the known +8 cm TDoA2 bias lives.
# Commands are CLAMPED to this box (never refused).
SAFE_X = (0.30, 4.74)
SAFE_Y = (-0.30, 6.90)
SAFE_Z = (-0.50, 2.20)   # low bound allows the LPS negative-floor artifact


def clamp_cmd(node, x, y, z):
    cx = min(max(x, SAFE_X[0]), SAFE_X[1])
    cy = min(max(y, SAFE_Y[0]), SAFE_Y[1])
    cz = min(max(z, SAFE_Z[0]), SAFE_Z[1])
    if (cx, cy, cz) != (x, y, z):
        node.get_logger().warn(
            f'command clamped to room safe-zone: '
            f'({x:.2f},{y:.2f},{z:.2f}) -> ({cx:.2f},{cy:.2f},{cz:.2f})',
            throttle_duration_sec=1.0)
    return cx, cy, cz


# ============================================================ LEADER

class LeaderNode(Node):

    def __init__(self):
        super().__init__('leader_node')

        # ---------------- parameters ----------------
        self.declare_parameter('leader_ns', 'cf231')
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
        # failsafe demo: seconds into TRAJECTORY after which the leader stops
        # broadcasting on /leader/* (still flies + lands itself). 0 = disabled.
        self.declare_parameter('simulate_leader_loss_after', 0.0)

        g = self.get_parameter
        self.ns = g('leader_ns').value
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
        self.loss_after = float(g('simulate_leader_loss_after').value)

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
        self.phase_t0 = 0.0
        self.arm_future = None
        self.arm_wait_t0 = None
        self.center = None               # circle center (rim entry)
        self.segments = None             # polygon segments [((x0,y0),(x1,y1),L), ...]
        self.perimeter = 0.0
        self.total_time = 0.0            # circle duration
        self.total_s = 0.0               # polygon total path length
        self.land_z0 = 0.0
        self.silent = False              # True after simulated comms loss
        self.done_logged = False

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'leader up: ns={self.ns} traj={self.traj} laps={self.laps} '
            f'height={self.height:.2f} m'
            + (f' | comms-loss demo at t={self.loss_after:.0f}s' if self.loss_after > 0 else ''))

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
        self.cmd[0], self.cmd[1], self.cmd[2] = clamp_cmd(self, *self.cmd)
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

        if self.traj == 'circle':
            # Rim entry: center one radius behind the hover point, theta starts
            # at 0, so position(t=0) == hover point.
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
            land_target = self.home[2] + 0.03   # robust to LPS negative-floor artifact
            self.cmd[2] = self.land_z0 + (land_target - self.land_z0) * f
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
        self.declare_parameter('watchdog_timeout', 0.8)   # leader-lost threshold [s]
        self.declare_parameter('max_step', 0.025)         # per-tick move limit [m]
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('takeoff_time', 5.0)
        self.declare_parameter('stabilize_time', 3.0)
        self.declare_parameter('land_time', 4.0)
        self.declare_parameter('wait_leader_timeout', 30.0)  # hover budget [s]
        self.declare_parameter('arm_timeout', 5.0)
        self.declare_parameter('leader_ns', 'cf231')      # for follow_source:=pose
        # 'setpoint' = follow leader's commanded position (robust to noise)
        # 'pose'     = follow leader's MEASURED position (literal LPS reading)
        self.declare_parameter('follow_source', 'setpoint')

        g = self.get_parameter
        self.ns = g('follower_ns').value
        self.leader_ns = g('leader_ns').value
        self.follow_source = str(g('follow_source').value).lower()
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
        if self.follow_source == 'pose':
            # follow the leader's MEASURED position (e.g. LPS) — noise is
            # tamed by the max_step limiter, the anti-crash fix
            self.leader_sub = self.create_subscription(
                PoseStamped, f'/{self.leader_ns}/pose', self.leader_cb, 10)
        else:
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
        self.phase_rx = None             # receive time of last phase heartbeat [s]
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
        self.phase_rx = self.now_s()     # leader-node heartbeat

    def set_state(self, new, now):
        self.get_logger().info(f'{self.state} -> {new}')
        self.state = new
        self.phase_t0 = now

    def push(self):
        self.cmd[0], self.cmd[1], self.cmd[2] = clamp_cmd(self, *self.cmd)
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
            pos_stale = now - self.leader_rx > self.watchdog
            hb_stale = (self.phase_rx is not None
                        and now - self.phase_rx > self.watchdog)
            if pos_stale or hb_stale:
                which = 'position stream' if pos_stale else 'leader heartbeat'
                self.get_logger().warn(
                    f'{which} stale > {self.watchdog:.2f}s — FAILSAFE LAND')
                self.begin_land('FAILSAFE_LAND', now)
                return
            self.step_toward(self.leader[0] + self.off[0],
                             self.leader[1] + self.off[1],
                             self.leader[2] + self.off[2])
            self.push()
            return

        if self.state in ('LAND', 'FAILSAFE_LAND'):
            f = min(1.0, (now - self.phase_t0) / self.land_time)
            land_target = self.home[2] + 0.03   # robust to LPS negative-floor artifact
            self.cmd[2] = self.land_z0 + (land_target - self.land_z0) * f
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
    parser = argparse.ArgumentParser(description='Option A leader-follower')
    parser.add_argument('--role', choices=['both', 'leader', 'follower'],
                        default='both',
                        help="which node(s) to run in this process (default: both)")
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
