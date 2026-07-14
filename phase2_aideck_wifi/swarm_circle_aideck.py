#!/usr/bin/env python3
"""
swarm_circle_aideck.py — TWO-DRONE SYNCHRONIZED CIRCLE, AI-DECK ERA
====================================================================
Crazyflie 2.1+ x2 | AI-deck Wi-Fi + Flow deck v2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

Converted from the LPS-era swarm_circle.py. Same behavior — both drones on
one physical circle, 180 degrees apart, staggered altitudes — but rebuilt for
per-drone coordinate frames (Flow deck: each drone's origin = its own takeoff
point, axes = its boot heading). Transport is whatever the crazyswarm2 server
uses; with tcp:// URIs this flies entirely over Wi-Fi, no Crazyradio.

HOW THE SHARED CIRCLE WORKS WITHOUT A SHARED FRAME
  Each drone flies a circle IN ITS OWN FRAME with its phase baked in:
     center_own = -R * (cos(phase), sin(phase));  pos(t) = center_own +
     R*(cos(w*t + phase), sin(w*t + phase))   -> pos(0) == own takeoff point.
  PHYSICAL PLACEMENT then makes the two own-frame circles coincide in the room:
     * Put cf2 exactly 2*RADIUS meters from cf231 along the line that both
       noses point across (cf231 at rim angle 0, cf2 at rim angle 180).
       With RADIUS=1.0: cf2 sits 2.0 m in the -X direction from cf231.
     * BOTH DRONES MUST FACE THE SAME DIRECTION AT BOOT — Flow frames inherit
       boot yaw; misaligned noses = rotated frames = silently wrong geometry.

SAFETY
  * PREFLIGHT gate: refuses to arm unless the server's mirrored firmware
    params show deck.bcFlow2 == 1 and stabilizer.estimator == 2 (Kalman).
    (Added after 2026-07-10: position setpoints without positioning flip
    the drone at takeoff.) Override: --ros-args -p skip_preflight:=true
  * Per-drone excursion clamp replaces the LPS room safe-zone: commands are
    clamped to MAX_EXCURSION from each drone's own takeoff point.
  * Staggered altitudes (0.6 / 0.9 m) kept from the original.

Run (server must be up: ros2 launch crazyflie launch.py backend:=cflib mocap:=False):
  python3 swarm_circle_aideck.py
"""

import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import csv, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

# ── Drone names (must match crazyflies.yaml) ──
D1, D2 = "cf231", "cf2"

# ── Circle parameters (one PHYSICAL circle; per-drone frames) ──
RADIUS = 1.0            # circle radius (m)
OMEGA  = 0.4            # angular speed (rad/s)
LAPS   = 1.0            # full loops to fly
PHASE  = {D1: 0.0, D2: math.pi}   # rim angle each drone starts (and is PLACED) at

# Physical placement vector D1 -> D2 in the room, implied by the phases:
# rim(pi) - rim(0) = (-2R, 0). Used ONLY to report true physical separation.
PLACEMENT = (-2.0 * RADIUS, 0.0)

# ── Altitudes (staggered for vertical safety, heights above own takeoff) ──
ALT = {D1: 0.6, D2: 0.9}

# ── Timing ──
TAKEOFF_TIME = 3.0
POSE_SETTLE  = 3.0      # was LPS_SETTLE; Kalman+Flow also likes a moment
DT           = 0.25     # GoTo update period during the circle

CIRCLE_DURATION = LAPS * 2 * math.pi / OMEGA

# ── Per-drone excursion clamp (replaces the LPS room safe-zone) ──
# The circle takes each drone up to 2R from its own start; allow margin.
MAX_EXCURSION = 2.0 * RADIUS + 0.4

CSV_PATH = os.path.expanduser("~/swarm_circle_aideck_log.csv")


class PreflightChecker:
    """Refuse to arm unless Flow deck (deck.bcFlow2==1) and Kalman
    (stabilizer.estimator==2) are confirmed via the server's mirrored
    firmware params. Fails CLOSED if the params can't be read."""

    def __init__(self, node, ns, server_node='crazyflie_server', timeout=10.0):
        self.node = node; self.ns = ns; self.timeout = timeout
        self.names = [f'{ns}.params.deck.bcFlow2',
                      f'{ns}.params.stabilizer.estimator']
        self.client = node.create_client(GetParameters,
                                         f'/{server_node}/get_parameters')
        self.future = None; self.t0 = None

    @staticmethod
    def _num(pv):
        if pv.type == 2: return pv.integer_value
        if pv.type == 3: return pv.double_value
        return None

    def poll(self, now):
        log = self.node.get_logger()
        if self.t0 is None: self.t0 = now
        if now - self.t0 > self.timeout:
            log.error(f'[{self.ns}] preflight: could not verify within '
                      f'{self.timeout:.0f}s — server up and connected?')
            return 'FAIL'
        if self.future is None:
            if self.client.service_is_ready():
                req = GetParameters.Request(); req.names = self.names
                self.future = self.client.call_async(req)
            return 'PENDING'
        if not self.future.done(): return 'PENDING'
        try:
            v = self.future.result().values
            flow, est = self._num(v[0]), self._num(v[1])
        except Exception as exc:
            log.error(f'[{self.ns}] preflight: bad response ({exc})'); return 'FAIL'
        if flow is None or est is None:
            log.error(f'[{self.ns}] preflight: params not found on server'); return 'FAIL'
        if int(flow) >= 1 and int(est) == 2:
            log.info(f'[{self.ns}] preflight PASS: Flow deck + Kalman'); return 'PASS'
        log.error(f'[{self.ns}] preflight FAIL: deck.bcFlow2={int(flow)} (need 1), '
                  f'stabilizer.estimator={int(est)} (need 2=Kalman) — refusing to arm')
        return 'FAIL'


class SwarmCircleAideck(Node):
    def __init__(self):
        super().__init__("swarm_circle_aideck")
        self._finished = False
        self.pose = {D1: None, D2: None}
        self.home = {D1: None, D2: None}          # own-frame takeoff points
        self.setpoint = {D1: (0, 0, ALT[D1]), D2: (0, 0, ALT[D2])}
        self.log_data = []
        self.t0 = None
        self.circle_t0 = None
        self.state = "WAIT_POSE"
        self._pending = {D1: None, D2: None}
        self._wait_until = 0

        self.declare_parameter('skip_preflight', False)
        self.skip_preflight = bool(self.get_parameter('skip_preflight').value)

        self.arm_cli = {}; self.tk_cli = {}; self.goto_cli = {}; self.land_cli = {}
        self.preflight = {}
        for d in (D1, D2):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m, dd=d: self._pcb(dd, m), 10)
            self.arm_cli[d]  = self.create_client(Arm,     f"/{d}/arm")
            self.tk_cli[d]   = self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d] = self.create_client(GoTo,    f"/{d}/go_to")
            self.land_cli[d] = self.create_client(Land,    f"/{d}/land")
            self.preflight[d] = PreflightChecker(self, d)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"SwarmCircleAideck: one physical circle r={RADIUS}, omega={OMEGA}, "
            f"{D1} at rim 0 deg, {D2} at rim 180 deg. PLACE {D2} "
            f"{2*RADIUS:.1f} m in -X from {D1}, BOTH NOSES SAME DIRECTION.")

    # ---------- callbacks / helpers ----------

    def _pcb(self, d, m):
        self.pose[d] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        if self.t0 is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            row = {"time": round(t, 3)}
            for dd in (D1, D2):
                p = self.pose[dd] or (0, 0, 0); s = self.setpoint[dd]
                row[f"{dd}_x"], row[f"{dd}_y"], row[f"{dd}_z"] = p
                row[f"{dd}_ex"], row[f"{dd}_ey"], row[f"{dd}_ez"] = s
            if self.pose[D1] and self.pose[D2]:
                row["separation_xy"] = self._physical_separation()
            self.log_data.append(row)

    def _physical_separation(self):
        """True room-frame separation. Poses live in DIFFERENT per-drone
        frames now, so raw pose deltas are meaningless — reconstruct via the
        known placement vector (assumes aligned boot headings)."""
        a, b = self.pose[D1], self.pose[D2]
        ha, hb = self.home[D1] or (0, 0, 0), self.home[D2] or (0, 0, 0)
        dx = PLACEMENT[0] + (b[0] - hb[0]) - (a[0] - ha[0])
        dy = PLACEMENT[1] + (b[1] - hb[1]) - (a[1] - ha[1])
        return math.hypot(dx, dy)

    def _now(self):  return self.get_clock().now().nanoseconds
    def _past(self): return self._now() > self._wait_until
    def _both_pose(self): return all(self.pose[d] is not None for d in (D1, D2))
    def _both_done(self):
        return all(self._pending[d] is not None and self._pending[d].done()
                   for d in (D1, D2))

    def _goto(self, d, x, y, z, dur):
        # clamp to excursion budget around OWN takeoff point
        hx, hy, _ = self.home[d]
        ex, ey = x - hx, y - hy
        r = math.hypot(ex, ey)
        if r > MAX_EXCURSION:
            s = MAX_EXCURSION / r
            x, y = hx + ex * s, hy + ey * s
            self.get_logger().warn(f"{d}: goto clamped to excursion budget")
        req = GoTo.Request()
        req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
        req.yaw = 0.0
        req.duration.sec = int(dur); req.duration.nanosec = int((dur % 1) * 1e9)
        req.relative = False           # absolute IN THIS DRONE'S OWN FRAME
        self.setpoint[d] = (x, y, z)
        return self.goto_cli[d].call_async(req)

    def _circle_point(self, d, t):
        """Circle point in drone d's OWN frame. pos(0) == its takeoff point."""
        hx, hy, _ = self.home[d]
        ph = PHASE[d]
        cx = hx - RADIUS * math.cos(ph)
        cy = hy - RADIUS * math.sin(ph)
        th = OMEGA * t + ph
        return cx + RADIUS * math.cos(th), cy + RADIUS * math.sin(th), ALT[d]

    # ---------- state machine ----------

    def _tick(self):
        s = self.state

        if s == "WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until == 0:
                self._wait_until = self._now() + int(POSE_SETTLE * 1e9)
                self.get_logger().info(f"Both have pose. Settling {POSE_SETTLE}s ...")
                for d in (D1, D2):
                    self.get_logger().info(
                        f"  {d}: x={self.pose[d][0]:.2f} y={self.pose[d][1]:.2f} "
                        f"z={self.pose[d][2]:.2f}")
                return
            if not self._past(): return
            for d in (D1, D2):
                self.home[d] = self.pose[d]      # capture own-frame origins
            self.t0 = self._now() / 1e9
            self.state = "PREFLIGHT"

        elif s == "PREFLIGHT":
            if self.skip_preflight:
                self.get_logger().warn("preflight SKIPPED by parameter")
                self.state = "ARM"; return
            now_s = self._now() / 1e9
            results = [self.preflight[d].poll(now_s) for d in (D1, D2)]
            if any(r == 'FAIL' for r in results):
                self.get_logger().fatal(
                    "GROUNDED: flight-readiness failed on at least one drone. "
                    "Fix hardware (Flow deck + Kalman) or use "
                    "-p skip_preflight:=true after verifying the console.")
                self.state = "DONE"
            elif all(r == 'PASS' for r in results):
                self.state = "ARM"

        elif s == "ARM":
            if self._pending[D1] is None:
                for d in (D1, D2):
                    r = Arm.Request(); r.arm = True
                    self._pending[d] = self.arm_cli[d].call_async(r)
                self.get_logger().info("Arming both ...")
            elif self._both_done():
                self.get_logger().info("Both armed.")
                self._pending = {D1: None, D2: None}
                self.state = "TAKEOFF"

        elif s == "TAKEOFF":
            if self._pending[D1] is None:
                for d in (D1, D2):
                    r = Takeoff.Request(); r.height = ALT[d]
                    r.duration.sec = int(TAKEOFF_TIME)
                    r.duration.nanosec = int((TAKEOFF_TIME % 1) * 1e9)
                    self._pending[d] = self.tk_cli[d].call_async(r)
                self._wait_until = self._now() + int((TAKEOFF_TIME + 1.5) * 1e9)
                self.get_logger().info(
                    f"Takeoff: {D1}->{ALT[D1]}m  {D2}->{ALT[D2]}m")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.state = "APPROACH"

        elif s == "APPROACH":
            # circle start == own takeoff XY (rim entry), so this just
            # corrects any takeoff drift back to the start point
            if self._pending[D1] is None:
                self._pending[D1] = self._goto(D1, *self._circle_point(D1, 0), 3.0)
                self._pending[D2] = self._goto(D2, *self._circle_point(D2, 0), 3.0)
                self._wait_until = self._now() + int(4.0 * 1e9)
                self.get_logger().info("Aligning on circle start points ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.circle_t0 = self._now() / 1e9
                self.get_logger().info(f"Flying synchronized circle ({LAPS} lap/s) ...")
                self.state = "CIRCLE"

        elif s == "CIRCLE":
            t = self._now() / 1e9 - self.circle_t0
            if t > CIRCLE_DURATION:
                self.state = "SETTLE"; self._pending = {D1: None, D2: None}; return
            self._goto(D1, *self._circle_point(D1, t), DT)
            self._goto(D2, *self._circle_point(D2, t), DT)
            self._wait_until = self._now() + int(DT * 1e9)
            if self.pose[D1] and self.pose[D2] and int(t * 4) % 8 == 0:
                self.get_logger().info(
                    f"  t={t:4.1f}s  physical separation={self._physical_separation():.2f}m")

        elif s == "SETTLE":
            if self._pending[D1] is None:
                self._pending[D1] = self._goto(D1, *self._circle_point(D1, 0), 2.0)
                self._pending[D2] = self._goto(D2, *self._circle_point(D2, 0), 2.0)
                self._wait_until = self._now() + int(3.0 * 1e9)
                self.get_logger().info("Settling before landing ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.state = "LAND"

        elif s == "LAND":
            if self._pending[D1] is None:
                for d in (D1, D2):
                    r = Land.Request(); r.height = 0.0
                    r.duration.sec = 3; r.duration.nanosec = 0
                    self._pending[d] = self.land_cli[d].call_async(r)
                self._wait_until = self._now() + int(5.0 * 1e9)
                self.get_logger().info("Both landing ...")
            elif self._both_done() and self._past():
                self._pending = {D1: None, D2: None}
                self.get_logger().info("Both landed.")
                self.state = "SAVE"

        elif s == "SAVE":
            self._save()
            self.get_logger().info("Swarm circle complete.")
            self.state = "DONE"

        elif s == "DONE":
            self._finished = True

    def _save(self):
        if not self.log_data:
            self.get_logger().warn("No data to save."); return
        keys = ["time"]
        for d in (D1, D2):
            keys += [f"{d}_x", f"{d}_y", f"{d}_z", f"{d}_ex", f"{d}_ey", f"{d}_ez"]
        keys += ["separation_xy"]
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        seps = [r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps:
            self.get_logger().info(
                f"Physical separation: min={min(seps):.2f} max={max(seps):.2f} "
                f"mean={sum(seps)/len(seps):.2f} m (target ~{2*RADIUS:.1f} at 180 deg; "
                f"drift beyond that = flow-frame divergence, expected to grow slowly)")
        try:
            import swarm_plots
            png = swarm_plots.auto_plot(CSV_PATH, title="Swarm Circle AI-deck (180 deg)")
            if png:
                self.get_logger().info(f"PNG -> {png}")
        except Exception as e:
            self.get_logger().warn(f"PNG generation skipped: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCircleAideck()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try: node._save()
        except Exception: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
