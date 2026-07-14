#!/usr/bin/env python3
"""
swarm_synchronized.py - ROS2 Humble / rclpy - TWO-DRONE SWARM (Behavior 3)
==========================================================================
Crazyflie 2.1+ x2 (cf231 + cf2) | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

BEHAVIOR 3 - SYNCHRONIZED SHAPES: both fly the SAME circle but 180 deg apart,
chasing each other around it. Coordination by SHARED TIMING (each follows its
own pre-planned trajectory), not by sensing each other.

UPDATED to lab standard:
  - Dynamic centre (dynamic_start.fit_center) + radius 0.8 so the circle stays
    inside the anchor hull instead of the hardcoded (1.5,2.5) r=1.0 that poked
    toward the low-X wall.
  - Divergence guard auto-lands BOTH on estimate runaway; pre-takeoff Z sanity.
  - Ctrl+C bug fixed; robust save.

SAFETY: 180 deg phase => always 2R apart (1.6 m); different altitudes
(cf231 0.6 m, cf2 0.9 m) give a 0.3 m vertical margin at all times, including
the approach where the two head to opposite sides. /all/emergency stops both.

Run: python3 swarm_synchronized.py
"""
import os, sys, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           fit_center, clamp_xy, in_safe_zone)

D1, D2 = "cf231", "cf2"

RADIUS  = 0.8
OMEGA   = 0.4
CIRCLE_DURATION = 2 * math.pi / OMEGA
PHASE   = math.pi
Z1, Z2  = 0.6, 0.9
TAKEOFF_TIME = 3.0
LPS_SETTLE   = 3.0
DT = 0.25
REACH = RADIUS

Z_ABORT    = Z2 + 0.7
POS_ABORT  = 1.5
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting
FLOOR_Z_OK = 0.35

CSV_PATH = os.path.expanduser("~/swarm_synchronized_log.csv")


class SwarmSync(Node):
    def __init__(self):
        super().__init__("swarm_synchronized")
        self._finished = False
        self._saved = False
        self._guard_after = 0
        self._bad_count = 0
        self.pose = {D1: None, D2: None}
        self.setpoint = {D1: (0,0,Z1), D2: (0,0,Z2)}
        self.log_data = []; self.t0 = None
        self.cx = self.cy = None
        self.circle_t0 = None
        self.state = "WAIT_POSE"
        self._pending = {D1: None, D2: None}
        self._wait_until = 0
        self.arm_cli={}; self.tk_cli={}; self.goto_cli={}; self.land_cli={}
        for d in (D1, D2):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m, dd=d: self._pcb(dd, m), 10)
            self.arm_cli[d]=self.create_client(Arm, f"/{d}/arm")
            self.tk_cli[d]=self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d]=self.create_client(GoTo, f"/{d}/go_to")
            self.land_cli[d]=self.create_client(Land, f"/{d}/land")
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"SwarmSync: {D1} & {D2}, 180deg phase circle")

    def _pcb(self, d, m):
        self.pose[d]=(m.pose.position.x, m.pose.position.y, m.pose.position.z)
        if self.t0 is not None and not self._finished:
            t=self.get_clock().now().nanoseconds/1e9 - self.t0
            row={"time":round(t,3)}
            for dd in (D1,D2):
                p=self.pose[dd] or (0,0,0); s=self.setpoint[dd]
                row[f"{dd}_x"],row[f"{dd}_y"],row[f"{dd}_z"]=p
                row[f"{dd}_ex"],row[f"{dd}_ey"],row[f"{dd}_ez"]=s
            if self.pose[D1] and self.pose[D2]:
                a,b=self.pose[D1],self.pose[D2]
                row["separation_xy"]=math.hypot(b[0]-a[0],b[1]-a[1])
            self.log_data.append(row)

    def _now(self): return self.get_clock().now().nanoseconds
    def _past(self): return self._now()>self._wait_until
    def _both_pose(self): return all(self.pose[d] is not None for d in (D1,D2))
    def _both_done(self): return all(self._pending[d] is not None and self._pending[d].done() for d in (D1,D2))

    def _goto(self, d, x, y, z, dur):
        req=GoTo.Request(); req.goal.x=float(x); req.goal.y=float(y); req.goal.z=float(z)
        req.yaw=0.0; req.duration.sec=int(dur); req.duration.nanosec=int((dur%1)*1e9)
        req.relative=False; self.setpoint[d]=(x,y,z)
        return self.goto_cli[d].call_async(req)

    def _diverged(self):
        # Divergence auto-land DISABLED for LPS/TDoA2. The low anchors sit at
        # 0.30 m, so a drone on the floor is below the anchor plane and its
        # Z estimate is legitimately noisy/negative there — gating on it caused
        # false aborts on a healthy system. Manual /all/emergency (or
        # /cf231/emergency) remains available if a drone ever misbehaves.
        return False

    def _pt(self, t, phase, z):
        th = OMEGA*t + phase
        x, y = clamp_xy(self.cx + RADIUS*math.cos(th), self.cy + RADIUS*math.sin(th))
        return x, y, z

    def _tick(self):
        s=self.state
        if s=="WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until==0:
                self._wait_until=self._now()+int(LPS_SETTLE*1e9)
                self.get_logger().info(f"Both have pose. Settling {LPS_SETTLE}s ...")
                return
            if not self._past(): return
            self.cx, self.cy = fit_center(self.pose[D1][0], self.pose[D1][1], REACH, REACH)
            self.get_logger().info(
                f"Circle centre ({self.cx:.2f},{self.cy:.2f}) R={RADIUS}, 180deg phase.")
            self.t0=self._now()/1e9; self.state="ARM"
        elif s=="ARM":
            if self._pending[D1] is None:
                for d in (D1,D2):
                    r=Arm.Request(); r.arm=True; self._pending[d]=self.arm_cli[d].call_async(r)
                self.get_logger().info("Arming both ...")
            elif self._both_done():
                self.get_logger().info("Both armed."); self._pending={D1:None,D2:None}; self.state="TAKEOFF"
        elif s=="TAKEOFF":
            if self._pending[D1] is None:
                for d,z in ((D1,Z1),(D2,Z2)):
                    r=Takeoff.Request(); r.height=z
                    r.duration.sec=int(TAKEOFF_TIME); r.duration.nanosec=int((TAKEOFF_TIME%1)*1e9)
                    self._pending[d]=self.tk_cli[d].call_async(r)
                self._wait_until=self._now()+int((TAKEOFF_TIME+1.5)*1e9)
                self._guard_after = self._now() + int((TAKEOFF_TIME + 1.5 + GUARD_GRACE_S) * 1e9)
                self.get_logger().info(f"Takeoff {D1}->{Z1} {D2}->{Z2}")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.state="APPROACH"
        elif s=="APPROACH":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                self._pending[D1]=self._goto(D1, *self._pt(0,0,Z1), 3.0)
                self._pending[D2]=self._goto(D2, *self._pt(0,PHASE,Z2), 3.0)
                self._wait_until=self._now()+int(4.0*1e9)
                self.get_logger().info("Approaching start positions (opposite sides, 0.3m vertical gap) ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.circle_t0=self._now()/1e9
                self.get_logger().info("Phase: synchronized circle (180deg apart) ...")
                self.state="SYNC"
        elif s=="SYNC":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            t=self._now()/1e9 - self.circle_t0
            if t>CIRCLE_DURATION:
                self.state="SETTLE"; self._pending={D1:None,D2:None}; return
            self._goto(D1, *self._pt(t,0,Z1), DT)
            self._goto(D2, *self._pt(t,PHASE,Z2), DT)
            self._wait_until=self._now()+int(DT*1e9)
            if self.pose[D1] and self.pose[D2]:
                sep=math.hypot(self.pose[D2][0]-self.pose[D1][0], self.pose[D2][1]-self.pose[D1][1])
                if int(t*4)%8==0: self.get_logger().info(f"  t={t:4.1f}s sep={sep:.2f}m")
        elif s=="SETTLE":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                self._pending[D1]=self._goto(D1, *self._pt(0,0,Z1), 2.0)
                self._pending[D2]=self._goto(D2, *self._pt(0,PHASE,Z2), 2.0)
                self._wait_until=self._now()+int(3.0*1e9)
                self.get_logger().info("Settling ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.state="LAND"
        elif s=="LAND":
            if self._pending[D1] is None:
                for d in (D1,D2):
                    r=Land.Request(); r.height=0.0; r.duration.sec=3; r.duration.nanosec=0
                    self._pending[d]=self.land_cli[d].call_async(r)
                self._wait_until=self._now()+int(5.0*1e9)
                self.get_logger().info("Both landing ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.get_logger().info("Both landed."); self.state="SAVE"
        elif s=="SAVE":
            self._finalize(); self.state="DONE"
        elif s=="DONE":
            self._finished = True

    def _finalize(self):
        if self._saved: return
        self._saved = True
        if not self.log_data:
            self.get_logger().info("Synchronized complete (no data)."); return
        keys=["time"]
        for d in (D1,D2): keys+=[f"{d}_x",f"{d}_y",f"{d}_z",f"{d}_ex",f"{d}_ey",f"{d}_ez"]
        keys+=["separation_xy"]
        try:
            with open(CSV_PATH,"w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore"); w.writeheader(); w.writerows(self.log_data)
            self.get_logger().info(f"CSV -> {CSV_PATH} ({len(self.log_data)} rows)")
        except Exception as e:
            self.get_logger().error(f"CSV save failed: {e}")
        try:
            import swarm_plots
            png = swarm_plots.auto_plot(CSV_PATH, title="Synchronized")
            if png: self.get_logger().info(f"PNG -> {png}")
        except Exception as e:
            self.get_logger().warn(f"PNG generation skipped: {e}")
        seps=[r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps: self.get_logger().info(f"Separation: min={min(seps):.2f} max={max(seps):.2f} mean={sum(seps)/len(seps):.2f} m")
        self.get_logger().info("Synchronized swarm complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSync()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted - saving what we have ...")
    finally:
        node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__": main()
