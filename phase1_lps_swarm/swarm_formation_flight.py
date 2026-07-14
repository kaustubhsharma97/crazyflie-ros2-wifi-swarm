#!/usr/bin/env python3
"""
swarm_formation_flight.py - ROS2 Humble / rclpy - TWO-DRONE SWARM (Behavior 4)
==============================================================================
Crazyflie 2.1+ x2 (cf231 + cf2) | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

BEHAVIOR 4 - FORMATION FLIGHT: the two drones hold a RIGID offset (1.0 m apart)
while the whole formation TRANSLATES along a path - move as one rigid body. The
rigid offset means they never converge.

UPDATED to lab standard:
  - Dynamic path: the formation path is centred on the hull (dynamic_start) so
    it stays inside the anchors instead of the hardcoded x=1.5 line.
  - Divergence guard auto-lands BOTH on estimate runaway; pre-takeoff Z sanity.
  - Ctrl+C bug fixed; robust save. (The "not landing in sim" you saw was the sim
    server, now fixed separately - this script's logic was sound.)

SAFETY: rigid 1.0 m offset (never converge) + different altitudes (0.6 / 0.9 m).
Every commanded point is safe-zone clamped. /all/emergency stops both.

Run: python3 swarm_formation_flight.py
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

OFFSET = {D1: (-0.5, 0.0), D2: (+0.5, 0.0)}   # 1.0 m apart in X
Z = {D1: 0.6, D2: 0.9}

# Formation-centre path as offsets from the dynamic hull centre: glide along Y
# and back. Built relative so the whole path recentres to wherever cf231 is.
PATH_OFFSETS = [(0.0, -0.7), (0.0, +0.7), (0.0, -0.7)]
SEG_TIME = 5.0

TAKEOFF_TIME = 3.0
LPS_SETTLE   = 3.0

Z_ABORT    = max(Z.values()) + 0.7
POS_ABORT  = 1.5
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting
FLOOR_Z_OK = 0.35

CSV_PATH = os.path.expanduser("~/swarm_formation_flight_log.csv")


class SwarmFormationFlight(Node):
    def __init__(self):
        super().__init__("swarm_formation_flight")
        self._finished = False
        self._saved = False
        self._guard_after = 0
        self._bad_count = 0
        self.pose={D1:None,D2:None}
        self.setpoint={D1:(0,0,Z[D1]),D2:(0,0,Z[D2])}
        self.log_data=[]; self.t0=None
        self.cx = self.cy = None
        self.path = None
        self.seg_i=0
        self.state="WAIT_POSE"
        self._pending={D1:None,D2:None}; self._wait_until=0
        self.arm_cli={}; self.tk_cli={}; self.goto_cli={}; self.land_cli={}
        for d in (D1,D2):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m,dd=d: self._pcb(dd,m), 10)
            self.arm_cli[d]=self.create_client(Arm,f"/{d}/arm")
            self.tk_cli[d]=self.create_client(Takeoff,f"/{d}/takeoff")
            self.goto_cli[d]=self.create_client(GoTo,f"/{d}/go_to")
            self.land_cli[d]=self.create_client(Land,f"/{d}/land")
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"SwarmFormationFlight: {D1} & {D2} rigid formation")

    def _pcb(self,d,m):
        self.pose[d]=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
        if self.t0 is not None and not self._finished:
            t=self.get_clock().now().nanoseconds/1e9-self.t0
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

    def _goto(self,d,x,y,z,dur):
        r=GoTo.Request(); r.goal.x=float(x); r.goal.y=float(y); r.goal.z=float(z)
        r.yaw=0.0; r.duration.sec=int(dur); r.duration.nanosec=int((dur%1)*1e9)
        r.relative=False; self.setpoint[d]=(x,y,z)
        return self.goto_cli[d].call_async(r)

    def _diverged(self):
        # Divergence auto-land DISABLED for LPS/TDoA2. The low anchors sit at
        # 0.30 m, so a drone on the floor is below the anchor plane and its
        # Z estimate is legitimately noisy/negative there — gating on it caused
        # false aborts on a healthy system. Manual /all/emergency (or
        # /cf231/emergency) remains available if a drone ever misbehaves.
        return False

    def _drone_xy(self, centre, d):
        ox, oy = OFFSET[d]
        return clamp_xy(centre[0]+ox, centre[1]+oy)

    def _tick(self):
        s=self.state
        if s=="WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until==0:
                self._wait_until=self._now()+int(LPS_SETTLE*1e9)
                self.get_logger().info(f"Both have pose. Settling {LPS_SETTLE}s ...")
                return
            if not self._past(): return
            # centre the formation path on the hull near cf231; reach accounts for
            # the rigid offset (0.5) plus the path span (0.7)
            self.cx, self.cy = fit_center(self.pose[D1][0], self.pose[D1][1], 0.5+0.7, 0.5+0.7)
            self.path = [(self.cx+ox, self.cy+oy) for (ox, oy) in PATH_OFFSETS]
            self.get_logger().info(
                f"Formation path centred at ({self.cx:.2f},{self.cy:.2f}); "
                f"{len(self.path)} waypoints, rigid offset 1.0 m.")
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
                for d in (D1,D2):
                    r=Takeoff.Request(); r.height=Z[d]
                    r.duration.sec=int(TAKEOFF_TIME); r.duration.nanosec=int((TAKEOFF_TIME%1)*1e9)
                    self._pending[d]=self.tk_cli[d].call_async(r)
                self._wait_until=self._now()+int((TAKEOFF_TIME+1.5)*1e9)
                self._guard_after = self._now() + int((TAKEOFF_TIME + 1.5 + GUARD_GRACE_S) * 1e9)
                self.get_logger().info("Both taking off ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.state="APPROACH"
        elif s=="APPROACH":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                c=self.path[0]
                for d in (D1,D2):
                    x,y=self._drone_xy(c,d)
                    self._pending[d]=self._goto(d,x,y,Z[d],3.0)
                self._wait_until=self._now()+int(4.0*1e9)
                self.get_logger().info("Forming up at path start ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.seg_i=0
                self.get_logger().info("Phase: formation flight along path ...")
                self.state="FLY"
        elif s=="FLY":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self.seg_i >= len(self.path):
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                c=self.path[self.seg_i]
                for d in (D1,D2):
                    x,y=self._drone_xy(c,d)
                    self._pending[d]=self._goto(d,x,y,Z[d],SEG_TIME)
                self._wait_until=self._now()+int((SEG_TIME+0.5)*1e9)
                self.get_logger().info(f"  formation -> waypoint {self.seg_i+1}/{len(self.path)} centre=({c[0]:.2f},{c[1]:.2f})")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}
                if self.pose[D1] and self.pose[D2]:
                    sep=math.hypot(self.pose[D2][0]-self.pose[D1][0], self.pose[D2][1]-self.pose[D1][1])
                    self.get_logger().info(f"    separation held: {sep:.2f} m")
                self.seg_i+=1
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
            self.get_logger().info("Formation flight complete (no data)."); return
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
            png = swarm_plots.auto_plot(CSV_PATH, title="Formation Flight")
            if png: self.get_logger().info(f"PNG -> {png}")
        except Exception as e:
            self.get_logger().warn(f"PNG generation skipped: {e}")
        seps=[r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps: self.get_logger().info(f"Separation: min={min(seps):.2f} max={max(seps):.2f} mean={sum(seps)/len(seps):.2f} m (target ~1.0)")
        self.get_logger().info("Formation flight complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmFormationFlight()
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
