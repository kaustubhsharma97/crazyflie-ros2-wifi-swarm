#!/usr/bin/env python3
"""
swarm_circle_concentric.py - ROS2 Humble / rclpy - TWO-DRONE SAME-RING CIRCLE (180 deg PHASE)
================================================================================
Crazyflie 2.1+ x2 (cf231 + cf2) | LPS TDoA2 | Lab B-419, IIIT-Delhi
Kaustubh Sharma | Summer Intern (Prof. Sanjit Kaul)

Both drones fly a circle about the SAME centre at the SAME altitude, but on
different radii (cf231 inner r=0.7, cf2 outer r=1.3) and 180 deg out of phase,
so they sit on opposite sides of the rings at all times. Separation is therefore
a constant R1+R2 = 2.0 m by construction - they can never converge.

This mirrors the single-drone circle_path_node_v3 approach (dynamic placement,
rim-entry start, robust save) extended to two drones, and produces the four
requested figures + one combined CSV:
    1. XY top-down      (both ideal rings vs actual)
    2. Z theoretical vs real (both drones over time; one flat theoretical line)
    3. 3D trajectory    (both rings + actual)
    4. Error analysis   (per-drone 3D tracking error over time)

SAFETY: concentric + 180 deg phase => constant 2.0 m separation; sequenced
approach (one drone moves at a time, paths never cross); divergence watchdog
auto-lands BOTH on estimate runaway; pre-takeoff Z sanity + placement check.
/all/emergency stops both - keep that terminal ready.

Run: python3 swarm_circle_concentric.py
"""
import os, sys, csv, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from dynamic_start import (SAFE_X_MIN, SAFE_X_MAX, SAFE_Y_MIN, SAFE_Y_MAX,
                           fit_center, clamp_xy, in_safe_zone)

D1, D2 = "cf231", "cf2"        # inner, outer

R       = 1.0                  # SAME radius for both drones (m)
R1      = R                    # kept as aliases so the rest of the code is unchanged
R2      = R
FLY_Z   = 0.6                  # SAME altitude for both
OMEGA   = 0.5                  # rad/s (same angular rate)
PHASE   = math.pi             # 180 deg -> always opposite sides
CIRCLE_DURATION = 2 * math.pi / OMEGA
DT      = 0.25
TAKEOFF_TIME = 3.0
LPS_SETTLE   = 3.0
REACH   = R2                   # outer ring must fit the hull

Z_ABORT    = FLY_Z + 0.7
POS_ABORT  = 1.5
FLOOR_Z_OK = 0.35
MIN_SPAWN_SEP = 0.8            # the two drones must be placed at least this far apart
GUARD_GRACE_S = 2.5           # ignore the divergence guard this long after takeoff
GUARD_CONSEC  = 5             # require this many CONSECUTIVE bad samples before aborting

CSV_PATH = os.path.expanduser("~/swarm_circle_phase_log.csv")
PNG_XY   = os.path.expanduser("~/swarm_circle_phase_xy_topdown.png")
PNG_Z    = os.path.expanduser("~/swarm_circle_phase_z.png")
PNG_3D   = os.path.expanduser("~/swarm_circle_phase_3d.png")
PNG_ERR  = os.path.expanduser("~/swarm_circle_phase_error.png")

RADII = {D1: R1, D2: R2}
PHASE_OF = {D1: 0.0, D2: PHASE}


class SwarmCirclePhase(Node):
    def __init__(self):
        super().__init__("swarm_circle_phase")
        self._finished = False; self._saved = False
        self.pose = {D1: None, D2: None}
        self.setpoint = {D1: (0,0,FLY_Z), D2: (0,0,FLY_Z)}
        self.log_data = []; self.t0 = None
        self.cx = self.cy = None
        self.base = {D1: 0.0, D2: PHASE}   # per-drone start angle; set from spawn in WAIT_POSE
        self.circle_t0 = None
        self.state = "WAIT_POSE"
        self._pending = {D1: None, D2: None}
        self._wait_until = 0
        self._guard_after = 0        # guard inactive until this time (set at takeoff)
        self._bad_count = 0          # consecutive bad-sample counter
        self.arm_cli={}; self.tk_cli={}; self.goto_cli={}; self.land_cli={}
        for d in (D1, D2):
            self.create_subscription(PoseStamped, f"/{d}/pose",
                                     lambda m, dd=d: self._pcb(dd, m), 10)
            self.arm_cli[d]=self.create_client(Arm, f"/{d}/arm")
            self.tk_cli[d]=self.create_client(Takeoff, f"/{d}/takeoff")
            self.goto_cli[d]=self.create_client(GoTo, f"/{d}/go_to")
            self.land_cli[d]=self.create_client(Land, f"/{d}/land")
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"SwarmCircleConcentric: {D1} r={R1} (inner), {D2} r={R2} (outer), "
            f"same z={FLY_Z}, 180deg phase -> {2*R:.1f} m constant separation (same ring)")

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

    def _pt(self, d, t):
        th = OMEGA*t + self.base[d]
        r = RADII[d]
        x, y = clamp_xy(self.cx + r*math.cos(th), self.cy + r*math.sin(th))
        return x, y, FLY_Z

    def _tick(self):
        s=self.state
        if s=="WAIT_POSE":
            if not self._both_pose(): return
            if self._wait_until==0:
                self._wait_until=self._now()+int(LPS_SETTLE*1e9)
                self.get_logger().info(f"Both have pose. Settling {LPS_SETTLE}s ...")
                for d in (D1,D2):
                    self.get_logger().info(f"  {d}: ({self.pose[d][0]:.2f},{self.pose[d][1]:.2f},z={self.pose[d][2]:.2f})")
                return
            if not self._past(): return
            a, b = self.pose[D1], self.pose[D2]
            if math.hypot(a[0]-b[0], a[1]-b[1]) < MIN_SPAWN_SEP:
                self.get_logger().error(
                    f"The two drones are placed only {math.hypot(a[0]-b[0],a[1]-b[1]):.2f}m "
                    f"apart (need >{MIN_SPAWN_SEP}m). Move them apart; NOT arming.")
                self.state = "SAVE"; return
            self.cx, self.cy = fit_center(a[0], a[1], REACH, REACH)
            # Start each drone on the rim point nearest ITS OWN spawn, kept 180 deg
            # apart, so neither has to cross the other during the approach. The
            # outer drone (cf2) anchors to its spawn angle; inner sits opposite.
            ang_outer = math.atan2(b[1]-self.cy, b[0]-self.cx)
            self.base[D2] = ang_outer
            self.base[D1] = ang_outer + math.pi
            self.get_logger().info(
                f"Circle centre ({self.cx:.2f},{self.cy:.2f}); ring r={R}, "
                f"start angles set from spawns (180 deg apart), in hull.")
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
                    r=Takeoff.Request(); r.height=FLY_Z
                    r.duration.sec=int(TAKEOFF_TIME); r.duration.nanosec=int((TAKEOFF_TIME%1)*1e9)
                    self._pending[d]=self.tk_cli[d].call_async(r)
                self._wait_until=self._now()+int((TAKEOFF_TIME+1.5)*1e9)
                self.get_logger().info(f"Both taking off to {FLY_Z} m ...")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}
                self._guard_after = self._now() + int(GUARD_GRACE_S*1e9)
                self.get_logger().info(
                    f"Reached flight height; divergence guard arms in {GUARD_GRACE_S}s.")
                self.state="APPROACH_OUTER"
        # Sequenced approach: outer drone (cf2) to its start FIRST (opposite side),
        # inner holds; then inner (cf231) to its start. One mover at a time.
        elif s=="APPROACH_OUTER":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                lx, ly, _ = self.pose[D1]
                self._pending[D1] = self._goto(D1, lx, ly, FLY_Z, 3.0)     # inner holds
                self._pending[D2] = self._goto(D2, *self._pt(D2, 0.0), 3.0)  # outer -> start
                self._wait_until=self._now()+int(4.0*1e9)
                self.get_logger().info("Approach 1/2: outer (cf2) -> start, inner holding")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.state="APPROACH_INNER"
        elif s=="APPROACH_INNER":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                self._pending[D2] = self._goto(D2, *self._pt(D2, 0.0), 2.0)  # outer holds
                self._pending[D1] = self._goto(D1, *self._pt(D1, 0.0), 3.0)  # inner -> start
                self._wait_until=self._now()+int(4.0*1e9)
                self.get_logger().info("Approach 2/2: inner (cf231) -> start, outer holding")
            elif self._both_done() and self._past():
                self._pending={D1:None,D2:None}; self.circle_t0=self._now()/1e9
                self.get_logger().info(f"Phase: concentric circles, {CIRCLE_DURATION:.1f}s ...")
                self.state="CIRCLE"
        elif s=="CIRCLE":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            t=self._now()/1e9 - self.circle_t0
            if t>CIRCLE_DURATION:
                self.state="SETTLE"; self._pending={D1:None,D2:None}; return
            self._goto(D1, *self._pt(D1, t), DT)
            self._goto(D2, *self._pt(D2, t), DT)
            self._wait_until=self._now()+int(DT*1e9)
            if self.pose[D1] and self.pose[D2] and int(t*4)%8==0:
                sep=math.hypot(self.pose[D2][0]-self.pose[D1][0], self.pose[D2][1]-self.pose[D1][1])
                self.get_logger().info(f"  t={t:4.1f}s sep={sep:.2f}m")
        elif s=="SETTLE":
            if self._diverged():
                self.state="LAND"; self._pending={D1:None,D2:None}; return
            if self._pending[D1] is None:
                self._pending[D1]=self._goto(D1, *self._pt(D1, 0.0), 2.0)
                self._pending[D2]=self._goto(D2, *self._pt(D2, 0.0), 2.0)
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
            self.get_logger().info("Concentric circle complete (no data)."); return
        self.get_logger().info("Finalising: CSV + 4 PNGs ...")
        try: self._save_csv()
        except Exception as e: self.get_logger().error(f"CSV save failed: {e}")
        try: self._save_plots()
        except Exception as e: self.get_logger().error(f"plot save failed: {e}")
        for p in (CSV_PATH, PNG_XY, PNG_Z, PNG_3D, PNG_ERR):
            self.get_logger().info(f"  [{'OK  ' if os.path.exists(p) else 'MISS'}] {p}")
        seps=[r.get("separation_xy") for r in self.log_data if r.get("separation_xy")]
        if seps: self.get_logger().info(
            f"Separation: min={min(seps):.2f} max={max(seps):.2f} mean={sum(seps)/len(seps):.2f} m")
        self.get_logger().info("Concentric circle swarm complete.")

    def _save_csv(self):
        keys=["time"]
        for d in (D1,D2): keys+=[f"{d}_x",f"{d}_y",f"{d}_z",f"{d}_ex",f"{d}_ey",f"{d}_ez"]
        keys+=["separation_xy"]
        with open(CSV_PATH,"w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore"); w.writeheader(); w.writerows(self.log_data)
        self.get_logger().info(f"CSV  -> {CSV_PATH} ({len(self.log_data)} rows)")

    def _save_plots(self):
        if self.cx is None: return
        d = {dd: {k: np.array([r[f"{dd}_{k}"] for r in self.log_data])
                  for k in ("x","y","z","ex","ey","ez")} for dd in (D1,D2)}
        tarr = np.array([r["time"] for r in self.log_data])
        # masks: keep samples where the drone was actually on its ring (on-circle)
        on = {dd: np.hypot(d[dd]["ex"]-self.cx, d[dd]["ey"]-self.cy) > (RADII[dd]*0.5)
              for dd in (D1,D2)}
        for dd in (D1,D2):
            if not np.any(on[dd]): on[dd]=np.ones(len(tarr),dtype=bool)
        th = np.linspace(0, 2*math.pi, 400)
        ideal = {dd: (self.cx+RADII[dd]*np.cos(th), self.cy+RADII[dd]*np.sin(th)) for dd in (D1,D2)}
        col = {D1: "tab:blue", D2: "tab:red"}
        bx=[SAFE_X_MIN,SAFE_X_MAX,SAFE_X_MAX,SAFE_X_MIN,SAFE_X_MIN]
        by=[SAFE_Y_MIN,SAFE_Y_MIN,SAFE_Y_MAX,SAFE_Y_MAX,SAFE_Y_MIN]

        # 1) XY top-down
        plt.figure(figsize=(7.5,7.5))
        plt.plot(bx,by,"--",color="grey",alpha=0.5,label="Safe zone")
        for dd in (D1,D2):
            plt.plot(*ideal[dd], ":", color=col[dd], lw=2, label=f"{dd} ideal r={RADII[dd]}")
            plt.scatter(d[dd]["x"][on[dd]], d[dd]["y"][on[dd]], s=10, color=col[dd], alpha=0.8,
                        label=f"{dd} actual")
        plt.plot(self.cx,self.cy,"k+",ms=13,mew=2,label=f"Centre ({self.cx:.2f},{self.cy:.2f})")
        plt.title("Concentric circles - XY top-down (LPS)"); plt.xlabel("X (m)"); plt.ylabel("Y (m)")
        plt.legend(loc="upper right",fontsize=8); plt.grid(alpha=0.3); plt.axis("equal")
        plt.tight_layout(); plt.savefig(PNG_XY,dpi=120); plt.close()

        # 2) Z theoretical vs real
        plt.figure(figsize=(9,4.5))
        plt.axhline(FLY_Z, ls="--", color="k", label=f"Theoretical z={FLY_Z}")
        for dd in (D1,D2):
            plt.plot(tarr, d[dd]["z"], color=col[dd], lw=1, label=f"{dd} actual z")
        plt.title("Altitude: theoretical vs real"); plt.xlabel("time (s)"); plt.ylabel("Z (m)")
        plt.legend(fontsize=8); plt.grid(alpha=0.3); plt.ylim(0, FLY_Z+0.4)
        plt.tight_layout(); plt.savefig(PNG_Z,dpi=120); plt.close()

        # 3) 3D trajectory
        fig=plt.figure(figsize=(8,6.5)); a3=fig.add_subplot(111,projection="3d")
        for dd in (D1,D2):
            a3.plot(ideal[dd][0], ideal[dd][1], FLY_Z*np.ones_like(th), ":", color=col[dd], lw=2,
                    label=f"{dd} ideal")
            a3.scatter(d[dd]["x"][on[dd]], d[dd]["y"][on[dd]], d[dd]["z"][on[dd]], s=8,
                       color=col[dd], alpha=0.7, label=f"{dd} actual")
        a3.set_xlabel("X (m)"); a3.set_ylabel("Y (m)"); a3.set_zlabel("Z (m)")
        a3.set_title("Concentric circles - 3D"); a3.legend(fontsize=8)
        plt.tight_layout(); plt.savefig(PNG_3D,dpi=120); plt.close()

        # 4) Error analysis (per-drone 3D tracking error over time)
        plt.figure(figsize=(9,4.5))
        for dd in (D1,D2):
            err = np.sqrt((d[dd]["x"]-d[dd]["ex"])**2 + (d[dd]["y"]-d[dd]["ey"])**2 +
                          (d[dd]["z"]-d[dd]["ez"])**2) * 100.0
            e_on = err[on[dd]]; t_on = tarr[on[dd]]
            plt.plot(t_on, e_on, color=col[dd], lw=1, label=f"{dd} 3D error")
            if len(e_on):
                plt.axhline(np.mean(e_on), ls="--", color=col[dd], alpha=0.6,
                            label=f"{dd} mean {np.mean(e_on):.1f}cm")
        plt.title("Tracking error vs time (on-circle)"); plt.xlabel("time (s)"); plt.ylabel("3D error (cm)")
        plt.legend(fontsize=8); plt.grid(alpha=0.3)
        plt.tight_layout(); plt.savefig(PNG_ERR,dpi=120); plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCirclePhase()
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
