# Crazyflie 2.1+ Trajectory & Swarm Control — Pure rclpy / ROS2

> **Historical notes.** This file preserves the author's working notes as
> written during the internship. The repository layout it references
> predates the `phase0/phase1/phase2` reorganisation, and some hardware
> findings here were later superseded — for the current layout see
> `README.md`; for the LPS + AI-deck stacking conditions see
> `docs/FLIGHT_RUNBOOK.md` §0.

Autonomous single-drone trajectory tracking and two-drone swarm behaviours for
Crazyflie 2.1+ nano-quadcopters, implemented in **pure `rclpy` + `crazyflie_interfaces`
— no `cflib` in any flight script**. Developed during a summer research internship at
the IRAS Hub, IIIT-Delhi (Lab B-419) under Prof. Sanjit Kaul.

- **Positioning:** Loco Positioning System (LPS), TDoA2, 8 UWB anchors
- **Stack:** ROS2 Humble, `crazyswarm2`, Gazebo Fortress (custom sim), Ubuntu 22.04
- **Backend:** `ros2 launch crazyflie launch.py backend:=cflib mocap:=False`
  (the `cflib` here is only the low-level radio backend of crazyswarm2; all control
  logic in these scripts is pure rclpy and never calls cflib directly)

---

## Repository structure

```
single_drone/   Single-drone trajectory scripts
swarm/          Two-drone swarm behaviours + leader-follower
sim/            Custom Gazebo kinematic sim servers
worlds/         Gazebo SDF world files (lab + anchors)
helpers/        Shared helpers (safe-zone / dynamic placement)
```

### single_drone/
| Script | Trajectory |
|---|---|
| `circle_path_node_v3.py` | Circle (rim-entry, `START_FROM_CURRENT` toggle) |
| `circle_path_node_v4.py` | Circle — gentle takeoff + stabilise hover |
| `circle_path_node_v5.py` | Circle — 1.2 m radius, gentle takeoff (most visible circle) |
| `square_launch_node.py` | Square |
| `hexagon_node.py` | Hexagon |
| `figure8_node_sim.py` | Figure-8 |
| `spiral_node.py` | Spiral |
| `parabola_trajectory_node.py` | Parabola |
| `simple_launch_node.py` | Basic takeoff / hover / land |

### swarm/
| Script | Behaviour |
|---|---|
| `swarm_circle_concentric.py` | Two drones, different-radius concentric circles, 180° apart (constant 2.0 m separation) |
| `swarm_circle_phase.py` | Two drones, same-radius circle, 180° out of phase |
| `swarm_leader_follower.py` | Follower trails leader (analytic-tangent trailing, sequenced approach) |
| `follow_me.py` | Drone follows a hand-held motor-off Crazyflie used as an LPS tag |
| `swarm_formation_hold.py` | Two drones hold a fixed-offset formation |
| `swarm_formation_flight.py` | Rigid formation translated along a path |
| `swarm_synchronized.py` | Both fly the same circle 180° apart at different altitudes |
| `leader_follower_optionA.py` | Leader broadcasts a pre-planned trajectory (circle/square/triangle) on a ROS2 topic; follower trails with a watchdog **failsafe** that lands the follower if the leader stream is lost |

### sim/
- `cf_sim_server.py`, `cf_sim_server_swarm.py` — custom single-persistent-worker
  kinematic sim servers (move drones via `ign set_pose`; no MulticopterVelocityControl
  physics plugin, which crashes Gazebo when duplicated).

### worlds/
- `cf_lab.sdf`, `cf_lab_swarm.sdf` — Gazebo worlds with the real B-419 anchor layout,
  four walls and floor.

### helpers/
- `dynamic_start.py` — safe-zone constants + dynamic placement helpers
  (`fit_center`, `clamp_xy`, `in_safe_zone`) built from the real anchor hull.

---

## Running

```bash
# 1. real hardware backend (single or swarm YAML in place)
ros2 launch crazyflie launch.py backend:=cflib mocap:=False

# 2. run a script (place scripts + dynamic_start.py in the same folder)
python3 single_drone/circle_path_node_v5.py
python3 swarm/swarm_circle_phase.py
python3 swarm/leader_follower_optionA.py --ros-args -p trajectory:=triangle
```

Each flight script writes a `.csv` log and trajectory `.png` plots to the home
folder. `/all/emergency` (or `/cf231/emergency`) stops the drone(s) — keep that
terminal ready.

---

## Key engineering findings

- **Noise-limited, not gain-limited.** TDoA2 position accuracy floors at ~10–15 cm;
  measured single-drone circle tracking ~24–25 cm. Beyond a point, PID tuning gives
  diminishing returns because the limit is positioning noise, not controller gain.
- **+8 cm Y-bias** in the high-Y region — a TDoA2 geometric artefact, not a fault.
- **Floor-Z artefact.** The low anchors sit at z = 0.30 m, so a drone on the ground
  is *below* the anchor plane and its Z estimate reads negative there; it becomes
  accurate once airborne (verified by hand at flight height with a tape measure).
  Because of this, floor-Z arming gates are inappropriate for LPS and were removed.
- **Leader-follower stability.** The original follower differentiated the *measured*
  (noisy) leader position, which produced random bearings and caused a wall/ceiling
  crash. Fixed by trailing the leader's *commanded* trajectory analytically
  (tangent from the known circle angle) with a per-tick step limit.
- **AI-deck investigation (deferred).** Migrating leader-follower to AI-deck Wi-Fi
  communication requires stacking the AI-deck with a positioning deck (LPS or flow),
  which needs a custom pin fabrication and hardware approval — beyond the internship
  timeline. The Option-A leader-follower (`leader_follower_optionA.py`) demonstrates
  the control + failsafe logic over a ROS2 transport as a proof of concept.

---

## Notes

- Scripts assume drone namespaces `cf231` (leader) and `cf2` (follower); adjust to
  your `crazyflies.yaml`.
- `estimator: 2` (Kalman) is required for LPS.
- The three converted formation behaviours (`hold`, `synchronized`, `formation_flight`)
  are geometry- and syntax-verified but should be flown in simulation first.
- Analysis tools (`tracking_error.py`, `error_plots.py`, `swarm_plots.py`,
  `static_check.py`) referenced by some scripts are maintained separately.
