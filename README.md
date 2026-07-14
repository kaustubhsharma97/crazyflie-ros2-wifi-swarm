# Crazyflie 2.1+ Multi-Drone Control — Pure rclpy over LPS and AI-Deck Wi-Fi

**Kaustubh Sharma** — Summer Research Intern, IRAS Hub (Lab B-419), IIIT-Delhi
Supervisor: **Prof. Sanjit Kaul** | May–July 2026
B.Tech ECE (Advanced Communication Technology), JIIT Noida

## About this work

I am **Kaustubh Sharma**, a B.Tech student in Electronics & Communication
Engineering (Advanced Communication Technology) at JIIT Noida. This
repository is the complete record of my summer research internship at the
**IRAS Hub, IIIT-Delhi (Lab B-419)**, May–July 2026, under the supervision
of **Prof. Sanjit Kaul**.

Over the internship I took a pair of Crazyflie 2.1+ nano-quadrotors from a
single scripted takeoff to a full multi-drone system: seven autonomous
trajectories flown on real hardware, a PID tuning study that reduced circle
tracking error from 35 cm to 24 cm and identified the positioning-noise
floor as the limiting factor, four two-drone swarm behaviors, a hand-guided
follow-me flight, and finally a complete migration of the control link from
the Crazyradio dongle to the AI-deck's Wi-Fi — putting both drones under
full ROS2 control over the lab network with no dongle in the loop, and
building a leader–follower system with a communication-loss failsafe on
top of it. A deliberate constraint ran through all of it: **every line of
flight code is pure rclpy + crazyflie_interfaces — no cflib** —
distinguishing this stack from parallel cflib-based work in the lab and
proving the ROS2-native path end to end. (Crazyswarm2's `backend:=cflib`
launch option refers only to the server's low-level link layer; no flight
script in this repository calls cflib.)

Multi-drone control of Crazyflie 2.1+ quadrotors implemented **entirely in
rclpy (ROS2 Humble) + crazyflie_interfaces** on top of Crazyswarm2 — no cflib
in any flight code. The work spans three phases:

* **Phase 0 — Single drone (LPS / Crazyradio):** seven autonomous
  trajectories (simple launch, circle, square, parabola, figure-8, hexagon,
  spiral), each flown in simulation and on real hardware, plus the circle
  PID tuning study (`circle_path_node.py` v1→v5 preserves the full tuning
  lineage: 35 cm → 24 cm mean error).
* **Phase 1 — Two-drone swarm (LPS / Crazyradio):** single- and two-drone autonomous flight
  using an 8-anchor UWB Loco Positioning System (TDoA2) with one shared
  Crazyradio dongle.
* **Phase 2 — AI-deck / Wi-Fi:** replacing the Crazyradio with the AI-deck's
  ESP32 Wi-Fi link, achieving **two drones under full ROS2 control over the
  lab Wi-Fi with no dongle anywhere in the loop**, and building a
  leader–follower system with a communication-loss failsafe.

---

## Headline results

| Result | Detail |
|---|---|
| 7 trajectories flown on real hardware | simple, circle, square, parabola, figure-8, hexagon, spiral (sim + real) |
| Circle tracking error 35 cm → **24 cm** | Position-P = 3.0, rim-entry start, R = 0.8 m, ω = 0.5 rad/s; system shown to be **positioning-noise-limited** (TDoA2 floor ≈ 10–15 cm), not gain-limited |
| 5 two-drone swarm behaviors | formation hold, leader–follower, synchronized 180° phase, formation flight, concentric circles |
| Follow-me | drone follows a hand-carried Crazyflie used as a live LPS tag |
| **Wi-Fi-only control (no Crazyradio)** | both drones connected via `tcp://<deck-ip>:5000` through Crazyswarm2's cflib backend; pose telemetry verified stable at 10 Hz |
| Leader–follower over Wi-Fi | fixed-offset trailing, 3 selectable trajectories, watchdog failsafe (follower lands itself on leader loss), room safe-zone clamping |
| Troubleshooting knowledge base | 10+ root-caused failures documented in [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |

---

## Architecture (Phase 2)

```
                    ┌────────────────── LAPTOP (ROS2 Humble) ─────────────────┐
   trajectory ───▶  │  leader node ──▶ /cf231/cmd_position                    │
   (leader only)    │  leader position ──▶ /leader/setpoint or /cf231/pose    │
                    │  follower node ◀── reads leader position, computes      │
                    │                    chase target ──▶ /cf2/cmd_position   │
                    │  crazyswarm2 server (backend:=cflib)                    │
                    └───────┬─────────────────────────────────┬───────────────┘
                       Wi-Fi (tcp://ip:5000)             Wi-Fi (tcp://ip:5000)
                            ▼                                 ▼
                    LEADER drone                       FOLLOWER drone
                    AI-deck (comms)                    AI-deck (comms)
                    LPS deck (position estimate only)  LPS deck (position estimate only)
                            ▲            UWB           ▲
                            └──── 8 wall anchors ──────┘
```

Design rules established with Prof. Kaul:

1. The trajectory is sent **only to the leader**; the follower has zero
   trajectory knowledge (grep the follower code — no shape appears in it).
2. The follower learns the leader's position **exclusively over Wi-Fi**
   (`follow_source:=pose` follows the leader's *measured* LPS position).
3. The LPS system is used **only** for each drone's own position estimate;
   it carries no inter-drone information.
4. If the leader's stream goes stale (default 0.8 s), the follower
   **lands itself in place** (Prof. Kaul's failsafe requirement).

The one caveat that shaped the project: a follower with only an AI-deck
cannot fly position control — the AI-deck provides communication, not
positioning. `docs/phantom_drift.png` shows a motionless, deck-less drone
believing it moved 11.3 m in 1.9 s (≈ 6 m/s phantom velocity). Every
positioning-free flight attempt confirmed this (3 controlled crashes).
Bitcraze's own AI-deck Wi-Fi flight example likewise requires a positioning
deck.

## Repository layout

```
phase0_single_drone/    7 trajectories + circle tuning lineage (v1..v5) +
                        dynamic_start.py (shared safe-zone/start module)
phase1_lps_swarm/       LPS-era two-drone swarm scripts (flown on hardware),
                        follow_me.py (drone follows a hand-carried LPS tag —
                        the internship's favorite demo) + swarm_plots.py
phase2_aideck_wifi/     Wi-Fi-era scripts:
  leader_follower.py      ★ the mission — 3 trajectories, failsafe, safe-zone
  leader_follower_*.py    per-shape single-file variants (early versions)
  leader_node.py /        original two-process split (supports two-machine
  follower_node.py        operation via --role)
  hover_test.py           first-flight shakedown + hover-stability metrics
  swarm_circle_aideck.py  LPS swarm circle converted to per-drone frames
config/                 crazyswarm2 YAMLs for every era (LPS single/swarm,
                        mixed radio+tcp, dual-Wi-Fi, AI-only) + anchor positions
sim/                    custom Gazebo Fortress simulation: kinematic sim
                        servers (single + swarm), lab worlds, drone models
tools/                  find_decks.sh (AI-deck discovery + port probe),
                        use_single_drone.sh / use_swarm.sh (yaml switchers)
docs/                   flight-day floor plans (2D/3D), phantom-drift figure,
                        flight runbook, PROJECT_NOTES.md (author's own notes)
```

## Quickstart (Phase 2, dual-stack configuration)

```bash
# 0. pre-flight: locate decks, verify port 5000 open on both
./tools/find_decks.sh

# 1. server — cflib backend is REQUIRED for tcp:// links
cp config/crazyflies_dual_wifi.yaml <crazyswarm2>/crazyflie/config/crazyflies.yaml
ros2 launch crazyflie launch.py backend:=cflib mocap:=False

# 2. keep the kill switch pre-typed in its own terminal
ros2 service call /all/emergency std_srvs/srv/Empty

# 3. shakedown, then the mission
python3 phase2_aideck_wifi/hover_test.py --ros-args -p drones:=cf2 -p height:=0.6
python3 phase2_aideck_wifi/leader_follower.py --ros-args \
    -p trajectory:=circle -p follow_source:=pose -p offset_x:=-0.8

# 4. failsafe demo: leader mutes its broadcast mid-flight and lands itself;
#    the follower's watchdog must land it independently
python3 phase2_aideck_wifi/leader_follower.py --ros-args \
    -p trajectory:=circle -p follow_source:=pose -p offset_x:=-0.8 \
    -p simulate_leader_loss_after:=15.0
```

Full procedure, gates, and tape-mark placement: `docs/FLIGHT_RUNBOOK.md`.

## Status at handoff (July 2026)

Working and verified on hardware: everything through two-drone Wi-Fi control
(pose at 10 Hz on both drones, commands over TCP, motors responding).
Flight of the final dual-stack configuration is pending one hardware item:
**LPS + AI-deck stacking requires custom long pin headers** (the LPS deck did
not enumerate when stacked with the available headers). Fabrication in the
ECE lab was scoped but deferred for time/risk. The moment stacking works
(or Flow decks are approved), the quickstart above flies unchanged.

## Future work

1. Fabricate/procure stacking headers → fly the dual-Wi-Fi configuration.
2. Option B: true deck-to-deck comms (custom ESP32 firmware; the stock
   aideck-esp-firmware only runs a TCP *server* — it never initiates
   peer connections).
3. Follow-me over Wi-Fi (port of the Phase-1 favorite).
4. Static-grid characterization of the known +8 cm Y-bias in the high-Y
   room region (TDoA2 geometry).
5. Re-add remaining analysis-suite sources (`tracking_error.py`,
   `error_plots.py`, `analyze_trajectory.py`, `static_check.py`) — only
   compiled caches of these survived the home-directory archive.

## Acknowledgments

My deepest thanks to **Prof. Sanjit Kaul**, whose supervision shaped this
internship far beyond a task list. He set problems the way good research
advisors do — precise objectives with the *how* left open — and each pivot
he introduced (the PID tuning study, the failsafe requirement, the move
from radio to Wi-Fi, the strict separation of positioning from
communication) turned out to be exactly the push that deepened the work.
His insistence on honest, measured results over comfortable claims taught
me more about engineering discipline than any course has, and the standard
he holds his lab to is one I will carry with me. It was a privilege to
spend a summer building under his guidance at the IRAS Hub.

Thanks also to my PhD mentor in the lab for hardware judgment at exactly
the right moments, to labmates **Raghav, Dewang and Anika** (whose parallel
cflib implementations at
[Raghs-7/crazy-flies](https://github.com/Raghs-7/crazy-flies) were an
invaluable reference and sanity check), and to the Bitcraze documentation
and community, without which the AI-deck troubleshooting chapters of this
repo would have taken far longer.

— Kaustubh Sharma, IIIT-Delhi, July 2026
