# Flight Runbook — dual-stack Wi-Fi leader-follower (for whoever flies this next)

Prereq: both drones with LPS (top) + AI-deck (bottom, long pins) mounted,
8 anchors powered, batteries > 4.1 V, tape marks per docs/flight_day_floorplan.png.

## Gates (do not skip — the flight scripts contain no readiness checks)
1. cfclient console per drone: `2 deck(s) found` (bcDWM1000 + bcAI),
   `Kalman (2)`, TDoA2 detected, `got ip:` on the lab SSID. Disconnect after.
2. `tools/find_decks.sh` → both port 5000 OPEN, IPs match config/crazyflies_dual_wifi.yaml.
3. Server: `ros2 launch crazyflie launch.py backend:=cflib mocap:=False`
   → both `is connected!`, no teardown.
4. `ros2 topic echo /cf231/pose --once` (and cf2): sane room coordinates
   (X 0–5, Y 0–8, Z near 0 or slightly negative). Garbage = stop.
5. 60 s static check per drone on a known mark (scatter ≈ 10–15 cm is normal).

## Ladder (kill switch pre-typed in its own terminal:
`ros2 service call /all/emergency std_srvs/srv/Empty`)
1. `python3 hover_test.py --ros-args -p drones:=cf2 -p height:=0.6`
2. same with `-p drones:=cf231`
3. Mission: `python3 leader_follower.py --ros-args -p trajectory:=circle -p follow_source:=pose -p offset_x:=-0.8 -p laps:=1`
4. Failsafe demo: add `-p simulate_leader_loss_after:=15.0`
5. `-p trajectory:=square` / `triangle`; then swarm_circle_aideck.py if desired.

Placement is dynamic anywhere interior to the anchors: circle extends 1.6 m
in −X from the leader (leader X ≥ 3.0); square/triangle extend +X/+Y
(leader ≈ (2.0, 2.8)); follower ~0.8 m behind in −X (step-limiter corrects
imperfect placement). Commands are clamped to X [0.30, 4.74], Y [−0.30, 6.90],
Z ≤ 2.20. Power-cycle both drones before every server launch (stale-socket rule).
