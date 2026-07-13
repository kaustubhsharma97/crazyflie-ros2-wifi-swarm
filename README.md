# Crazyflie 2.1+ UAV Drone ROS 2 based Multi-Agent Swarm & AI-Deck Wi-Fi Architecture

A complete ROS 2 Humble (rclpy) multi-agent drone swarm framework for Crazyflie 2.1+. Features non-blocking state machines, dynamic PID tuning, custom Gazebo simulation parity, and a novel transition from TDoA2 LPS tracking to decentralized leader-follower failsafe protocols via direct AI-deck (ESP32) TCP/Wi-Fi socket communication.

Author: Kaustubh Sharma

Organization: Indraprastha Institute of Information Technology Delhi (IIIT-Delhi)

Lab: Networking and Robotics Lab (Under Prof. Sanjit Kaul)

Date: Summer 2026

🚀 Key Architectural Phases

Phase 0: Single-Drone LPS Optimization (/phase0_single_drone)

Engineered 7 non-blocking state-machine trajectory algorithms (Launch, Circle, Square, Parabola, Figure-8, Hexagon, Spiral).

Suppressed spatial tracking error to a ~24-25cm hardware noise floor via dynamic coordinate transformations and PID tuning.

Automated .csv telemetry data pipelines and .png 3D spatial error visualization generation.

Phase 1: Multi-Agent LPS Swarm (/phase1_lps_swarm)

Synchronized multi-drone flight across 4 distinct behaviors (Concentric, Phase-Shifted, Synchronized, Follow-Me).

Engineered a camera-less "Follow-me" tracking protocol using an active LPS tag over a shared radio dongle (Channel 80).

Implemented guard grace-period algorithms to bypass initial Extended Kalman Filter (EKF) UWB settling spikes.

Phase 2: AI-Deck Wi-Fi Transition (/phase2_aideck_wifi)

Transitioned from anchor-dependent TDoA2 tracking to a decentralized direct deck-to-deck Wi-Fi communication architecture.

Flashed custom ESP32/GAP8 firmware via CPX routing to enable low-latency 10Hz pose telemetry streaming over raw TCP sockets.

Engineered a strict 0.5s packet-loss watchdog timer that triggers an automated safe-landing sequence upon stream interruption.

Simulation Parity (/sim)

Resolved native multicopter Gazebo physics plugin crashes by developing a persistent-worker multi-agent simulation environment (cf_sim_server_swarm.py).

Engineered visual-only inline .sdf drone models to ensure perfect telemetry parity between the virtual world and the physical UWB anchor network.

📂 Repository Structure

crazyflie-ros2-wifi-swarm/
├── config/                  # Server YAML configs (Single, Swarm, AI-Deck, Mixed)
├── docs/                    # Architectural notes, floor plans, and runbooks
├── phase0_single_drone/     # Core single-agent non-blocking trajectory scripts
├── phase1_lps_swarm/        # UWB-based multi-agent behaviors & Follow-me
├── phase2_aideck_wifi/      # AI-deck TCP/Wi-Fi leader-follower protocols
├── sim/                     # Custom Gazebo models & multi-agent sim servers
├── tools/                   # Bash utilities (find_decks.sh, server toggles)
└── publish.sh               # Execution pipeline script


⚙️ Hardware & Software Stack

Hardware: Crazyflie 2.1+, Loco Positioning System (TDoA2 UWB), AI-deck (ESP32/GAP8), Crazyradio PA.

Software: Ubuntu 22.04, ROS 2 (Humble), Crazyswarm2, Python 3, Gazebo.

📊 Telemetry & Error Analysis

This framework features built-in diagnostics. Scripts automatically dump runtime telemetry to CSV and generate 4-panel Matplotlib analysis PNGs (XY Top-down, 3D trajectory, Z-variance, and 3D Euclidean error) upon safe landing or keyboard interrupt.

📝 License

This project is licensed under the MIT License - see the LICENSE file for details.
