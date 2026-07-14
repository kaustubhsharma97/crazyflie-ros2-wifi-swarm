# Getting Started — from a blank laptop to flying this repository

This guide is deliberately self-contained: every command and every core
concept a new student needs to continue this project, in order. Follow it
top to bottom. Written July 2026 for Ubuntu 22.04 + ROS 2 Humble (the exact
stack everything here was built and flown on — do not "upgrade" versions).

**What you need:** a laptop with ≥ 8 GB RAM and ≥ 60 GB free disk, the lab's
Crazyflie 2.1+ drones, LPS anchors, AI-decks, a Crazyradio PA, and lab
Wi-Fi credentials.

---

## 1. Install Ubuntu 22.04.5 LTS(Jammy Jellyfish) (dual boot with Windows)

1. On Windows, download the **Ubuntu 22.04.x Desktop ISO** from ubuntu.com
   and **balenaEtcher**. Flash the ISO to an 8 GB+ USB stick with Etcher.
2. Free up disk space: Windows key → "Create and format hard disk
   partitions" → right-click your biggest partition → **Shrink Volume** →
   shrink by at least **60000 MB**. Leave the freed space *unallocated*.
3. Disable Windows **Fast Startup**: Control Panel → Power Options →
   "Choose what the power buttons do" → uncheck Fast Startup. (It locks the
   disk and corrupts dual boots.)
4. Reboot into BIOS (usually F2/F12/Del at power-on). Disable **Secure
   Boot** if Ubuntu refuses to boot later; set the USB stick first in boot
   order. Save, reboot.
5. Ubuntu installer: language → keyboard → **Normal installation** + check
   "Install third-party software" → at "Installation type" choose
   **"Install Ubuntu alongside Windows Boot Manager"** (it uses the
   unallocated space automatically). If that option is missing, choose
   "Something else" and create on the free space: an `ext4` partition
   mounted at `/` (~55 GB) and leave the rest — but "alongside" is the safe
   path.
6. Reboot. The purple **GRUB** menu now offers Ubuntu or Windows at every
   power-on. Pick Ubuntu.

   https://releases.ubuntu.com/jammy/

## 2. First-boot essentials

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git build-essential python3-pip curl wget net-tools \
                    ncurses-dev unzip arp-scan
```

## 3. Install ROS 2 Humble

Run these exactly, in order (each block is copy-pasteable):

```bash
# locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# enable required repositories and add the ROS 2 apt source
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS 2 Humble desktop + build tools
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions

# source ROS in every terminal, forever
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify: `ros2 --help` prints usage; in two terminals,
`ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener`
should chat with each other.

## 4. ROS 2 in one page — the concepts this repo is built on

* **Node** — one running program (each script here creates exactly one:
  `Node('triangle_node')`).
* **Topic** — a named data stream. Publishers write, subscribers read,
  many-to-many, asynchronous. Every drone here exposes `/cfX/pose`
  (its position estimate, in) and `/cfX/cmd_position` (position commands,
  out). `ros2 topic list`, `ros2 topic echo /cf2/pose`,
  `ros2 topic hz /cf2/pose` are your three most-used commands.
* **Service** — a request/response call. Used here for `/cfX/arm`,
  `/cfX/takeoff`, `/cfX/land`, `/cfX/go_to`, and the emergency stop
  (`ros2 service call /all/emergency std_srvs/srv/Empty`).
* **Parameter** — a per-node setting, overridable at launch:
  `--ros-args -p side:=1.2`. Every script here declares its knobs this way.
* **Launch file** — starts several nodes at once
  (`ros2 launch crazyflie launch.py ...` starts the server, teleop, GUI).

**The pattern every flight script here uses — the non-blocking state
machine.** Never `sleep()` in ROS code: it freezes the node so callbacks
(incoming poses!) stop arriving. Instead, a timer ticks a function at
20 Hz; the function checks `self.state`, does one small step, and returns:

```python
self.timer = self.create_timer(1.0 / 20.0, self.tick)   # 20 Hz heartbeat

def tick(self):
    if self.state == 'TAKEOFF':
        f = min(1.0, (now - self.phase_t0) / self.takeoff_time)
        self.cmd[2] = self.home[2] + (self.height - self.home[2]) * f
        self.push()                      # publish one cmd_position
        if f >= 1.0:
            self.set_state('HOVER', now) # transition, next tick continues
```

Time-based interpolation (`f` from 0→1) gives gentle ramps; state
transitions replace blocking waits. Read `phase0_single_drone/
simple_launch_node.py` first — it is the smallest complete example.

**Golden rules learned the hard way in this lab:**
1. Never call `rclpy.shutdown()` inside a callback while `spin()` runs —
   it deadlocks the node (this single bug once froze all 16 scripts here).
2. Timers, not sleeps (above).
3. Service calls from inside a timer must be `call_async()` with the future
   checked on later ticks — a blocking call inside a callback deadlocks.
4. One terminal = one job: server, script, kill switch each get their own.

## 5. Workspace + Crazyswarm2 (the drone server)

Crazyswarm2 is the bridge between ROS 2 and the drones: it owns the radio/
Wi-Fi links and exposes each drone's topics and services. Our scripts never
talk to hardware directly — only to this server.

```bash
mkdir -p ~/crazyswarm2_ws/src && cd ~/crazyswarm2_ws/src
git clone --recursive https://github.com/IMRCLab/crazyswarm2.git
cd ~/crazyswarm2_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
echo "source ~/crazyswarm2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The server reads `crazyflies.yaml` from the crazyflie package's `config/`
directory — the `config/` folder of THIS repo contains a ready yaml for
every hardware era (LPS single, LPS swarm, Wi-Fi, mixed). Copy the one you
need over the server's `crazyflies.yaml`, then:

```bash
ros2 launch crazyflie launch.py backend:=cflib mocap:=False
```

`backend:=cflib` is required for Wi-Fi (`tcp://`) links; the default C++
backend is radio-only. `mocap:=False` because this lab uses LPS, not
motion capture (but the yaml must still contain the full `motion_capture`
block — see TROUBLESHOOTING.md §6).

## 6. Crazyflie client tools

```bash
pip3 install cfclient
# USB permissions for the Crazyradio (once):
sudo groupadd plugdev 2>/dev/null; sudo usermod -a -G plugdev $USER
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-bitcraze.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/99-bitcraze.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# log out and back in for the group change
```

`cfclient` (GUI) is your microscope: connect to a drone and read the
**Console tab** — every hardware diagnosis in TROUBLESHOOTING.md started
there. `cfloader` (installed with it) flashes firmware over the radio.

## 7. Firmware toolchain (only when re-flashing)

```bash
sudo apt install -y gcc-arm-none-eabi
git clone https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
make cf2_defconfig
make menuconfig     # Expansion deck configuration -> Support the AI-deck ->
                    # WiFi setup at startup -> Connect to a Wifi network ->
                    # enter the lab SSID + password
make -j$(nproc)
cfloader flash build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7XX
```

**Order is sacred:** `cf2_defconfig` FIRST, `menuconfig` SECOND — defconfig
run after menuconfig silently wipes the Wi-Fi credentials (see
TROUBLESHOOTING.md §1). For GAP8/ESP32 (AI-deck) flashing, install Docker
(`sudo apt install docker.io`) and follow TROUBLESHOOTING.md §2–3 — those
sections encode a week of hard-won deck forensics.

## 8. The hardware, in five paragraphs

**The Crazyflie 2.1+** is a 27 g quadrotor. Its STM32 runs the autopilot
(estimator + controllers), its nRF51 handles the radio. It flies commands
relative to a **state estimate** — its onboard belief of where it is.

**Positioning is not optional.** The single most important lesson in this
repo: a drone that cannot measure its position cannot hold or track one.
`docs/phantom_drift.png` shows a *motionless* deck-less drone believing it
moves at ~6 m/s — position commands against that flip the airframe at
takeoff (measured three times). Every flying configuration pairs the comms
link with a positioning source.

**LPS (Loco Positioning System)** = 8 ultra-wideband wall anchors + an LPS
deck on the drone. In TDoA2 mode the anchors chirp on a schedule and the
deck computes position from time-differences — like indoor GPS, ~10–15 cm
noise. Anchor positions live in `config/anchors_updated_positions.yaml`.
Quirk: on the floor the drone sits *below* the 0.30 m low-anchor plane, so
ground Z reads slightly negative — geometry, not a fault (TROUBLESHOOTING §8).

**The AI-deck** adds an ESP32 (Wi-Fi) and a GAP8 (AI co-processor). The
three chips talk over **CPX** (Crazyflie Packet eXchange). Once the deck
joins the lab Wi-Fi as a station, the ESP32 serves CPX over TCP on port
5000 — and Crazyswarm2 (cflib backend) can use `tcp://<ip>:5000` as the
drone's link instead of the radio. That is the entire Phase-2 architecture.
The deck serves **one TCP client at a time**: power-cycle drones before
every server launch. `tools/find_decks.sh` finds each deck's current IP by
MAC (no router admin needed).

**The estimator:** with a positioning deck present the firmware runs its
EKF, fusing IMU + position measurements into the state estimate that
`/cfX/pose` publishes and the position controller consumes. The PID gains
tuned in Phase 0 (`circle_path_node_v5.py`) took circle tracking to the
~24 cm noise floor of the positioning itself.

## 9. Simulation (fly with zero hardware)

```bash
sudo apt install -y ignition-fortress ros-humble-ros-ign-bridge
cd sim && ./setup_gazebo_sim.sh    # installs models, prints run instructions
```

The sim server (`cf_sim_server_swarm.py`) moves visual-only drone models
kinematically and publishes the same topics as real hardware — every
flight script in this repo runs unmodified against it. Develop here first;
touch hardware second.

## 10. Your learning path through this repository

* **Week 1:** Sections 1–5 above; run the sim; read and run
  `phase0_single_drone/simple_launch_node.py`, then `triangle` and
  `circle_path_node.py` v1→v5 in order — the diffs between versions ARE the
  PID tuning lesson.
* **Week 2:** Real hardware. cfclient console literacy → single-drone LPS
  flights of the phase-0 scripts → generate and read your own CSVs/plots.
* **Week 3:** `phase1_lps_swarm/` two-drone behaviors (start with
  `swarm_formation_hold.py`), then `follow_me.py`.
* **Week 4:** Phase 2 — read `docs/FLIGHT_RUNBOOK.md` end to end, then
  `phase2_aideck_wifi/hover_test.py` and `leader_follower.py`.
  **Read TROUBLESHOOTING.md before touching any AI-deck** — it will save
  you the week it cost.

## 11. Glossary

**CRTP** — Crazy RealTime Protocol, the command/telemetry packet format.
**CPX** — inter-chip/host packet exchange on the AI-deck (carries CRTP over
Wi-Fi). **TDoA2** — time-difference-of-arrival LPS mode (passive tags,
many drones). **EKF** — Extended Kalman Filter, the sensor-fusion state
estimator. **Setpoint** — the commanded target the controller chases.
**cmd_position** — streamed position setpoints (this repo's control mode).
**High-level commander** — onboard takeoff/goto/land trajectory generation
(used by the phase-1 scripts). **URI** — a drone's link address
(`radio://0/80/2M/E7E7E7E7XX` or `tcp://192.168.0.XX:5000`).
**Watchdog** — staleness timer; here, the follower lands itself if the
leader's stream stops. **Safe-zone clamp** — commands limited to the
anchor-measured room box, never refused, only clamped.

---

*Everything in this guide was executed on the machines and drones of Lab
B-419 during Summer 2026. Where reality disagreed with theory, reality is
documented in TROUBLESHOOTING.md — trust that file.*
