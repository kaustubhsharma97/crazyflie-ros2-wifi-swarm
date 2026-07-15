# Flight Runbook — dual-stack Wi-Fi leader-follower (for whoever flies this next)

> **STATUS AT HANDOFF (July 2026):** the dual-stack configuration (LPS deck + AI-deck
> on the same drone) is **NOT yet flyable — it needs hardware modification first.**
> Read §0 before you mount anything. Everything else in this runbook (gates, ladder)
> is verified and applies the moment the stack enumerates.

---

## 0. Dual-stacking the LPS deck + AI-deck — read this BEFORE mounting

We tried the obvious thing — stack both decks with the pins available in the lab —
and the LPS deck **never enumerated** (cfclient never printed `bcDWM1000`). This is
not a defective deck. Two independent problems have to be solved, and **normal long
pins alone fix only the first one.**

Reference (keep this page open): Bitcraze expansion-deck system —
<https://www.bitcraze.io/documentation/system/platform/cf2-expansiondecks/>

### 0.1 Problem 1 — mechanical: connector reach and deck detection

* The Crazyflie 2.1+ and every deck use **female pass-through connectors fitted with
  male pins in two lengths**. Bitcraze supports exactly three arrangements:
  one deck on top; **one on top + one on bottom** (stock pin set); or **two decks on
  top** (requires the long male pin set).
* Decks are detected at power-on by the nRF51 reading each deck's **one-wire (OW)
  memory**. If a pin doesn't reach — wrong length, worn header, deck sandwiched
  outside the supported arrangements — the OW read fails silently and the firmware
  simply never initialises that deck. **No `deck(s) found` line at boot = that deck
  does not exist** as far as the STM32 is concerned; there is no retry.
* Practical consequences:
  * Prefer **AI-deck on the bottom, LPS deck on top** (LPS antenna clear upward, AI
    camera clear forward, battery secured by the top deck). This is a *supported*
    stock-pin arrangement mechanically.
  * If you must put both on top, you need the genuine Bitcraze **long male pins**;
    the assorted headers in the lab drawer do not reach through two decks — this is
    exactly the failure we hit. Custom long-pin headers can be fabricated in the ECE
    workshop (scoped, not done); match pin length so the OW pin seats fully on BOTH
    decks.
  * Mind the **orientation logos** printed on every deck — wrong orientation can
    permanently damage the deck and the Crazyflie.
  * Weight sanity: LPS 3.3 g + AI-deck 4.4 g ≈ 7.7 g of payload on a 27 g airframe.
    It flies, but sluggishly — the `> 4.1 V` battery gate below stops being optional.

### 0.2 Problem 2 — electrical: the decks fight over IO1 (this is the real blocker)

Even with perfect pins, **an unmodified LPS deck and AI-deck conflict electrically.**
Bitcraze's deck-to-deck compatibility matrix does *not* mark Loco + AI as "yes" — it
marks it **"with a patch or workaround it is possible"**. From the pin-allocation
table on the page above:

| Pin | LPS (Loco) deck uses it for | AI-deck uses it for |
|---|---|---|
| **IO1** | DW1000 **IRQ** | GAP8 **BOOT** strap |
| IO4 | (alternate IRQ, via solder bridge) | used by the AI-deck (Bitcraze note: shared with µSD-deck CS) |
| TX2/RX2 | (alternate RST, via solder bridge) | **CPX link to the ESP32** — our whole Wi-Fi path |
| SPI (SCK/MOSI/MISO) | DW1000 bus | free |
| UART1, I2C | free | GAP8 |

So:

* **IO1 is claimed by both decks.** The LPS interrupt line lands on the same pin the
  GAP8 samples at power-on to decide how to boot. Depending on who wins you get a
  GAP8 that boots wrong, a DW1000 whose interrupts are corrupted, or both. (You have
  met IO1 before: it is the same bootstrap pin that blocked the OTA ESP32 update on
  the retired deck aideck-7BD624. It moonlights.)
* The LPS deck **does** provide escape hatches: its IRQ and RST can be re-routed to
  the alternate pads (IO4 and TX2) via **solder bridges / 0 Ω resistors** — this is
  the "patch" Bitcraze means. But look at the table: **IO4 is also touched by the
  AI-deck, and TX2/RX2 carry CPX to the ESP32**, which is the very link this project
  flies on. The re-route target must be chosen so it collides with *neither*.
* Any hardware re-route must be mirrored in the **STM32 firmware build**: the Loco
  driver has an alternative-pin option in `crazyflie-firmware` menuconfig (Expansion
  deck configuration → Loco deck). Re-routing the pad without rebuilding the
  firmware (or vice-versa) gives you a deck that enumerates and then never gets an
  interrupt — positioning silently dead.

**Bottom line for the next student: an unmodified, off-the-shelf LPS deck will NOT
work stacked with the AI-deck — physical soldering on the LPS deck is mandatory.**
This is a small hardware-rework project, not a mounting exercise. Before touching a
soldering iron: (1) re-check the live
pin-allocation table at the link above (it changes), (2) search the Bitcraze forum
for the current recommended Loco+AI pin assignment, (3) pick the alternate pins so
they avoid IO1, the AI-deck's IO4 use, and UART2/CPX, (4) bridge the pads on the LPS
deck, (5) rebuild the STM32 firmware with the matching alt-pin config (remember the
build order from TROUBLESHOOTING.md: `cf2_defconfig` FIRST, `menuconfig` SECOND),
and only then mount.

### 0.3 Acceptance test for the stack (bench, props off)

0. **Eyeball check first — the LEDs tell you immediately.** With the stack seated
   and the drone powered, the **LPS deck's LED must glow on the top deck while the
   AI-deck's LEDs are glowing below it at the same time.** Both lit together =
   both decks are receiving power through the stacked headers. LPS LED dark while
   the AI-deck is lit = the LPS deck isn't seated/powered through the pins (§0.1)
   — do not bother launching cfclient yet. Note the LEDs only prove *power*, not
   detection: a deck can light up and still fail the one-wire enumeration, so
   step 1 remains the real gate.
1. Power on → cfclient console must print **`2 deck(s) found`** listing **both**
   `bcDWM1000` and `bcAI`. Anything less: stop, it's mechanical/OW (§0.1).
2. Console shows `Kalman (2)` and TDoA2 detected, **and** `got ip:` on the lab SSID
   — positioning and Wi-Fi alive on the same airframe at the same time.
3. `tools/find_decks.sh` sees the deck; `ros2 topic echo /cfX/pose --once` over the
   tcp:// link gives sane room coordinates and the pose keeps updating for 60 s
   (IRQ works — this is the check that catches a pad/firmware mismatch from §0.2).

Pass all three → continue to the Gates below exactly as written.

### 0.4 Until the rework is done (working fallbacks)

* **Missions needing positioning:** fly the Phase-1 configuration (LPS + Crazyradio,
  `config/crazyflies_SWARM.yaml`) — everything in `phase1_lps_swarm/` works today.
* **Wi-Fi link work:** single-deck AI-only drones over `crazyflies_dual_wifi.yaml`
  — full ROS 2 control, pose at 10 Hz, motors respond. **Do not attempt free flight
  without a positioning deck** — see `docs/phantom_drift.png` for why (11.3 m of
  imagined motion in 1.9 s from a drone sitting still on the floor).
* **Hybrid experiments:** `config/crazyflies_mixed.yaml` (leader on the dongle,
  follower on Wi-Fi) exists and is documented.

---

## Prereqs (once the stack passes §0.3)

Both drones with LPS + AI-deck mounted per §0.1, 8 anchors powered,
batteries > 4.1 V, tape marks per `docs/flight_day_floorplan.png`.

## Gates (do not skip — the leader-follower mission scripts contain no readiness
## checks by design; safety there is structural: clamps, step limits, watchdogs)

1. cfclient console per drone: `2 deck(s) found` (bcDWM1000 + bcAI),
   `Kalman (2)`, TDoA2 detected, `got ip:` on the lab SSID. Disconnect after.
2. `tools/find_decks.sh` → both port 5000 OPEN, IPs match `config/crazyflies_dual_wifi.yaml`.
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
5. `-p trajectory:=square` / `triangle`; then `swarm_circle_aideck.py` if desired
   (note: unlike the mission scripts, that one *does* carry a fail-closed preflight
   gate; `skip_preflight` overrides it).

Placement is dynamic anywhere interior to the anchors: circle extends 1.6 m
in −X from the leader (leader X ≥ 3.0); square/triangle extend +X/+Y
(leader ≈ (2.0, 2.8)); follower ~0.8 m behind in −X (step-limiter corrects
imperfect placement). Commands are clamped to X [0.30, 4.74], Y [−0.30, 6.90],
Z ≤ 2.20. Power-cycle both drones before every server launch (stale-socket rule).
