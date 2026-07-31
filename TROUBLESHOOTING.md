# Troubleshooting Knowledge Base

Every failure below was hit on real hardware in B-419, root-caused, and
fixed (or definitively characterized). Symptom → root cause → fix, in
roughly the order encountered. If you inherit these drones, read this
before touching anything.

---

## 1. `AIDECK: Not setting up WiFi` after flashing custom firmware

**Symptom:** STM32 console shows the AI-deck driver initializing but the
deck never joins the network.
**Root cause:** Wi-Fi credentials are set via `make menuconfig` in
crazyflie-firmware (Expansion deck configuration → Support the AI-deck →
WiFi setup at startup → Connect to a Wifi network). Running
`make cf2_defconfig` **after** menuconfig silently wipes them.
**Fix:** order is `cf2_defconfig` → `menuconfig` → `make`. Verify before
flashing: `grep -i DECK_AI build/.config` must show the STA option + SSID.
Note the same binary (`build/cf2.bin`) is flashed to every drone — the
radio address lives in the nRF config block, not in this firmware.

## 2. GAP8 demo firmware hijacks the Wi-Fi (deck joins nothing / makes its own AP)

**Symptom:** console shows `GAP8: -- Face tracking example --` or
`GAP8: Setting up WiFi AP`, then `wifi_init_softap finished`; a "WiFi
streaming example" network appears instead of the deck joining the lab SSID.
Often accompanied by `UDP send failed: errno 118/12` spam.
**Root cause:** factory/demo GAP8 images (face-tracking, wifi-img-streamer
built `with_ap`) configure the ESP32's Wi-Fi themselves and override the
Crazyflie's STA instructions. CPX Wi-Fi control is last-writer-wins and the
GAP8 writes ~2.4 s after boot.
**Fix:** flash the GAP8 with a passive example before Wi-Fi commissioning:

```bash
git clone https://github.com/bitcraze/aideck-gap8-examples.git && cd aideck-gap8-examples
sudo docker run --rm -v ${PWD}:/module --privileged bitcraze/aideck \
    tools/build/make-example examples/other/hello_world_gap8 clean all
cfloader flash examples/other/hello_world_gap8/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img \
    deck-bcAI:gap8-fw -w radio://0/80/2M/E7E7E7E7XX
```

The build's final JTAG step fails with "no device found" — **ignore it**;
the `.img` is already built at that point. A GAP8 OTA flash hanging at
4%/99% can still have written successfully — power-cycle and check the
console before retrying.

## 3. `Connection refused` on port 5000 — ancient ESP32 firmware (terminal case)

**Symptom:** deck joins Wi-Fi, gets an IP, but nothing ever listens on
TCP 5000. Boot log fingerprint: an extra `CPX: CPX connected` line and
`WIFI: Client connected` messages phase-locked to the ESP boot millisecond
(e.g. always at t=X437 ms, every 5000 ms) — that "client" is the firmware's
internal status poll, **not** a network client (verified by honeypot:
aliasing the deck's IP onto a laptop and listening caught nothing).
**Root cause:** pre-CPX-era ESP32 firmware (UDP-streamer generation) has no
TCP server at all. On one deck (aideck-7BD624) OTA reflash of the ESP32 was
additionally impossible — cfloader hangs at "reset to bootloader" because
the old firmware drives the IO1 bootstrap pin, blocking entry into the ESP
ROM bootloader.
**Fix:** update the ESP32 (`cfloader flash aideck_esp.bin deck-bcAI:esp-fw
-w <uri>`, ~3 quiet minutes — do not interrupt) — success shows as a
*personality change*: the extra CPX line and 5-second ticks disappear. If
OTA is pin-blocked (7BD624), only a wired JTAG/serial flash or a **deck
swap** fixes it. Note: cfclient's full bootloader "Program" also reflashes
the STM32 to stock — re-flash the custom Wi-Fi build afterward (bit us
twice).

## 4. One-client TCP rule / stale sockets

**Symptom:** port 5000 refused on a previously working deck.
**Root cause:** the ESP32's CPX server serves exactly one client and stops
listening while occupied; after an unclean client shutdown (Ctrl-C'd
server, crashed process) it can hold the dead connection.
**Fix / ritual:** power-cycle both drones before every server launch; never
leave cfclient connected while the crazyswarm2 server runs.

## 5. Deck IPs drift (no router admin access)

**Symptom:** server dials yesterday's IP; connection refused; the
all-or-nothing server tears down the healthy drone too (its `struct.error:
unpack requires a buffer of 2 bytes` traceback is just the reader thread
hitting the closed socket — ignore it).
**Fix:** `tools/find_decks.sh` — identifies both decks by MAC via
`arp-scan`, prints current IPs and probes port 5000. Run before every
session; update the yaml if an IP moved. (DHCP reservations would solve
this permanently but require router admin.)

## 6. Crazyswarm2 over TCP — what actually works

`backend:=cflib` accepts `tcp://<ip>:5000` URIs and connects (verified:
both drones simultaneously, mixed `radio://` + `tcp://` also supported in
one session). The default C++ backend is radio-only. Yaml parsing requires
the full `motion_capture` block (`tracking`, `marker`, `dynamics`) even
with `mocap:=False` — missing keys throw launch-time KeyErrors.
Measured link quality (idle): pose 10 Hz steady, worst inter-message gap
226 ms, ping RTT 5–26 ms — hence the follower watchdog default of 0.8 s.

## 7. Flight without a positioning deck is impossible (3 controlled crashes)

**Symptom:** drone flips or veers instantly at takeoff; motionless drone
on the floor reports positions like (−8.75, −33.68) drifting at ~6 m/s
(`docs/phantom_drift.png`).
**Root cause:** the AI-deck provides communication only. With no LPS/Flow
deck the estimator dead-reckons on IMU noise; the position controller
chases the phantom and slams the airframe over. Wi-Fi delivers commands
perfectly — the drone simply cannot know where it is. No public
implementation flies AI-deck-only; Bitcraze's own Wi-Fi flight example
requires a Flow deck.
**Fix:** positioning deck. Also restored to `leader_follower.py`: room
safe-zone clamping from the anchor geometry, landing to `home_z + 0.03`
(artifact-proof), and a dual watchdog (position stream + leader heartbeat).

## 8. LPS quirks that are NOT faults

* **Negative floor-Z:** drones sit below the 0.30 m low-anchor plane, so
  ground Z reads slightly negative by geometry. Height verified accurate at
  flight altitude. Never gate on floor-Z.
* **Divergence guards need grace:** a guard with no grace period trips on
  single-sample TDoA2 noise right after takeoff (use ~2.5 s grace + 5
  consecutive bad samples, or rely on the safe-zone clamp instead).
* **Single-sample Z glitches** in logs are TDoA2 artifacts, not flight events.
* **+8 cm Y-bias** in the high-Y room region (suspected anchor geometry) —
  characterize with `static_check.py` before trusting millimeters there.

## 9. LPS + AI-deck stacking (open hardware item — rework required)

**Symptom:** stacked LPS + AI-deck → cfclient never prints `bcDWM1000`; the
LPS deck is invisible to the firmware.
**Root cause(s) — there are two, and fixing only the first is not enough:**

1. *Mechanical:* with the headers available in the lab the LPS deck's pins
   (including the one-wire detection pin) do not seat through the stack, so
   deck detection fails silently at boot. Proper Bitcraze long-pin headers
   or ECE-lab fabricated equivalents are required (scoped, deferred for
   time/risk). Quick check: the **LPS deck's LED should glow on top while
   the AI-deck LEDs glow below** — LED dark = not even powered through the
   pins. (LEDs prove power only, not enumeration.)
2. *Electrical:* even with perfect pins, **unmodified decks conflict on
   IO1** — the LPS DW1000 IRQ and the AI-deck's GAP8 BOOT strap share it.
   Bitcraze's deck-compatibility matrix rates Loco + AI as *"with a patch
   or workaround it is possible"*, not "yes". The LPS deck must be
   **solder-bridged to its alternate IRQ/RST pads**, chosen to avoid the
   AI-deck's IO4 use and the CPX UART2 pins, and the STM32 firmware must be
   rebuilt with the matching Loco alternate-pin config.

**Fix / procedure, acceptance test, and fallbacks:** `docs/FLIGHT_RUNBOOK.md`
§0 (single authority for the stack rework). Target arrangement remains
**LPS on top (UWB antenna clear), AI-deck underneath**. Reference:
<https://www.bitcraze.io/documentation/system/platform/cf2-expansiondecks/>

## 10. Toolchain gotchas (Ubuntu 22.04)

Docker: `permission denied` on the socket → `sudo` or usermod + full
re-login; "network not found" → `systemctl restart docker` or run the
build with `--network=none`; and a failed `cd` (repo never cloned) makes
docker mount the wrong directory — always check the **first** error line
of a paste, not the last.
