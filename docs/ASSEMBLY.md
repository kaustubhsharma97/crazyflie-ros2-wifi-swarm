# Assembly — Crazyflie 2.1+ from the box (read before touching the propellers)

New drone or rebuilt after a crash? This page takes you from sealed box to
"ready for the runbook gates." Assembly takes ~10 minutes; getting the
propellers wrong takes one takeoff to discover (the drone flips instantly).
Official reference with photos/video, keep it open alongside:
<https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/>

## 0. BEFORE assembling anything — power-on self test

Connect the bare board to a micro-USB power source:

* **M4 LED blinks GREEN 5× fast → PASS**, continue.
* **M1 LED blinks RED 5× fast (repeating) → FAIL** — stop, don't assemble a
  dead board; check the Bitcraze support forum.

## 1. Motors

1. **Twist each motor's wire pair** along its length (reduces electrical
   noise, fits the mount hooks better).
2. Press-fit each motor into a motor mount, all the way to the stop.
   Use a table edge against the mount if it's stiff — **never press on the
   motor shaft**, that damages the motor.
3. It does **not** matter which motor goes in which position — motors are
   identical; only the propellers are handed.
4. Plug the four motor connectors into the board.

## 2. Propellers — THE step people get wrong

There are **two kinds of propellers, in two separate bags**:

| Prop marking | Rotation | Mount on motors |
|---|---|---|
| **47-17**  (older kits: "B"/"B1"/"B2") | CCW | **M1 and M3** |
| **47-17R** (older kits: "A"/"A1"/"A2") | CW  | **M2 and M4** |

Memory rule used in this lab: **"R-props go on M2 & M4; plain props on M1 & M3."**

Motor numbers M1–M4 are printed on the PCB next to each arm — read the
board, don't guess from orientation. Three visual checks per prop:

* **Convex (curved) side faces UP.**
* The **sharper corner of the blade tip trails** the rotation direction.
* After mounting all four: diagonal props match (M1↔M3 same kind,
  M2↔M4 same kind). If two *adjacent* props match, you've mixed them.

Wrong-handed or upside-down props don't fail gently — the drone flips on
takeoff. If a fresh build flips immediately: it's the props, 95% of the time.

## 3. Battery, pad and headers

1. Stick the **foam pad** on the top side between the expansion headers
   (grips the battery, protects the electronics).
2. The box has **short and long pin headers**. Bare drone or one deck on
   top: short pins. Deck underneath (e.g. the AI-deck arrangement) or
   stacking: long pins — and read `FLIGHT_RUNBOOK.md` §0 before stacking
   anything.
3. Mount decks with the **orientation logo** matching the board — wrong
   orientation can permanently damage deck and drone.
4. Connect the battery; tuck the wires under the PCB.

## 4. First power-on

* The power switch is a **push button**, not a slider.
* On boot the drone **calibrates its sensors — it must be absolutely
  still on a level surface** for the first seconds. Don't pick it up.
* Orientation: the **blue LEDs are the BACK** of the drone.
* Charging: plug micro-USB with the drone powered on; the back-left blue
  LED blinks while charging. **Never charge LiPo batteries unattended.**

## 5. Hand-off to the runbook

Assembled and calibrated → connect with `cfclient`, check the console
(decks detected, no errors), then fly the shakedown ladder in
`docs/FLIGHT_RUNBOOK.md` starting with `hover_test.py` at low height —
that first 30 s hover is what validates your propeller work.

## Crash-repair quick reference

After a crash: check each prop for chips (replace in pairs per diagonal),
confirm no motor mount cracked, re-seat any deck, and re-run the power-on
self test of §0 before flying again. Spare motors/props live in the lab
spares box — reorder before the last spare is used, not after.
