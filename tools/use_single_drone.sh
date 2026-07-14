#!/bin/bash
# Switch to SINGLE-DRONE config (cf231 only)
SRC=~/Downloads/crazyflies_SINGLE.yaml
DST1=~/crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
DST2=~/crazyswarm2_ws/install/crazyflie/share/crazyflie/config/crazyflies.yaml
cp "$SRC" "$DST1" && cp "$SRC" "$DST2" && echo "✅ SINGLE-DRONE config active (cf231 only)" || echo "❌ failed — check that $SRC exists"
