#!/bin/bash
# Switch to SWARM config (cf231 + cf2)
SRC=~/Downloads/crazyflies_SWARM.yaml
DST1=~/crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
DST2=~/crazyswarm2_ws/install/crazyflie/share/crazyflie/config/crazyflies.yaml
cp "$SRC" "$DST1" && cp "$SRC" "$DST2" && echo "✅ SWARM config active (cf231 + cf2)" || echo "❌ failed — check that $SRC exists"
