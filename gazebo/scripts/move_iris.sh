#!/bin/bash
# Teleport the iris drone around via gz set_pose service.
# Visible in the gst camera viewer as a changing downward view.
set -e

MODEL='iris_with_gimbal'
SVC='/world/iris_runway/set_pose'

send_pose() {
  local x=$1 y=$2 z=$3
  gz service -s "$SVC" \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "name: \"$MODEL\", position: { x: $x, y: $y, z: $z }" >/dev/null
  echo ">> teleported to ($x, $y, $z)"
}

echo ">> ascend to 20m (2s/step)"
for z in 2 5 10 15 20; do
  send_pose 0 0 $z
  sleep 1.5
done

echo ">> slide East to 10m (1s/step)"
for x in 2 4 6 8 10; do
  send_pose $x 0 20
  sleep 1
done

echo ">> slide North to 10m"
for y in 2 4 6 8 10; do
  send_pose 10 $y 20
  sleep 1
done

echo ">> descend to 2m above runway"
for z in 15 10 5 2; do
  send_pose 10 10 $z
  sleep 1
done

echo ">> done. to reset: send_pose 0 0 0.2 (or just stop_sim.sh)"
