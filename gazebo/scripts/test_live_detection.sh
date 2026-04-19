#!/usr/bin/env bash
# test_live_detection.sh — manual B-level smoke for P3b.3+P3b.4.
#
# One command: Gazebo + SITL + camera + gst viewer + companion (gstreamer
# backend) + continuous teleport of the drone to hover above the marker at
# (5, 0, 10) so the downward camera frames the 1.5 m ArUco marker.
#
# Success criterion: companion log shows "det=N/M" with N growing (ArUco
# is detected on most frames). Runs for DURATION seconds then cleans up.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
DURATION="${DURATION:-25}"
WORLD_NAME=iris_precland
MODEL=iris_with_downcam

SIM_LOG="/tmp/test_live_sim.log"
COMP_LOG="/tmp/test_live_companion.log"

SIM_PID=""
TELE_PID=""
COMP_PID=""

cleanup() {
  echo
  echo "[test_live] stopping..."
  [[ -n "$COMP_PID" ]] && kill -INT  "$COMP_PID" 2>/dev/null || true
  [[ -n "$TELE_PID" ]] && kill -9    "$TELE_PID" 2>/dev/null || true
  [[ -n "$SIM_PID"  ]] && kill -9    "$SIM_PID"  2>/dev/null || true
  pkill -9 -x gz-sim-server              2>/dev/null || true
  pkill -9 -x ruby                       2>/dev/null || true
  pkill -9 -x arducopter                 2>/dev/null || true
  pkill -9 -f sitl_udp_bridge            2>/dev/null || true
  pkill -9 -f 'gst-launch-1.0.*udpsrc.*5600' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">> launching run_iris_sim.sh (no viewer — companion binds UDP:5600) (bg, log -> $SIM_LOG)"
bash "$SCRIPT_DIR/run_iris_sim.sh" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
disown

echo ">> waiting up to 45s for TCP :5763 (SITL serial2 for companion)"
for _ in $(seq 1 45); do
  sleep 1
  ss -tln 2>/dev/null | grep -q ':5763 ' && break
done
if ! ss -tln 2>/dev/null | grep -q ':5763 '; then
  echo "ERROR: :5763 never opened — sim log tail:"
  tail -40 "$SIM_LOG"
  exit 1
fi
echo "   :5763 open"

echo ">> teleport loop: $MODEL at (5, 0, 10) upright @ 5 Hz"
(
  while true; do
    gz service -s "/world/${WORLD_NAME}/set_pose" \
      --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 500 \
      --req "name: \"$MODEL\", position: {x: 5, y: 0, z: 10}, orientation: {x: 0, y: 0, z: 0, w: 1}" \
      >/dev/null 2>&1 || true
    sleep 0.2
  done
) &
TELE_PID=$!
sleep 2

echo ">> running companion for ${DURATION}s (stage=1, gstreamer, marker_id=0)"
cd "$REPO_DIR"
timeout --preserve-status "$DURATION" stdbuf -oL -eL python3 -u companion/companion.py \
  --stage 1 \
  --camera-backend gstreamer --cam-port 5600 \
  --mav-url tcp:127.0.0.1:5763 \
  --marker-id 0 \
  --show 2>&1 | tee "$COMP_LOG" &
COMP_PID=$!
wait "$COMP_PID" 2>/dev/null || true

echo
echo "=== SUMMARY ==="
LAST_STATS=$(grep 'companion: frames=' "$COMP_LOG" | tail -1 || true)
if [[ -n "$LAST_STATS" ]]; then
  echo "$LAST_STATS"
  DET=$(echo "$LAST_STATS" | grep -oE 'det=[0-9]+/[0-9]+' | head -1)
  echo "detection count: $DET"
else
  echo "no 'frames=' progress line seen — companion log tail:"
  tail -15 "$COMP_LOG"
fi
