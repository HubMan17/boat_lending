#!/usr/bin/env bash
# run_iris_sim.sh — Phase 3b all-in-one bring-up:
#   headless Gazebo (iris_precland.sdf) + arducopter JSON SITL +
#   UDP bridge to Mission Planner on Windows.
#
# Pipeline:
#   gz sim -s -r iris_precland.sdf ──9002/9003 UDP──▶ arducopter (JSON)
#   arducopter (TCP :5760) ◀── sitl_udp_bridge.py ──▶ UDP 14550 (MP/Windows)
#   GstCameraPlugin ── H.264/RTP ──▶ UDP :5600 (companion / gst viewer)
#
# Usage:
#   bash gazebo/scripts/run_iris_sim.sh [--viewer]
#
# Env overrides:
#   ARDUPILOT_DIR   ArduPilot tree      (default: ~/boat_lending/ardupilot)
#   AG_DIR          ardupilot_gazebo    (default: ~/ardupilot_gazebo)
#   WORLD           SDF world file      (default: <repo>/gazebo/worlds/iris_precland.sdf)
#   WIN_IP          Mission Planner IP  (default: WSL default-route gateway)
#   GCS_PORT        UDP port for MP     (default: 14550)
#   SERIAL2         --serial2 URI       (default: tcp:5763)
#   DEFAULTS        params file         (default: autotest copter.parm)
#   EXTRA_DEFAULTS  comma-separated extra parm overlays (repo-relative ok)
#   LOG_DIR         log directory       (default: /tmp)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

WANT_VIEWER=0
for arg in "$@"; do
  case "$arg" in
    --viewer)    WANT_VIEWER=1 ;;
    --no-viewer) WANT_VIEWER=0 ;;
    -h|--help)
      sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
AG_DIR="${AG_DIR:-$HOME/ardupilot_gazebo}"
WORLD="${WORLD:-$REPO_DIR/gazebo/worlds/iris_precland.sdf}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
GCS_PORT="${GCS_PORT:-14550}"
SERIAL2="${SERIAL2:-tcp:5763}"
DEFAULTS="${DEFAULTS:-$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm}"
LOG_DIR="${LOG_DIR:-/tmp}"

ARDU_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
BRIDGE="$REPO_DIR/scripts/sitl_udp_bridge.py"

[[ -x "$ARDU_BIN" ]] || { echo "ERROR: $ARDU_BIN not found (build with './waf copter')" >&2; exit 1; }
[[ -f "$WORLD"   ]] || { echo "ERROR: world file missing: $WORLD" >&2; exit 1; }
[[ -f "$BRIDGE"  ]] || { echo "ERROR: bridge script missing: $BRIDGE" >&2; exit 1; }
[[ -f "$DEFAULTS" ]] || { echo "ERROR: defaults file missing: $DEFAULTS" >&2; exit 1; }
[[ -d "$AG_DIR/build" ]] || { echo "ERROR: ardupilot_gazebo build dir missing at $AG_DIR/build" >&2; exit 1; }

if [[ -n "${EXTRA_DEFAULTS:-}" ]]; then
  IFS=',' read -ra _extras <<< "$EXTRA_DEFAULTS"
  for _f in "${_extras[@]}"; do
    _abs="$_f"
    [[ "$_f" != /* ]] && _abs="$REPO_DIR/$_f"
    [[ -f "$_abs" ]] || { echo "ERROR: extra defaults file missing: $_abs" >&2; exit 1; }
    DEFAULTS="$DEFAULTS,$_abs"
  done
fi

# bashrc is not sourced by non-interactive bash; export explicitly
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/gazebo/models:$REPO_DIR/gazebo/worlds:$AG_DIR/models:$AG_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

WORLD_NAME="$(basename "$WORLD" .sdf)"
GZ_LOG="$LOG_DIR/gz_sim.log"
ARDU_LOG="$LOG_DIR/arducopter.log"
VIEWER_LOG="$LOG_DIR/gst_viewer.log"

echo "world        : $WORLD"
echo "ardupilot dir: $ARDUPILOT_DIR"
echo "ag dir       : $AG_DIR"
echo "windows host : $WIN_IP"
echo "gcs port     : $GCS_PORT"
echo "serial2      : $SERIAL2"
echo "defaults     : $DEFAULTS"
echo "viewer       : $([[ $WANT_VIEWER -eq 1 ]] && echo enabled || echo disabled)"
echo "gz log       : $GZ_LOG"
echo "copter log   : $ARDU_LOG"
echo

pkill -9 -x gz-sim-server 2>/dev/null || true
pkill -9 -x ruby          2>/dev/null || true
pkill -9 -x arducopter    2>/dev/null || true
pkill -9 -f sitl_udp_bridge 2>/dev/null || true
pkill -9 -f 'gst-launch-1.0.*udpsrc.*5600' 2>/dev/null || true
sleep 1

GZ_PID=""
SITL_PID=""
VIEWER_PID=""

cleanup() {
  echo
  echo "[run_iris_sim] stopping all..."
  [[ -n "$VIEWER_PID" ]] && kill -9 "$VIEWER_PID" 2>/dev/null || true
  [[ -n "$SITL_PID"   ]] && kill -9 "$SITL_PID"   2>/dev/null || true
  [[ -n "$GZ_PID"     ]] && kill -9 "$GZ_PID"     2>/dev/null || true
  pkill -9 -x gz-sim-server   2>/dev/null || true
  pkill -9 -x ruby            2>/dev/null || true
  pkill -9 -f sitl_udp_bridge 2>/dev/null || true
  pkill -9 -f 'gst-launch-1.0.*udpsrc.*5600' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo '>> starting gz sim (headless)'
rm -f "$GZ_LOG"
setsid gz sim -s -r -v 2 "$WORLD" > "$GZ_LOG" 2>&1 < /dev/null &
GZ_PID=$!
disown
echo "   gz pid=$GZ_PID"

echo '>> waiting up to 20s for physics to tick'
for _ in $(seq 1 20); do
  sleep 1
  if timeout 1 gz topic -e -t "/world/$WORLD_NAME/stats" -n 1 > /dev/null 2>&1; then
    echo "   sim ticking"
    break
  fi
done

echo '>> starting arducopter JSON SITL'
cd "$ARDUPILOT_DIR"
setsid ./build/sitl/bin/arducopter -S -I0 -w \
  --model JSON --speedup 1 \
  --defaults "$DEFAULTS" \
  --serial2 "$SERIAL2" \
  > "$ARDU_LOG" 2>&1 < /dev/null &
SITL_PID=$!
disown
echo "   arducopter pid=$SITL_PID"

echo '>> waiting up to 30s for TCP :5760'
for _ in $(seq 1 30); do
  sleep 1
  if ss -tln 2>/dev/null | grep -q ':5760 '; then
    echo "   :5760 open"
    break
  fi
done
if ! ss -tln 2>/dev/null | grep -q ':5760 '; then
  echo "ERROR: arducopter didn't open TCP 5760 — see $ARDU_LOG" >&2
  tail -20 "$ARDU_LOG" >&2 || true
  exit 1
fi

echo '>> enabling camera streaming'
CAM_TOPIC=""
for _ in $(seq 1 15); do
  CAM_TOPIC=$(gz topic -l 2>/dev/null | grep enable_streaming | head -1 || true)
  [[ -n "$CAM_TOPIC" ]] && break
  sleep 1
done
if [[ -n "$CAM_TOPIC" ]]; then
  echo "   topic: $CAM_TOPIC"
  gz topic -t "$CAM_TOPIC" -m gz.msgs.Boolean -p 'data: true' || echo "   WARN: enable_streaming publish failed"
else
  echo "   WARN: no enable_streaming topic found — camera may not stream"
fi

if [[ $WANT_VIEWER -eq 1 ]]; then
  echo '>> starting gst viewer (bg, log -> '"$VIEWER_LOG"')'
  rm -f "$VIEWER_LOG"
  setsid gst-launch-1.0 -q udpsrc port=5600 \
      caps='application/x-rtp,encoding-name=H264,payload=96' ! \
      rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false \
      > "$VIEWER_LOG" 2>&1 < /dev/null &
  VIEWER_PID=$!
  disown
  echo "   viewer pid=$VIEWER_PID"
fi

echo '>> launching UDP bridge (Ctrl+C stops everything)'
exec python3 "$BRIDGE" \
  --sitl-host 127.0.0.1 --sitl-port 5760 \
  --gcs-host "$WIN_IP"  --gcs-port "$GCS_PORT"
