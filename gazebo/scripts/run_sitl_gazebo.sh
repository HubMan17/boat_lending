#!/usr/bin/env bash
# run_sitl_gazebo.sh — bring up headless Gazebo Harmonic + ArduCopter SITL
# + UDP bridge for Mission Planner (Windows host). Verifies the JSON-SITL
# coupling over UDP 9002/9003 between ArduPilotPlugin and arducopter.
#
# Pipeline:
#
#   gz sim -s -r iris_coupling.sdf  ──9002/9003 UDP──▶ arducopter (JSON)
#   arducopter (TCP :5760) ◀── sitl_udp_bridge.py ──▶ UDP 14550 (MP/Windows)
#
# Connecting a TCP client to :5760 is REQUIRED: ArduCopter's main loop waits
# for a serial/GCS client before ticking. Without it the vehicle loop never
# runs, no servos are ever sent, and the UDP coupling stays silent
# (root cause of what P3b.2 was diagnosing).
#
# Env overrides:
#   ARDUPILOT_DIR   ArduPilot tree      (default: ~/boat_lending/ardupilot)
#   WIN_IP          Windows host IP     (default: WSL default-route gateway)
#   WORLD           SDF world file      (default: <repo>/gazebo/worlds/iris_coupling.sdf)
#   AG_DIR          ardupilot_gazebo    (default: ~/ardupilot_gazebo)
#   DEFAULTS        params file         (default: autotest copter.parm)
#   GCS_PORT        UDP port for MP     (default: 14550)
#   SERIAL2         --serial2 URI       (default: tcp:5763)
#   LOG_DIR         log directory       (default: /tmp)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
AG_DIR="${AG_DIR:-$HOME/ardupilot_gazebo}"
WORLD="${WORLD:-$REPO_DIR/gazebo/worlds/iris_coupling.sdf}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
GCS_PORT="${GCS_PORT:-14550}"
DEFAULTS="${DEFAULTS:-$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm}"
SERIAL2="${SERIAL2:-tcp:5763}"
LOG_DIR="${LOG_DIR:-/tmp}"

ARDU_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
BRIDGE="$REPO_DIR/scripts/sitl_udp_bridge.py"

if [[ ! -x "$ARDU_BIN" ]]; then
  echo "ERROR: $ARDU_BIN not found — build with './waf copter'" >&2; exit 1
fi
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: world file missing: $WORLD" >&2; exit 1
fi
if [[ ! -f "$DEFAULTS" ]]; then
  echo "ERROR: defaults file missing: $DEFAULTS" >&2; exit 1
fi
if [[ ! -f "$BRIDGE" ]]; then
  echo "ERROR: bridge script missing: $BRIDGE" >&2; exit 1
fi
if [[ ! -d "$AG_DIR/build" ]]; then
  echo "ERROR: ardupilot_gazebo build dir missing at $AG_DIR/build" >&2; exit 1
fi

# bashrc is not sourced by non-interactive bash; export explicitly
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$AG_DIR/models:$AG_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

GZ_LOG="$LOG_DIR/gz_sim.log"
ARDU_LOG="$LOG_DIR/arducopter.log"

echo "world        : $WORLD"
echo "ardupilot dir: $ARDUPILOT_DIR"
echo "ag dir       : $AG_DIR"
echo "windows host : $WIN_IP"
echo "gcs port     : $GCS_PORT"
echo "serial2      : $SERIAL2"
echo "defaults     : $DEFAULTS"
echo "gz log       : $GZ_LOG"
echo "copter log   : $ARDU_LOG"
echo

# Clean up any stale processes
pkill -9 -x gz-sim-server 2>/dev/null || true
pkill -9 -x ruby          2>/dev/null || true
pkill -9 -x arducopter    2>/dev/null || true
pkill -9 -f sitl_udp_bridge 2>/dev/null || true
sleep 1

echo '>> starting gz sim (headless)'
rm -f "$GZ_LOG"
setsid gz sim -s -r -v 2 "$WORLD" > "$GZ_LOG" 2>&1 < /dev/null &
GZ_PID=$!
disown
echo "   gz pid=$GZ_PID"

echo '>> waiting up to 20s for physics to tick'
WORLD_NAME="$(basename "$WORLD" .sdf)"
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
  kill -9 "$GZ_PID" "$SITL_PID" 2>/dev/null || true
  exit 1
fi

cleanup() {
  echo
  echo "[run_sitl_gazebo] stopping all..."
  kill -9 "$SITL_PID" 2>/dev/null || true
  kill -9 "$GZ_PID"   2>/dev/null || true
  pkill -9 -x gz-sim-server   2>/dev/null || true
  pkill -9 -x ruby            2>/dev/null || true
  pkill -9 -f sitl_udp_bridge 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo '>> launching UDP bridge (Ctrl+C stops everything)'
exec python3 "$BRIDGE" \
  --sitl-host 127.0.0.1 --sitl-port 5760 \
  --gcs-host "$WIN_IP"  --gcs-port "$GCS_PORT"
