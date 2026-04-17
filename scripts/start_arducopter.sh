#!/usr/bin/env bash
# start_arducopter.sh — launch ArduCopter SITL from WSL configured for the
# Webots JSON-SITL backend, and relay its MAVLink to Mission Planner on the
# Windows host via UDP 14550.
#
# Pipeline (no MAVProxy, no sim_vehicle.py — both misbehave on this WSL box):
#
#   arducopter ──JSON-SITL UDP 9002/9003──▶ Webots controller (Windows)
#   arducopter (TCP :5760) ◀──── sitl_udp_bridge.py ────▶ UDP 14550 (MP/Windows)
#
# Ctrl+C stops both the bridge and arducopter.
#
# The Webots controller (ardupilot_vehicle_controller) runs inside Webots on
# Windows and must know the WSL IP to send FDM replies back. That IP is
# printed below — paste it into the `--sitl-address` controllerArgs entry in
# your .wbt file (or export SITL_ADDRESS=... on Windows if your controller
# reads it from env).
#
# Env overrides:
#   ARDUPILOT_DIR  ArduPilot tree      (default: ~/boat_lending/ardupilot)
#   WIN_IP         Windows host IP     (default: WSL default-route gateway)
#   JSON_PORT      Webots UDP ctrl in  (default: 9002)
#   GCS_PORT       UDP port for MP     (default: 14550)
#   DEFAULTS       base params file    (default: <repo>/params/iris.parm)
#   EXTRA_DEFAULTS comma-separated extra param files layered on top of DEFAULTS
#                  (default: empty; example: params/precland_copter.parm)
#   SERIAL2        extra serial endpoint (default: tcp:0.0.0.0:5763)
#   LOG_DIR        arducopter log dir  (default: /tmp)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
JSON_PORT="${JSON_PORT:-9002}"
GCS_PORT="${GCS_PORT:-14550}"
DEFAULTS="${DEFAULTS:-$REPO_DIR/params/iris.parm}"
EXTRA_DEFAULTS="${EXTRA_DEFAULTS:-}"
SERIAL2="${SERIAL2:-tcp:0.0.0.0:5763}"
LOG_DIR="${LOG_DIR:-/tmp}"

if [[ -z "$WIN_IP" ]]; then
    echo "ERROR: could not auto-detect Windows host IP; set WIN_IP=..." >&2
    exit 1
fi

ARDU_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
if [[ ! -x "$ARDU_BIN" ]]; then
    echo "ERROR: $ARDU_BIN not found. Build with:" >&2
    echo "  cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi
if [[ ! -f "$DEFAULTS" ]]; then
    echo "ERROR: defaults file missing: $DEFAULTS" >&2
    exit 1
fi

if [[ -n "$EXTRA_DEFAULTS" ]]; then
    IFS=',' read -ra _extras <<< "$EXTRA_DEFAULTS"
    for _f in "${_extras[@]}"; do
        _abs="$_f"
        [[ "$_f" != /* ]] && _abs="$REPO_DIR/$_f"
        if [[ ! -f "$_abs" ]]; then
            echo "ERROR: extra defaults file missing: $_abs" >&2
            exit 1
        fi
        DEFAULTS="$DEFAULTS,$_abs"
    done
fi

BRIDGE="$SCRIPT_DIR/sitl_udp_bridge.py"
if [[ ! -f "$BRIDGE" ]]; then
    echo "ERROR: bridge script not found: $BRIDGE" >&2
    exit 1
fi

WSL_IP="$(ip -4 -o addr show scope global 2>/dev/null \
    | awk '{print $4}' | cut -d/ -f1 | head -n1)"

ARDU_LOG="$LOG_DIR/arducopter.log"
echo "ardupilot dir  : $ARDUPILOT_DIR"
echo "windows host   : $WIN_IP"
echo "WSL IP (paste into Webots --sitl-address): $WSL_IP"
echo "JSON-SITL UDP  : $JSON_PORT (ctrl in)  $((JSON_PORT+1)) (FDM out)"
echo "GCS UDP port   : $GCS_PORT"
echo "defaults       : $DEFAULTS"
echo "serial2 (e2e)  : $SERIAL2"
echo "arducopter log : $ARDU_LOG"
echo

cd "$ARDUPILOT_DIR"

pkill -9 -x arducopter        2>/dev/null || true
pkill -9 -f 'mavproxy\.py'    2>/dev/null || true
pkill -9 -f 'sim_vehicle\.py' 2>/dev/null || true
pkill -9 -f 'sitl_udp_bridge' 2>/dev/null || true
sleep 1

"$ARDU_BIN" -S -I0 \
    --model webots-python \
    --sim-address "$WIN_IP" \
    --speedup 1 \
    --defaults "$DEFAULTS" \
    -C "$SERIAL2" \
    > "$ARDU_LOG" 2>&1 &
SITL_PID=$!
echo "[start_arducopter] arducopter PID=$SITL_PID"

cleanup() {
    echo
    echo "[start_arducopter] stopping arducopter (PID=$SITL_PID)..."
    kill -9 "$SITL_PID" 2>/dev/null || true
    wait "$SITL_PID"   2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ss -tln 2>/dev/null | grep -q ':5760 '; then
        break
    fi
    sleep 1
done
if ! ss -tln 2>/dev/null | grep -q ':5760 '; then
    echo "ERROR: arducopter failed to open TCP 5760 — see $ARDU_LOG" >&2
    tail -20 "$ARDU_LOG" >&2 || true
    exit 1
fi

echo "[start_arducopter] launching UDP bridge (Ctrl+C to stop everything)"
exec python3 "$BRIDGE" \
    --sitl-host 127.0.0.1 --sitl-port 5760 \
    --gcs-host "$WIN_IP"  --gcs-port "$GCS_PORT"
