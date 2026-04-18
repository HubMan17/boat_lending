#!/usr/bin/env bash
# start_arduplane.sh — launch ArduPlane SITL (QuadPlane, Q-mode) from WSL
# configured for the Webots JSON-SITL backend, and relay MAVLink to
# Mission Planner on the Windows host via UDP 14550.
#
# Pipeline:
#   arduplane ──JSON-SITL UDP 9002/9003──▶ Webots controller (Windows)
#   arduplane (TCP :5760) ◀──── sitl_udp_bridge.py ────▶ UDP 14550 (MP/Windows)
#
# Ctrl+C stops both the bridge and arduplane.
#
# Env overrides:
#   ARDUPILOT_DIR  ArduPilot tree      (default: ~/boat_lending/ardupilot)
#   WIN_IP         Windows host IP     (default: WSL default-route gateway)
#   JSON_PORT      Webots UDP ctrl in  (default: 9002)
#   GCS_PORT       UDP port for MP     (default: 14550)
#   DEFAULTS       base params file    (default: <repo>/params/quadplane.parm)
#   EXTRA_DEFAULTS comma-separated extra param files layered on top of DEFAULTS
#   SERIAL2        extra serial endpoint (default: tcp:5763)
#   SERIAL3        extra serial endpoint (default: tcp:5764)
#   LOG_DIR        arduplane log dir   (default: /tmp)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
JSON_PORT="${JSON_PORT:-9002}"
GCS_PORT="${GCS_PORT:-14550}"
DEFAULTS="${DEFAULTS:-$REPO_DIR/params/quadplane.parm}"
EXTRA_DEFAULTS="${EXTRA_DEFAULTS:-}"
SERIAL2="${SERIAL2:-tcp:5763}"
SERIAL3="${SERIAL3:-tcp:5764}"
LOG_DIR="${LOG_DIR:-/tmp}"

if [[ -z "$WIN_IP" ]]; then
    echo "ERROR: could not auto-detect Windows host IP; set WIN_IP=..." >&2
    exit 1
fi

ARDU_BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"
if [[ ! -x "$ARDU_BIN" ]]; then
    echo "ERROR: $ARDU_BIN not found. Build with:" >&2
    echo "  cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf plane" >&2
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

ARDU_LOG="$LOG_DIR/arduplane.log"
echo "ardupilot dir  : $ARDUPILOT_DIR"
echo "windows host   : $WIN_IP"
echo "WSL IP (paste into Webots --sitl-address): $WSL_IP"
echo "JSON-SITL UDP  : $JSON_PORT (ctrl in)  $((JSON_PORT+1)) (FDM out)"
echo "GCS UDP port   : $GCS_PORT"
echo "defaults       : $DEFAULTS"
echo "serial2        : $SERIAL2"
echo "serial3        : $SERIAL3"
echo "arduplane log  : $ARDU_LOG"
echo

cd "$ARDUPILOT_DIR"

pkill -9 -x arduplane         2>/dev/null || true
pkill -9 -f 'mavproxy\.py'    2>/dev/null || true
pkill -9 -f 'sim_vehicle\.py' 2>/dev/null || true
pkill -9 -f 'sitl_udp_bridge' 2>/dev/null || true
sleep 1

"$ARDU_BIN" -S -I0 -w \
    --model webots-python \
    --sim-address "$WIN_IP" \
    --speedup 1 \
    --defaults "$DEFAULTS" \
    --serial2 "$SERIAL2" \
    --serial3 "$SERIAL3" \
    > "$ARDU_LOG" 2>&1 &
SITL_PID=$!
echo "[start_arduplane] arduplane PID=$SITL_PID"

cleanup() {
    echo
    echo "[start_arduplane] stopping arduplane (PID=$SITL_PID)..."
    kill -9 "$SITL_PID" 2>/dev/null || true
    wait "$SITL_PID"   2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[start_arduplane] waiting for TCP 5760 (up to 60s — start Webots now if not running)..."
for _ in $(seq 1 60); do
    if ss -tln 2>/dev/null | grep -q ':5760 '; then
        break
    fi
    sleep 1
done
if ! ss -tln 2>/dev/null | grep -q ':5760 '; then
    echo "ERROR: arduplane failed to open TCP 5760 — see $ARDU_LOG" >&2
    echo "HINT: Webots must be running with stage3_quadplane.wbt to feed FDM packets" >&2
    tail -20 "$ARDU_LOG" >&2 || true
    exit 1
fi

echo "[start_arduplane] launching UDP bridge (Ctrl+C to stop everything)"
exec python3 "$BRIDGE" \
    --sitl-host 127.0.0.1 --sitl-port 5760 \
    --gcs-host "$WIN_IP"  --gcs-port "$GCS_PORT"
