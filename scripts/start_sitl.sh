#!/usr/bin/env bash
# start_sitl.sh — launch ArduPlane SITL (QuadPlane) from WSL and relay its
# MAVLink to Mission Planner on the Windows host via UDP 14550.
#
# Pipeline (no MAVProxy, no sim_vehicle.py, no xterm — all three misbehave on
# this WSL box):
#
#   arduplane (TCP :5760, background)  <---->  sitl_udp_bridge.py (foreground)
#                                              |
#                                              +---udp--> <WIN_IP>:14550 (MP)
#
# Ctrl+C stops both the bridge and arduplane.
#
# Env overrides:
#   ARDUPILOT_DIR  ArduPilot tree    (default: ~/boat_lending/ardupilot)
#   WIN_IP         Windows host IP   (default: WSL default-route gateway)
#   GCS_PORT       UDP port for MP   (default: 14550)
#   LOG_DIR        arduplane log dir (default: /tmp)
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
GCS_PORT="${GCS_PORT:-14550}"
LOG_DIR="${LOG_DIR:-/tmp}"

if [[ -z "$WIN_IP" ]]; then
    echo "ERROR: could not auto-detect Windows host IP; set WIN_IP=..." >&2
    exit 1
fi

ARDU_BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/quadplane.parm"

if [[ ! -x "$ARDU_BIN" ]]; then
    echo "ERROR: $ARDU_BIN not found. Build with:" >&2
    echo "  cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf plane" >&2
    exit 1
fi
if [[ ! -f "$DEFAULTS" ]]; then
    echo "ERROR: defaults file missing: $DEFAULTS" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="$SCRIPT_DIR/sitl_udp_bridge.py"
if [[ ! -f "$BRIDGE" ]]; then
    echo "ERROR: bridge script not found: $BRIDGE" >&2
    exit 1
fi

ARDU_LOG="$LOG_DIR/arduplane.log"
echo "ardupilot dir : $ARDUPILOT_DIR"
echo "windows host  : $WIN_IP"
echo "GCS UDP port  : $GCS_PORT"
echo "arduplane log : $ARDU_LOG"
echo

cd "$ARDUPILOT_DIR"

pkill -9 -x arduplane         2>/dev/null || true
pkill -9 -f 'mavproxy\.py'    2>/dev/null || true
pkill -9 -f 'sim_vehicle\.py' 2>/dev/null || true
pkill -9 -f 'sitl_udp_bridge' 2>/dev/null || true
sleep 1

"$ARDU_BIN" -S -I0 --model quadplane --speedup 1 --defaults "$DEFAULTS" \
    > "$ARDU_LOG" 2>&1 &
SITL_PID=$!
echo "[start_sitl] arduplane PID=$SITL_PID"

cleanup() {
    echo
    echo "[start_sitl] stopping arduplane (PID=$SITL_PID)..."
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
    echo "ERROR: arduplane failed to open TCP 5760 — see $ARDU_LOG" >&2
    tail -20 "$ARDU_LOG" >&2 || true
    exit 1
fi

echo "[start_sitl] launching UDP bridge (Ctrl+C to stop everything)"
exec python3 "$BRIDGE" \
    --sitl-host 127.0.0.1 --sitl-port 5760 \
    --gcs-host "$WIN_IP"  --gcs-port "$GCS_PORT"
