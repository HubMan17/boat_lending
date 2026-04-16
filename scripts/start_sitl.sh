#!/usr/bin/env bash
# start_sitl.sh — launch ArduPlane SITL (QuadPlane) from WSL with MAVLink
# forwarded to the Windows host on UDP 14550 so Mission Planner can attach.
#
# Run from inside WSL (default distro Ubuntu-22.04). Keep the terminal open:
# sim_vehicle.py hosts MAVProxy in the foreground — Ctrl+C stops both SITL
# and MAVProxy cleanly.
#
# Env overrides:
#   ARDUPILOT_DIR  path to the ArduPilot tree  (default: ~/boat_lending/ardupilot)
#   WIN_IP         Windows host IP from WSL    (default: auto-detect via default route)
#   GCS_PORT       UDP port for MP on Windows  (default: 14550)
#   EXTRA_OUT      additional --out=... value  (optional, e.g. companion sink)
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
WIN_IP="${WIN_IP:-$(ip route show | awk '/^default/ {print $3; exit}')}"
GCS_PORT="${GCS_PORT:-14550}"

if [[ -z "$WIN_IP" ]]; then
    echo "ERROR: could not auto-detect Windows host IP; set WIN_IP=..." >&2
    exit 1
fi

echo "ardupilot dir : $ARDUPILOT_DIR"
echo "windows host  : $WIN_IP"
echo "GCS UDP port  : $GCS_PORT"
echo

cd "$ARDUPILOT_DIR"

OUT_ARGS=(--out="udpout:${WIN_IP}:${GCS_PORT}")
if [[ -n "${EXTRA_OUT:-}" ]]; then
    OUT_ARGS+=(--out="$EXTRA_OUT")
fi

exec ./Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f quadplane \
    --no-rebuild \
    "${OUT_ARGS[@]}"
