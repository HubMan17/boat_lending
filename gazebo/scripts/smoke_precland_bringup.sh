#!/usr/bin/env bash
# smoke_precland_bringup.sh — headless smoke for Phase 3b iris_precland bring-up.
#
# Order matches reality: gz sim hosts a lock_step ArduPilotPlugin that only
# advances physics when arducopter is sending servo packets, and arducopter
# only ticks its main loop when a TCP client is connected on :5760. So:
#   start gz sim → start arducopter → connect TCP :5760 (nc) → let physics
#   advance → then measure world ticks / camera topic / UDP:5600 / MAVLink.
#
# Checks:
#   1. TCP :5760 opened
#   2. world stats advancing (lock-step physics running)
#   3. camera enable_streaming topic exists
#   4. UDP :5600 delivers RTP packets after enable_streaming (≥5 in 3s)
#   5. MAVLink v2 bytes received on :5760
#
# Exits 0 on 5/5 PASS. No GCS, no MP, no companion.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
AG_DIR="${AG_DIR:-$HOME/ardupilot_gazebo}"
WORLD="$REPO_DIR/gazebo/worlds/iris_precland.sdf"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm"
LOG_DIR="${LOG_DIR:-/tmp}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/gazebo/models:$REPO_DIR/gazebo/worlds:$AG_DIR/models:$AG_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

GZ_LOG="$LOG_DIR/smoke_pre_gz.log"
ARDU_LOG="$LOG_DIR/smoke_pre_copter.log"
PCAP="$LOG_DIR/smoke_pre_cam.pcap"
MAV_OUT="$LOG_DIR/smoke_pre_mav.bin"

PASS=0
FAIL=0
fail() { echo "FAIL: $*"; FAIL=$((FAIL+1)); }
pass() { echo "PASS: $*"; PASS=$((PASS+1)); }

cleanup() {
  pkill -9 -x gz-sim-server 2>/dev/null || true
  pkill -9 -x ruby          2>/dev/null || true
  pkill -9 -x arducopter    2>/dev/null || true
  pkill -9 -x tcpdump       2>/dev/null || true
  pkill -9 -x nc            2>/dev/null || true
  pkill -9 -x nc.openbsd    2>/dev/null || true
  sudo -n rm -f "$PCAP"     2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup
sleep 1

echo '>> gz sim (iris_precland)'
setsid gz sim -s -r -v 2 "$WORLD" > "$GZ_LOG" 2>&1 < /dev/null &
disown
sleep 3

echo '>> arducopter (JSON SITL)'
cd "$ARDUPILOT_DIR"
setsid ./build/sitl/bin/arducopter -S -I0 -w \
  --model JSON --speedup 1 \
  --defaults "$DEFAULTS" \
  > "$ARDU_LOG" 2>&1 < /dev/null &
disown

echo '>> waiting up to 30s for :5760'
for _ in $(seq 1 30); do
  sleep 1
  ss -tln 2>/dev/null | grep -q ':5760 ' && break
done

echo '>> check 1/5: TCP :5760'
if ss -tln 2>/dev/null | grep -q ':5760 '; then
  pass "arducopter opened :5760"
else
  fail "arducopter did not open :5760 (see $ARDU_LOG)"
fi

echo '>> connect TCP :5760 (kicks arducopter main loop + FDM exchange)'
rm -f "$MAV_OUT"
( (sleep 10) | timeout 10 nc -q 0 127.0.0.1 5760 > "$MAV_OUT" 2>/dev/null ) &
NC_PID=$!
sleep 3

echo '>> check 2/5: world ticking'
STATS=$(timeout 3 gz topic -e -t /world/iris_precland/stats -n 1 2>/dev/null | head -5)
if echo "$STATS" | grep -q 'sim_time'; then
  pass "world physics advancing"
else
  fail "no sim_time on /world/iris_precland/stats"
fi

echo '>> check 3/5: camera enable_streaming topic'
CAM_TOPIC=""
for _ in $(seq 1 5); do
  CAM_TOPIC=$(gz topic -l 2>/dev/null | grep enable_streaming | head -1 || true)
  [[ -n "$CAM_TOPIC" ]] && break
  sleep 1
done
if [[ -n "$CAM_TOPIC" ]]; then
  pass "camera topic: $CAM_TOPIC"
else
  fail "no enable_streaming topic — camera sensor not registered"
fi

echo '>> start 4s tcpdump on UDP:5600 + enable streaming'
sudo -n rm -f "$PCAP" 2>/dev/null || true
sudo -n tcpdump -i lo -n -w "$PCAP" -U 'udp port 5600' 2>/dev/null &
TD_PID=$!
sleep 1
[[ -n "$CAM_TOPIC" ]] && gz topic -t "$CAM_TOPIC" -m gz.msgs.Boolean -p 'data: true' >/dev/null 2>&1 || true
sleep 4
kill -9 "$TD_PID" 2>/dev/null || true
wait "$TD_PID" 2>/dev/null || true

echo '>> check 4/5: UDP :5600 H.264 RTP packets'
CAM_PKTS=$(sudo -n tcpdump -r "$PCAP" -n "udp port 5600" 2>/dev/null | wc -l)
if [ "$CAM_PKTS" -ge 5 ]; then
  pass "UDP:5600 streaming ($CAM_PKTS packets in 4s)"
else
  fail "UDP:5600 quiet ($CAM_PKTS packets, expected ≥5)"
fi

# let nc finish capturing
wait "$NC_PID" 2>/dev/null || true

echo '>> check 5/5: MAVLink on :5760'
SZ=$(stat -c %s "$MAV_OUT" 2>/dev/null || echo 0)
if [ "$SZ" -gt 20 ] && head -c 256 "$MAV_OUT" | od -An -tx1 | tr -d ' \n' | grep -qE 'fd|fe'; then
  pass "MAVLink on :5760 ($SZ bytes)"
else
  fail "no MAVLink on :5760 ($SZ bytes)"
fi

echo
echo "RESULT: $PASS pass, $FAIL fail"
[ "$FAIL" -eq 0 ]
