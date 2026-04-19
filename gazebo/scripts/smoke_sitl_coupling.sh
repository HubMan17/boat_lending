#!/usr/bin/env bash
# smoke_sitl_coupling.sh — headless smoke test for the SITL ↔ Gazebo UDP loop.
#
# Brings up gz sim (iris_coupling.sdf), arducopter (JSON SITL), a short-lived
# TCP client on :5760 (so arducopter ticks), and verifies:
#   1. physics is advancing (world stats tick)
#   2. bidirectional UDP traffic on 9002/9003
#   3. SITL logs show FDM fields parsed
#   4. arducopter emits MAVLink on :5760
#
# Exits 0 if all four pass, non-zero otherwise. No GCS, no Mission Planner.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/boat_lending/ardupilot}"
AG_DIR="${AG_DIR:-$HOME/ardupilot_gazebo}"
WORLD="$REPO_DIR/gazebo/worlds/iris_coupling.sdf"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm"
LOG_DIR="${LOG_DIR:-/tmp}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$AG_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$AG_DIR/models:$AG_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

GZ_LOG="$LOG_DIR/smoke_gz.log"
ARDU_LOG="$LOG_DIR/smoke_copter.log"
PCAP="$LOG_DIR/smoke_coupling.pcap"
MAV_OUT="$LOG_DIR/smoke_mav.bin"

PASS=0
FAIL=0
fail() { echo "FAIL: $*"; FAIL=$((FAIL+1)); }
pass() { echo "PASS: $*"; PASS=$((PASS+1)); }

cleanup() {
  pkill -9 -x gz-sim-server 2>/dev/null || true
  pkill -9 -x ruby          2>/dev/null || true
  pkill -9 -x arducopter    2>/dev/null || true
  pkill -9 -x tcpdump       2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup
sleep 1

echo '>> gz sim'
setsid gz sim -s -r -v 2 "$WORLD" > "$GZ_LOG" 2>&1 < /dev/null &
disown
sleep 5

echo '>> check 1/4: world ticking'
if timeout 3 gz topic -e -t /world/iris_coupling/stats -n 1 2>/dev/null | grep -q 'sim_time'; then
  pass "world physics advancing"
else
  fail "no sim_time on /world/iris_coupling/stats (sim stuck)"
fi

echo '>> arducopter (JSON SITL)'
cd "$ARDUPILOT_DIR"
setsid ./build/sitl/bin/arducopter -S -I0 -w \
  --model JSON --speedup 1 \
  --defaults "$DEFAULTS" \
  > "$ARDU_LOG" 2>&1 < /dev/null &
disown

echo '>> waiting for :5760'
for _ in $(seq 1 30); do
  sleep 1
  ss -tln 2>/dev/null | grep -q ':5760 ' && break
done

echo '>> start 5s tcpdump'
rm -f "$PCAP"
sudo -n tcpdump -i lo -n -w "$PCAP" -U 'udp and (port 9002 or port 9003)' 2>/dev/null &
TD_PID=$!
sleep 1

echo '>> connect TCP :5760 for 6s (kicks SITL main loop)'
rm -f "$MAV_OUT"
( (sleep 6) | timeout 6 nc -q 0 127.0.0.1 5760 > "$MAV_OUT" 2>/dev/null ) &
sleep 7
kill -9 "$TD_PID" 2>/dev/null || true
wait "$TD_PID" 2>/dev/null || true

echo '>> check 2/4: bidirectional UDP on 9002/9003'
# Each direction should have > 10 pkts
OUT_CNT=$(sudo -n tcpdump -r "$PCAP" -n "src host 127.0.0.1 and dst port 9002" 2>/dev/null | wc -l)
IN_CNT=$(sudo -n tcpdump -r "$PCAP" -n "src port 9002 and dst host 127.0.0.1" 2>/dev/null | wc -l)
if [ "$OUT_CNT" -gt 10 ] && [ "$IN_CNT" -gt 10 ]; then
  pass "UDP 9002 bidi traffic (servo→plugin: $OUT_CNT pkts, fdm←plugin: $IN_CNT pkts)"
else
  fail "expected >10 pkts each way, got servo→plugin=$OUT_CNT fdm←plugin=$IN_CNT"
fi

echo '>> check 3/4: SITL parsed FDM'
if grep -qE 'Forcing use_time_sync|position|quaternion|velocity' "$ARDU_LOG"; then
  pass "arducopter parsed FDM fields"
else
  fail "arducopter log does not show FDM field parse"
fi

echo '>> check 4/4: MAVLink on :5760'
SZ=$(stat -c %s "$MAV_OUT" 2>/dev/null || echo 0)
# MAVLink v2 starts with 0xFD
if [ "$SZ" -gt 20 ] && head -c 256 "$MAV_OUT" | od -An -tx1 | tr -d ' \n' | grep -qE 'fd|fe'; then
  pass "arducopter emits MAVLink on :5760 ($SZ bytes in 6s)"
else
  fail "no MAVLink-like bytes on :5760 ($SZ bytes)"
fi

echo
echo "RESULT: $PASS pass, $FAIL fail"
[ "$FAIL" -eq 0 ]
