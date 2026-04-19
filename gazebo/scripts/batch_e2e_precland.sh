#!/usr/bin/env bash
# batch_e2e_precland.sh — run the Gazebo e2e precision landing test N times,
# restarting the sim for each run so state leaks don't mask regressions.
# Reports how many runs passed the XY threshold.
#
# Usage:
#   bash gazebo/scripts/batch_e2e_precland.sh [N]   (default N=10)
#
# Requires run_iris_sim.sh to be fully wired (P3b.3/P3b.4 complete) and
# EXTRA_DEFAULTS=params/precland_copter.parm so PLND_* overlay is live.
#
# Logs per run: /tmp/e2e_runN.log, /tmp/gt_runN.csv, /tmp/arducopter.log
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

N="${1:-10}"
THRESHOLD="${THRESHOLD:-0.5}"
ALT="${ALT:-10}"
MARKER_X="${MARKER_X:-5.0}"
MARKER_Y="${MARKER_Y:-0.0}"
OFFSET="${OFFSET:-0}"
CAM_FOV="${CAM_FOV:-1.2}"
LOG_ROOT="${LOG_ROOT:-/tmp}"

SUMMARY="$LOG_ROOT/batch_e2e_precland.summary"
: > "$SUMMARY"

passes=0
fails=0
errors=()

kill_sim() {
  pkill -9 -x gz-sim-server   2>/dev/null || true
  pkill -9 -x ruby            2>/dev/null || true
  pkill -9 -x arducopter      2>/dev/null || true
  pkill -9 -f sitl_udp_bridge 2>/dev/null || true
  pkill -9 -f 'gst-launch-1.0.*udpsrc.*5600' 2>/dev/null || true
  pkill -9 -f 'run_iris_sim'  2>/dev/null || true
  sleep 2
}

trap 'kill_sim; exit 130' INT TERM

echo "batch_e2e_precland: N=$N marker=($MARKER_X,$MARKER_Y) alt=$ALT offset=$OFFSET threshold=$THRESHOLD"
echo "logs root    : $LOG_ROOT"
echo

for i in $(seq 1 "$N"); do
  echo "=========================================="
  echo "RUN $i / $N"
  echo "=========================================="

  kill_sim

  SIM_LOG="$LOG_ROOT/sim_run${i}.log"
  E2E_LOG="$LOG_ROOT/e2e_run${i}.log"
  GT_CSV="$LOG_ROOT/gt_run${i}.csv"

  EXTRA_DEFAULTS="${EXTRA_DEFAULTS:-params/precland_copter.parm}" \
    setsid bash "$REPO_DIR/gazebo/scripts/run_iris_sim.sh" \
      > "$SIM_LOG" 2>&1 < /dev/null &
  SIM_PID=$!
  echo "sim pgid=$(ps -o pgid= $SIM_PID | tr -d ' '); log=$SIM_LOG"

  echo "waiting up to 60s for TCP :5765 (serial3)"
  ready=0
  for _ in $(seq 1 60); do
    sleep 1
    if ss -tln 2>/dev/null | grep -q ':5765 '; then
      ready=1
      break
    fi
  done
  if [[ $ready -eq 0 ]]; then
    echo "FAIL: sim not ready in 60s (see $SIM_LOG)"
    fails=$((fails+1))
    errors+=("run $i: sim not ready")
    echo "run=$i status=sim-not-ready xy=n/a" >> "$SUMMARY"
    kill_sim
    continue
  fi
  sleep 3

  python3 "$REPO_DIR/scripts/e2e_precland.py" \
    --mav tcp:127.0.0.1:5765 \
    --marker-x "$MARKER_X" --marker-y "$MARKER_Y" \
    --alt "$ALT" --offset "$OFFSET" --threshold "$THRESHOLD" \
    --companion --camera-backend gstreamer --cam-fov "$CAM_FOV" \
    --companion-mav tcp:127.0.0.1:5763 \
    --gt-csv "$GT_CSV" \
    > "$E2E_LOG" 2>&1
  ec=$?

  xy_line=$(grep -E '^\[e2e\] XY error:' "$E2E_LOG" | tail -1 || true)
  xy_val=$(echo "$xy_line" | awk '{for (i=1;i<=NF;i++) if ($i+0>0 || $i=="0.000"){print $i; exit}}' || true)
  if [[ -z "$xy_val" ]]; then
    xy_val=$(echo "$xy_line" | grep -oE '[0-9]+\.[0-9]+' | head -1 || echo "n/a")
  fi

  if [[ $ec -eq 0 ]]; then
    passes=$((passes+1))
    echo "run=$i status=PASS xy=$xy_val" >> "$SUMMARY"
    echo "RUN $i: PASS (xy=$xy_val)"
  else
    fails=$((fails+1))
    echo "run=$i status=FAIL xy=$xy_val exit=$ec" >> "$SUMMARY"
    errors+=("run $i: exit=$ec xy=$xy_val")
    echo "RUN $i: FAIL (exit=$ec xy=$xy_val, tail of log:)"
    tail -10 "$E2E_LOG" || true
  fi

  kill_sim
  sleep 1
done

kill_sim

echo
echo "=========================================="
echo "SUMMARY"
echo "=========================================="
echo "passed: $passes / $N"
echo "failed: $fails / $N"
if ((fails > 0)); then
  printf "   %s\n" "${errors[@]}"
fi
echo
echo "per-run: $SUMMARY"
if [[ -s "$SUMMARY" ]]; then
  mean=$(awk -F'xy=' '/status=PASS/ {s+=$2; c++} END {if (c>0) printf "%.3f", s/c; else print "n/a"}' "$SUMMARY")
  maxv=$(awk -F'xy=' '/status=PASS/ {if ($2+0>m) m=$2+0} END {if (m>0) printf "%.3f", m; else print "n/a"}' "$SUMMARY")
  echo "pass XY: mean=$mean max=$maxv"
fi

if ((passes == N)); then
  echo "RESULT: 10/10 PASS"
  exit 0
fi
exit 1
