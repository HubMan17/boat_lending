#!/bin/bash
# Start headless Gazebo iris_runway.sdf, enable camera streaming,
# open gst camera viewer. Usage: ./run_iris_sim.sh
set -e

# ardupilot_gazebo plugin + model paths (bashrc not sourced by non-interactive bash)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

WORLD='iris_runway.sdf'
GZ_LOG='/tmp/gz_sim.log'

echo '>> killing stale gz sim / gst'
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f 'gst-launch-1.0.*5600' 2>/dev/null || true
sleep 1

echo '>> starting gz sim headless (bg, log -> '"$GZ_LOG"')'
rm -f "$GZ_LOG"
nohup stdbuf -oL -eL gz sim -s -r -v 2 "$WORLD" > "$GZ_LOG" 2>&1 &
GZ_PID=$!
echo "   gz_pid=$GZ_PID"

echo '>> waiting up to 20s for camera sensor to appear'
for i in $(seq 1 20); do
  sleep 1
  if gz topic -l 2>/dev/null | grep -q enable_streaming; then
    echo "   ready after ${i}s"
    break
  fi
done

echo '>> enabling camera streaming'
CAM_TOPIC=$(gz topic -l 2>/dev/null | grep enable_streaming | head -1)
if [ -z "$CAM_TOPIC" ]; then
  echo '   ERROR: no enable_streaming topic found — gz sim may still be initializing'
  echo '   check /tmp/gz_sim.log for errors'
  exit 1
fi
echo "   topic: $CAM_TOPIC"
gz topic -t "$CAM_TOPIC" -m gz.msgs.Boolean -p 'data: true'

echo '>> starting gst camera viewer (Ctrl-C here closes only the viewer,'
echo '   gz sim keeps running in bg; use stop_sim.sh to fully stop)'
gst-launch-1.0 udpsrc port=5600 \
  caps='application/x-rtp,encoding-name=H264,payload=96' ! \
  rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
