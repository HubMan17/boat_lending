#!/bin/bash
# Grab frames from gz sim GstCameraPlugin UDP stream (port 5600) into JPEGs
# under gazebo/captures/ (gitignored). Useful for headless validation.
set -e
REPO_ROOT='/mnt/c/Users/user/Desktop/projects/boat_lending'
OUT_DIR="$REPO_ROOT/gazebo/captures"
mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/cam_frame_*.jpg
timeout 5 gst-launch-1.0 -q \
  udpsrc port=5600 caps='application/x-rtp,encoding-name=H264,payload=96' ! \
  rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! \
  multifilesink location="$OUT_DIR/cam_frame_%03d.jpg" max-files=10
ls -la "$OUT_DIR"/cam_frame_*.jpg 2>&1 | tail -5
