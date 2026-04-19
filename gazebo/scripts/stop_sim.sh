#!/bin/bash
# Kill all gz sim + gst camera viewer processes.
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
pkill -9 -f 'gst-launch-1.0.*5600' 2>/dev/null || true
sleep 1
echo '>> alive procs:'
pgrep -af 'gz sim|gst-launch' || echo '   none'
