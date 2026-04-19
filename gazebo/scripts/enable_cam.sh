#!/bin/bash
# One-shot: enable camera streaming on iris_runway camera sensor
TOPIC='/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming'
gz topic -t "$TOPIC" -m gz.msgs.Boolean -p 'data: true'
echo "enable sent to $TOPIC"
