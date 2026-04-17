#!/usr/bin/env python3
"""Save a single camera frame + annotate marker detection."""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "companion"))

import cv2
import numpy as np
from camera_receive import CameraReceiver

cam = CameraReceiver("127.0.0.1", 5599)
for attempt in range(20):
    try:
        cam.connect()
        break
    except ConnectionError:
        time.sleep(1)
else:
    sys.exit("no camera")

frame = cam.read_frame()
print(f"Frame shape: {frame.shape} (h, w)")
print(f"Header w={cam.width} h={cam.height}")

color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, params)
corners, ids, _ = detector.detectMarkers(frame)

if ids is not None:
    cv2.aruco.drawDetectedMarkers(color, corners, ids)
    for i, cid in enumerate(ids.flatten()):
        c = corners[i][0]
        cx, cy = c[:, 0].mean(), c[:, 1].mean()
        print(f"Marker {cid}: center pixel ({cx:.1f}, {cy:.1f})")
        cv2.circle(color, (int(cx), int(cy)), 5, (0, 0, 255), -1)

cv2.line(color, (320, 0), (320, 480), (0, 255, 0), 1)
cv2.line(color, (0, 240), (640, 240), (0, 255, 0), 1)
cv2.putText(color, "px+ RIGHT", (500, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
cv2.putText(color, "py+ DOWN", (5, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

out = os.path.join(os.path.dirname(__file__), "..", "camera_frame.png")
cv2.imwrite(out, color)
print(f"Saved: {out}")
cam.close()
