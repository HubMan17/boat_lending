#!/usr/bin/env python3
"""Smoke test for QuadPlane.proto — verifies model loads correctly in Webots.

Run from Webots as the controller of a QuadPlane node, or as an extern
controller while stage3_quadplane.wbt is open.

Checks:
  - 5 motors exist (m1-m5)
  - Sensors exist (accelerometer, gyro, gps, inertial unit)
  - Camera is accessible
"""

import sys
import os

if sys.platform.startswith("win"):
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "C:\\Program Files\\Webots")
elif sys.platform.startswith("linux"):
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
else:
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app")

os.environ.setdefault("WEBOTS_HOME", WEBOTS_HOME)
os.environ["PYTHONIOENCODING"] = "UTF-8"
sys.path.append(f"{WEBOTS_HOME}/lib/controller/python")

from controller import Robot


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    errors = []

    motor_names = ["m1_motor", "m2_motor", "m3_motor", "m4_motor", "m5_motor"]
    for name in motor_names:
        dev = robot.getDevice(name)
        if dev is None:
            errors.append(f"Motor '{name}' not found")
        else:
            print(f"OK  motor  {name}")

    sensor_names = ["accelerometer", "gyro", "gps", "inertial unit"]
    for name in sensor_names:
        dev = robot.getDevice(name)
        if dev is None:
            errors.append(f"Sensor '{name}' not found")
        else:
            print(f"OK  sensor {name}")

    camera = robot.getDevice("camera")
    if camera is None:
        errors.append("Camera 'camera' not found")
    else:
        camera.enable(100)
        robot.step(timestep)
        w = camera.getWidth()
        h = camera.getHeight()
        print(f"OK  camera {w}x{h}")

    if errors:
        print(f"\nFAIL — {len(errors)} error(s):")
        for e in errors:
            print(f"  - {e}")
        sys.exit(1)

    print("\nPASS — QuadPlane smoke test OK")
    sys.exit(0)


if __name__ == "__main__":
    main()
