#!/usr/bin/env python3
"""Smoke test for companion/tracker.py — PID target tracker."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "companion"))

from detector import Detection
from tracker import TargetTracker, VelocityCommand


def make_det(angle_x: float, angle_y: float) -> Detection:
    return Detection(
        marker_id=0,
        angle_x=angle_x,
        angle_y=angle_y,
        distance=5.0,
        x_body=angle_y * 5.0,
        y_body=angle_x * 5.0,
        z_body=5.0,
    )


def test_zero_error():
    tracker = TargetTracker(kp=1.0, ki=0.0, kd=0.0)
    det = make_det(0.0, 0.0)
    cmd = tracker.update(det, now=0.0)
    assert abs(cmd.vx) < 1e-6, f"vx={cmd.vx}"
    assert abs(cmd.vy) < 1e-6, f"vy={cmd.vy}"
    assert abs(cmd.vz) < 1e-6, f"vz={cmd.vz}"
    print("PASS: zero error -> zero velocity")


def test_positive_angle_x():
    tracker = TargetTracker(kp=1.0, ki=0.0, kd=0.0)
    det = make_det(angle_x=0.1, angle_y=0.0)
    cmd = tracker.update(det, now=0.0)
    assert cmd.vy > 0, f"marker right -> vy positive, got {cmd.vy}"
    assert abs(cmd.vx) < 1e-6, f"vx should be ~0, got {cmd.vx}"
    print(f"PASS: positive angle_x -> vy={cmd.vy:.3f}")


def test_positive_angle_y():
    tracker = TargetTracker(kp=1.0, ki=0.0, kd=0.0)
    det = make_det(angle_x=0.0, angle_y=0.2)
    cmd = tracker.update(det, now=0.0)
    assert cmd.vx > 0, f"marker forward -> vx positive, got {cmd.vx}"
    assert abs(cmd.vy) < 1e-6, f"vy should be ~0, got {cmd.vy}"
    print(f"PASS: positive angle_y -> vx={cmd.vx:.3f}")


def test_max_clamp():
    tracker = TargetTracker(kp=10.0, ki=0.0, kd=0.0, max_vel=0.5)
    det = make_det(angle_x=1.0, angle_y=1.0)
    cmd = tracker.update(det, now=0.0)
    assert abs(cmd.vx) <= 0.5 + 1e-6, f"vx={cmd.vx} > max"
    assert abs(cmd.vy) <= 0.5 + 1e-6, f"vy={cmd.vy} > max"
    print(f"PASS: velocity clamped to max_vel=0.5")


def test_convergence():
    tracker = TargetTracker(kp=2.0, ki=0.1, kd=0.3, max_vel=2.0)
    angle = 0.3
    dt = 0.05
    t = 0.0
    for step in range(100):
        det = make_det(angle_x=angle, angle_y=0.0)
        cmd = tracker.update(det, now=t)
        angle -= cmd.vy * dt
        t += dt
    assert abs(angle) < 0.01, f"did not converge: angle={angle:.4f}"
    print(f"PASS: convergence in 100 steps, final angle={angle:.5f}")


def test_integral_windup():
    tracker = TargetTracker(kp=0.0, ki=10.0, kd=0.0, integral_limit=0.3)
    t = 0.0
    for _ in range(200):
        det = make_det(angle_x=0.5, angle_y=0.0)
        cmd = tracker.update(det, now=t)
        t += 0.05
    assert abs(cmd.vy) <= 10.0 * 0.3 + 1e-6, (
        f"integral windup: vy={cmd.vy}")
    print(f"PASS: integral clamped, vy={cmd.vy:.3f}")


def test_reset():
    tracker = TargetTracker(kp=1.0, ki=1.0, kd=0.0)
    det = make_det(angle_x=0.5, angle_y=0.5)
    tracker.update(det, now=0.0)
    tracker.update(det, now=0.1)
    tracker.reset()
    det_zero = make_det(0.0, 0.0)
    cmd = tracker.update(det_zero, now=0.2)
    assert abs(cmd.vx) < 1e-6, f"after reset vx={cmd.vx}"
    assert abs(cmd.vy) < 1e-6, f"after reset vy={cmd.vy}"
    print("PASS: reset clears state")


if __name__ == "__main__":
    test_zero_error()
    test_positive_angle_x()
    test_positive_angle_y()
    test_max_clamp()
    test_convergence()
    test_integral_windup()
    test_reset()
    print(f"\nAll 7 tests PASSED")
