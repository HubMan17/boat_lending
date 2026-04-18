#!/usr/bin/env python3
"""End-to-end QuadPlane landing test (P3.6).

Connects to ArduPlane SITL via MAVLink, flies a GUIDED takeoff,
moves East by velocity commands, disconnects so companion can take
over serial2, then polls result file to check final position.

Flow:
  Phase 1: Connect serial2 -> GUIDED(15) -> arm -> takeoff -> move East -> stop
  Phase 2: Disconnect serial2 -> auto-launch companion (auto-detects ArduPlane)
  Phase 3: Poll e2e_result.txt -> validate XY error <= 0.5m

Prerequisites (all running before this script):
  - Webots with QuadPlane world
  - ArduPlane SITL via start_arduplane.sh (EXTRA_DEFAULTS=params/gps_denied.parm)

Usage:
  python scripts/e2e_quadplane.py [--mav tcp:127.0.0.1:5763] [--alt 30] [--offset 5]
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import os
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil


GUIDED_MODE = 15
QLAND_MODE = 20
MAV_TYPE_FIXED_WING = 1
TAKEOFF_ALT = 30.0
OFFSET_M = 5.0
XY_THRESHOLD = 0.5
MOVE_SPEED = 1.0
APPROACH_SPEED = 1.5
LANDING_TIMEOUT = 150.0

LOG = "[e2e-qp]"


def _is_autopilot_hb(hb) -> bool:
    return hb is not None and hb.type == MAV_TYPE_FIXED_WING


def wait_heartbeat(conn, timeout: float = 30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            print(f"{LOG} heartbeat: type={hb.type} mode={hb.custom_mode} "
                  f"armed={bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")
            return hb
    return None


def set_mode(conn, mode_id: int, timeout: float = 10.0) -> bool:
    conn.set_mode(mode_id)
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb) and hb.custom_mode == mode_id:
            return True
    return False


def arm(conn, timeout: float = 30.0) -> bool:
    conn.arducopter_arm()
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                return True
    return False


def takeoff(conn, alt: float) -> None:
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, alt,
    )


def wait_altitude(conn, target_alt: float, tolerance: float = 1.0,
                   timeout: float = 30.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                              timeout=2)
        if msg is None:
            continue
        alt = -msg.z
        if alt >= target_alt - tolerance:
            print(f"{LOG} altitude reached: {alt:.1f} m")
            return True
    return False


def request_data_stream(conn, rate_hz: int = 10) -> None:
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz, 1,
    )


def get_local_position(conn, timeout: float = 5.0):
    return conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                           timeout=timeout)


def send_velocity(conn, vx: float, vy: float, vz: float = 0.0,
                   yaw: float = 0.0) -> None:
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1001_1100_0111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        yaw, 0,
    )


def move_east(conn, offset: float, speed: float = MOVE_SPEED,
              timeout: float = 60.0) -> bool:
    pos = get_local_position(conn)
    if pos is None:
        print("ERROR: no LOCAL_POSITION_NED for move", file=sys.stderr)
        return False

    start_y = pos.y
    target_y = start_y + offset
    print(f"{LOG} moving {offset}m East at {speed} m/s (y: {start_y:.2f} -> {target_y:.2f})")

    deadline = time.time() + timeout
    last_log = 0.0
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                              timeout=2)
        if msg is None:
            continue

        remaining = target_y - msg.y
        if abs(remaining) <= 0.3:
            send_velocity(conn, 0, 0, yaw=0.0)
            print(f"{LOG} offset reached: y={msg.y:.2f} (err={abs(remaining):.2f}m)")
            return True

        vy = speed if remaining > 0 else -speed
        send_velocity(conn, 0, vy, yaw=0.0)

        now = time.time()
        if now - last_log >= 2.0:
            print(f"{LOG} moving: y={msg.y:.2f} remaining={remaining:.2f}m alt={-msg.z:.1f}m")
            last_log = now

    return False


RESULT_FILE = os.path.join(os.path.dirname(__file__), "..", "e2e_result.txt")


def read_result(path: str = RESULT_FILE):
    try:
        with open(path) as f:
            parts = f.read().strip().split()
            if len(parts) >= 2:
                return float(parts[0]), float(parts[1])
    except (FileNotFoundError, ValueError):
        pass
    return None, None


def run(mav_url: str, alt: float, offset: float, threshold: float,
        approach: bool = False, approach_speed: float = APPROACH_SPEED) -> int:
    print(f"{LOG} connecting to {mav_url}")
    conn = mavutil.mavlink_connection(mav_url, source_system=255)

    hb = wait_heartbeat(conn)
    if hb is None:
        print("ERROR: no HEARTBEAT", file=sys.stderr)
        return 1
    request_data_stream(conn)
    time.sleep(1)

    print(f"{LOG} setting GUIDED mode (mode={GUIDED_MODE})")
    if not set_mode(conn, GUIDED_MODE):
        print("ERROR: failed to set GUIDED mode", file=sys.stderr)
        return 1

    print(f"{LOG} arming")
    if not arm(conn):
        print("ERROR: failed to arm", file=sys.stderr)
        return 1
    print(f"{LOG} armed")

    print(f"{LOG} takeoff to {alt} m")
    takeoff(conn, alt)

    if not wait_altitude(conn, alt):
        print("ERROR: takeoff timeout", file=sys.stderr)
        return 1

    if offset > 0:
        if not move_east(conn, offset):
            print("ERROR: offset move timeout", file=sys.stderr)
            return 1

        if approach:
            print(f"{LOG} approach: flying toward marker at {approach_speed} m/s")
            deadline_app = time.time() + 60.0
            while time.time() < deadline_app:
                send_velocity(conn, 0, -approach_speed, yaw=0.0)
                msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
                if msg and abs(msg.y) <= 3.0:
                    print(f"{LOG} near marker zone y={msg.y:.2f}, releasing control")
                    break
                if msg and time.time() - deadline_app + 60 > 2:
                    t = time.time() - (deadline_app - 60)
                    if int(t) % 2 == 0:
                        print(f"{LOG} approaching: y={msg.y:.2f} alt={-msg.z:.1f}m")
            pos = get_local_position(conn)
            if pos:
                print(f"{LOG} released at y={pos.y:.2f} vy={pos.vy:.2f} m/s")
        else:
            print(f"{LOG} stabilizing yaw=0")
            for _ in range(20):
                send_velocity(conn, 0, 0, yaw=0.0)
                time.sleep(0.2)

    conn.close()

    if os.path.exists(RESULT_FILE):
        os.remove(RESULT_FILE)

    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    python = os.path.join(repo_dir, "companion", ".venv", "Scripts", "python.exe")
    companion_py = os.path.join(repo_dir, "companion", "companion.py")
    companion_cmd = [python, companion_py, "--stage", "2", "--track",
                     "--mav-url", mav_url,
                     "--track-max-alt", str(alt + 10)]

    print(f"{LOG} launching companion automatically...")
    companion_proc = subprocess.Popen(companion_cmd, cwd=repo_dir)
    print(f"{LOG} companion PID={companion_proc.pid}")
    print(f"{LOG} waiting for landing (timeout {LANDING_TIMEOUT}s)...")

    deadline = time.time() + LANDING_TIMEOUT
    x, y = None, None
    while time.time() < deadline:
        x, y = read_result()
        if x is not None:
            break
        time.sleep(2)

    companion_proc.terminate()
    try:
        companion_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        companion_proc.kill()

    if x is None:
        print("ERROR: companion did not land (no result file)", file=sys.stderr)
        return 1

    xy_error = math.sqrt(x * x + y * y)
    print(f"{LOG} landed at x={x:.3f} y={y:.3f}")
    print(f"{LOG} XY error from home: {xy_error:.3f} m (threshold: {threshold} m)")

    if xy_error <= threshold:
        print(f"{LOG} PASS (error {xy_error:.3f} <= {threshold})")
        return 0
    else:
        print(f"{LOG} FAIL (error {xy_error:.3f} > {threshold})")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="E2E QuadPlane landing test")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5763",
                        help="MAVLink connection string (default: tcp:127.0.0.1:5763)")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALT,
                        help="Takeoff altitude in meters (default: 30)")
    parser.add_argument("--offset", type=float, default=OFFSET_M,
                        help="East offset in meters (default: 5)")
    parser.add_argument("--threshold", type=float, default=XY_THRESHOLD,
                        help="Max XY error for PASS (default: 0.5)")
    parser.add_argument("--approach", action="store_true",
                        help="Approach mode: fly toward marker with velocity")
    parser.add_argument("--approach-speed", type=float, default=APPROACH_SPEED,
                        help="Approach speed in m/s (default: 1.5)")
    args = parser.parse_args()
    return run(args.mav, args.alt, args.offset, args.threshold,
               approach=args.approach, approach_speed=args.approach_speed)


if __name__ == "__main__":
    sys.exit(main())
