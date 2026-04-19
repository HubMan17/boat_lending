#!/usr/bin/env python3
"""End-to-end QuadPlane landing test (P3.6).

Takeoff in GUIDED, switch to QLAND, companion sends LANDING_TARGET
for PrecLand correction, wait for disarm, validate final XY position.

Flow:
  1. Connect serial2 -> GUIDED(15) -> arm -> takeoff to alt
  2. Start companion on serial3 (stage 1, LANDING_TARGET sender)
  3. Set QLAND mode. QLAND.run() == QLOITER.run() so PrecLand corrects
     lateral position from LANDING_TARGET messages during descent.
  4. Poll HEARTBEAT until disarm, capture final LOCAL_POSITION_NED.
  5. Validate |xy| <= threshold (default 0.5m).

Prerequisites:
  - Webots with stage3_quadplane.wbt (drone + marker visible)
  - ArduPlane SITL via ./scripts/start_arduplane.sh (default quadplane.parm
    now has PLND_ENABLED=1, RNGFND1_TYPE=10)

Usage:
  python scripts/e2e_quadplane.py [--mav tcp:127.0.0.1:5763] [--alt 30]
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil


GUIDED_MODE = 15
QLAND_MODE = 20
MAV_TYPE_FIXED_WING = 1
TAKEOFF_ALT = 30.0
XY_THRESHOLD = 0.5
LANDING_TIMEOUT = 180.0
COMPANION_MAV = "tcp:127.0.0.1:5764"

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
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
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


def wait_disarm(conn, timeout: float) -> bool:
    deadline = time.time() + timeout
    last_log = 0.0
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print(f"{LOG} disarmed")
                return True
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is not None and time.time() - last_log >= 3.0:
            print(f"{LOG} descending: x={msg.x:.2f} y={msg.y:.2f} alt={-msg.z:.1f}m")
            last_log = time.time()
    return False


def get_final_position(conn, timeout: float = 5.0):
    last = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if msg is not None:
            last = msg
    return last


def run(mav_url: str, alt: float, threshold: float,
        marker_x: float, marker_y: float) -> int:
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    python = os.path.join(repo_dir, "companion", ".venv", "Scripts", "python.exe")
    companion_py = os.path.join(repo_dir, "companion", "companion.py")

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

    companion_cmd = [python, companion_py, "--stage", "1",
                     "--mav-url", COMPANION_MAV]
    print(f"{LOG} launching companion (stage 1, LANDING_TARGET) on {COMPANION_MAV}")
    companion_proc = subprocess.Popen(companion_cmd, cwd=repo_dir)
    print(f"{LOG} companion PID={companion_proc.pid}, waiting 3s for detection")
    time.sleep(3)

    print(f"{LOG} setting QLAND mode (mode={QLAND_MODE})")
    if not set_mode(conn, QLAND_MODE):
        print("ERROR: failed to set QLAND mode", file=sys.stderr)
        companion_proc.terminate()
        return 1

    print(f"{LOG} waiting for disarm (timeout {LANDING_TIMEOUT}s)...")
    if not wait_disarm(conn, LANDING_TIMEOUT):
        print("ERROR: disarm timeout", file=sys.stderr)
        companion_proc.terminate()
        return 1

    time.sleep(1)
    final = get_final_position(conn)

    companion_proc.terminate()
    try:
        companion_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        companion_proc.kill()

    if final is None:
        print("ERROR: no LOCAL_POSITION_NED on landing", file=sys.stderr)
        return 1

    dx = final.x - marker_x
    dy = final.y - marker_y
    xy_error = math.sqrt(dx ** 2 + dy ** 2)
    print(f"{LOG} landed at x={final.x:.3f} y={final.y:.3f} alt={-final.z:.2f}m")
    print(f"{LOG} marker at x={marker_x:.3f} y={marker_y:.3f}")
    print(f"{LOG} XY error from marker: {xy_error:.3f} m (threshold: {threshold} m)")

    if xy_error <= threshold:
        print(f"{LOG} PASS (error {xy_error:.3f} <= {threshold})")
        return 0
    print(f"{LOG} FAIL (error {xy_error:.3f} > {threshold})")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="E2E QuadPlane landing test (P3.6)")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5763",
                        help="MAVLink connection (default: tcp:127.0.0.1:5763)")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALT,
                        help="Takeoff altitude (default: 30)")
    parser.add_argument("--threshold", type=float, default=XY_THRESHOLD,
                        help="Max XY error from marker for PASS (default: 0.5)")
    parser.add_argument("--marker-x", type=float, default=0.0,
                        help="Marker NED x (north) in metres (default: 0)")
    parser.add_argument("--marker-y", type=float, default=0.0,
                        help="Marker NED y (east) in metres (default: 0)")
    args = parser.parse_args()
    return run(args.mav, args.alt, args.threshold, args.marker_x, args.marker_y)


if __name__ == "__main__":
    sys.exit(main())
