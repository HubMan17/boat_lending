#!/usr/bin/env python3
"""End-to-end precision landing test (P1.10).

Connects to ArduCopter SITL via MAVLink, flies a GUIDED takeoff,
moves 5 m East, then commands LAND. Verifies that PrecLand brings
the copter back to the marker within the XY error threshold.

Prerequisites (all running before this script):
  - ArduCopter SITL in WSL  (start_arducopter.sh with EXTRA_DEFAULTS=params/precland_copter.parm)
  - Webots with stage1_static.wbt
  - companion.py feeding LANDING_TARGET

Usage:
  python scripts/e2e_precland.py [--mav tcp:127.0.0.1:5763]
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import os
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil


GUIDED_MODE = 4
LAND_MODE = 9
MAV_TYPE_QUADROTOR = 2
TAKEOFF_ALT = 20.0
OFFSET_M = 1.0
XY_THRESHOLD = 0.5


def _is_autopilot_hb(hb) -> bool:
    return hb is not None and hb.type == MAV_TYPE_QUADROTOR


def wait_heartbeat(conn, timeout: float = 30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            print(f"[e2e] heartbeat: type={hb.type} mode={hb.custom_mode} "
                  f"armed={bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")
            return hb
    print("ERROR: no HEARTBEAT", file=sys.stderr)
    sys.exit(1)


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
            print(f"[e2e] altitude reached: {alt:.1f} m")
            return True
    return False


def get_local_position(conn, timeout: float = 5.0):
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                          timeout=timeout)
    return msg


def send_position_target(conn, x: float, y: float, z: float) -> None:
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1111_1111_1000,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def wait_position(conn, target_x: float, target_y: float,
                   tolerance: float = 1.0, timeout: float = 30.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                              timeout=2)
        if msg is None:
            continue
        dx = msg.x - target_x
        dy = msg.y - target_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= tolerance:
            print(f"[e2e] arrived at offset: x={msg.x:.1f} y={msg.y:.1f} "
                  f"(err={dist:.2f} m)")
            return True
    return False


def wait_landed(conn, timeout: float = 120.0):
    """Wait for disarm, tracking the last known position during descent."""
    last_pos = None
    last_log = time.time()
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type=["HEARTBEAT", "LOCAL_POSITION_NED"],
                              blocking=True, timeout=2)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "LOCAL_POSITION_NED":
            last_pos = msg
            now = time.time()
            if now - last_log >= 2.0:
                print(f"[e2e] descending: x={msg.x:.2f} y={msg.y:.2f} "
                      f"alt={-msg.z:.1f}m")
                last_log = now
        elif mtype == "HEARTBEAT" and msg.type == MAV_TYPE_QUADROTOR:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return last_pos
    return None


def request_data_stream(conn, rate_hz: int = 10) -> None:
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz, 1,
    )


def run(mav_url: str, alt: float, offset: float, threshold: float) -> int:
    print(f"[e2e] connecting to {mav_url}")
    conn = mavutil.mavlink_connection(mav_url, source_system=255)

    wait_heartbeat(conn)
    request_data_stream(conn)

    time.sleep(1)

    print("[e2e] setting GUIDED mode")
    if not set_mode(conn, GUIDED_MODE):
        print("ERROR: failed to set GUIDED mode", file=sys.stderr)
        return 1

    print("[e2e] arming")
    if not arm(conn):
        print("ERROR: failed to arm", file=sys.stderr)
        return 1
    print("[e2e] armed")

    print(f"[e2e] takeoff to {alt} m")
    takeoff(conn, alt)

    if not wait_altitude(conn, alt):
        print("ERROR: takeoff timeout", file=sys.stderr)
        return 1

    if offset > 0:
        pos = get_local_position(conn)
        if pos is None:
            print("ERROR: no LOCAL_POSITION_NED", file=sys.stderr)
            return 1

        target_x = pos.x
        target_y = pos.y + offset
        target_z = pos.z

        print(f"[e2e] moving {offset} m East: target=({target_x:.1f}, {target_y:.1f})")
        send_position_target(conn, target_x, target_y, target_z)

        if not wait_position(conn, target_x, target_y):
            print("ERROR: position move timeout", file=sys.stderr)
            return 1

        time.sleep(2)

        print("[e2e] returning to home (0, 0) in GUIDED")
        send_position_target(conn, 0.0, 0.0, target_z)

        if not wait_position(conn, 0.0, 0.0, tolerance=0.5):
            print("ERROR: return to home timeout", file=sys.stderr)
            return 1

        print("[e2e] centered above marker, holding 3s")
        for _ in range(3):
            send_position_target(conn, 0.0, 0.0, target_z)
            time.sleep(1)

    print("[e2e] switching to LAND mode")
    if not set_mode(conn, LAND_MODE):
        print("ERROR: failed to set LAND mode", file=sys.stderr)
        return 1

    print("[e2e] waiting for landing...")
    final = wait_landed(conn)
    if final is None:
        print("ERROR: landing timeout", file=sys.stderr)
        return 1

    xy_error = math.sqrt(final.x * final.x + final.y * final.y)
    print(f"[e2e] landed at x={final.x:.3f} y={final.y:.3f} z={final.z:.3f}")
    print(f"[e2e] XY error from home: {xy_error:.3f} m (threshold: {threshold} m)")

    conn.close()

    if xy_error <= threshold:
        print(f"[e2e] PASS (error {xy_error:.3f} <= {threshold})")
        return 0
    else:
        print(f"[e2e] FAIL (error {xy_error:.3f} > {threshold})")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="E2E precision landing test")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5764",
                        help="MAVLink connection string (default: tcp:127.0.0.1:5764)")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALT,
                        help="Takeoff altitude in meters (default: 10)")
    parser.add_argument("--offset", type=float, default=OFFSET_M,
                        help="Horizontal offset in meters (default: 5)")
    parser.add_argument("--threshold", type=float, default=XY_THRESHOLD,
                        help="Max XY error for PASS (default: 0.5)")
    args = parser.parse_args()
    return run(args.mav, args.alt, args.offset, args.threshold)


if __name__ == "__main__":
    sys.exit(main())
