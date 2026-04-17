#!/usr/bin/env python3
"""GPS-denied hover smoke test (P2.3).

Connects to ArduCopter SITL, takes off in GUIDED, switches to LOITER,
hovers for 30 seconds, and verifies position drift <= threshold.

Prerequisites (all running before this script):
  - ArduCopter SITL in WSL  (EXTRA_DEFAULTS=params/precland_copter.parm,params/gps_denied.parm)
  - Webots with stage1_static.wbt
  - companion.py --stage 2 feeding OPTICAL_FLOW + DISTANCE_SENSOR

Usage:
  python scripts/smoke_gps_denied_hover.py [--mav tcp:127.0.0.1:5764]
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
LOITER_MODE = 5
FLOWHOLD_MODE = 22
LAND_MODE = 9
MAV_TYPE_QUADROTOR = 2

TAKEOFF_ALT = 20.0
HOVER_DURATION = 15
MOVE_SPEED = 1.5
DRIFT_THRESHOLD = 1.0


def _is_autopilot_hb(hb) -> bool:
    return hb is not None and hb.type == MAV_TYPE_QUADROTOR


def wait_heartbeat(conn, timeout: float = 30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            print(f"[hover] heartbeat: type={hb.type} mode={hb.custom_mode}")
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
            print(f"[hover] altitude reached: {alt:.1f} m")
            return True
    return False


def request_data_stream(conn, rate_hz: int = 10) -> None:
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz, 1,
    )


def monitor_hover(conn, duration: float, threshold: float) -> tuple[bool, float]:
    """Monitor position during hover, return (pass, max_drift)."""
    pos = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if pos is None:
        print("ERROR: no LOCAL_POSITION_NED at hover start", file=sys.stderr)
        return False, 999.0

    ref_x, ref_y = pos.x, pos.y
    print(f"[hover] reference position: x={ref_x:.2f} y={ref_y:.2f} alt={-pos.z:.1f}")

    max_drift = 0.0
    start = time.time()
    last_log = start
    samples = 0

    while time.time() - start < duration:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue

        dx = msg.x - ref_x
        dy = msg.y - ref_y
        drift = math.sqrt(dx * dx + dy * dy)
        if drift > max_drift:
            max_drift = drift
        samples += 1

        now = time.time()
        if now - last_log >= 5.0:
            elapsed = now - start
            print(f"[hover] t={elapsed:.0f}s drift={drift:.3f}m "
                  f"(max={max_drift:.3f}m) alt={-msg.z:.1f}m")
            last_log = now

    print(f"[hover] done: {samples} samples, max_drift={max_drift:.3f}m "
          f"(threshold={threshold}m)")
    return max_drift <= threshold, max_drift


def send_velocity(conn, vx: float, vy: float, vz: float = 0.0) -> None:
    """Send NED velocity command (vx=North, vy=East, vz=Down)."""
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1111_1100_0111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0,
    )


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


def get_local_position(conn, timeout: float = 5.0):
    return conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                           timeout=timeout)


def move_smooth(conn, target_x: float, target_y: float, speed: float,
                tolerance: float = 0.3, timeout: float = 60.0) -> bool:
    """Move to target at limited speed using velocity commands."""
    deadline = time.time() + timeout
    last_log = 0.0
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                              timeout=2)
        if msg is None:
            continue

        dx = target_x - msg.x
        dy = target_y - msg.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= tolerance:
            send_velocity(conn, 0, 0)
            print(f"[hover] reached: x={msg.x:.2f} y={msg.y:.2f} "
                  f"(err={dist:.2f}m)")
            return True

        scale = speed / dist
        send_velocity(conn, dx * scale, dy * scale)

        now = time.time()
        if now - last_log >= 2.0:
            print(f"[hover] moving: x={msg.x:.2f} y={msg.y:.2f} "
                  f"dist={dist:.2f}m alt={-msg.z:.1f}m")
            last_log = now

    return False


def wait_landed(conn, timeout: float = 60.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return True
    return False


def run(mav_url: str, alt: float, duration: float, threshold: float,
        offset: float = 0.0) -> int:
    print(f"[hover] connecting to {mav_url} (timeout 10s) ...")
    conn = mavutil.mavlink_connection(mav_url, source_system=255)
    print("[hover] connected")

    wait_heartbeat(conn)
    request_data_stream(conn)
    time.sleep(1)

    print("[hover] setting GUIDED mode")
    if not set_mode(conn, GUIDED_MODE):
        print("ERROR: failed to set GUIDED mode", file=sys.stderr)
        return 1

    print("[hover] arming")
    if not arm(conn):
        print("ERROR: failed to arm", file=sys.stderr)
        return 1
    print("[hover] armed")

    print(f"[hover] takeoff to {alt} m")
    takeoff(conn, alt)

    if not wait_altitude(conn, alt):
        print("ERROR: takeoff timeout", file=sys.stderr)
        return 1

    print("[hover] staying in GUIDED (GPS-denied, optical flow velocity)")

    print(f"[hover] monitoring hover for {duration}s...")
    passed, max_drift = monitor_hover(conn, duration, threshold)

    if offset > 0:
        pos = get_local_position(conn)
        if pos is None:
            print("ERROR: no position for offset test", file=sys.stderr)
            return 1

        target_x, target_y = pos.x, pos.y + offset
        print(f"[hover] smooth move {offset}m East at {MOVE_SPEED} m/s")
        if not move_smooth(conn, target_x, target_y, MOVE_SPEED):
            print("ERROR: offset move timeout", file=sys.stderr)
            return 1

        print("[hover] holding 3s...")
        for _ in range(3):
            send_position_target(conn, target_x, target_y, pos.z)
            time.sleep(1)

        print("[hover] smooth return to origin at {0} m/s".format(MOVE_SPEED))
        if not move_smooth(conn, 0.0, 0.0, MOVE_SPEED):
            print("ERROR: return to origin timeout", file=sys.stderr)
            return 1

        final = get_local_position(conn)
        if final:
            ret_err = math.sqrt(final.x**2 + final.y**2)
            print(f"[hover] return position: x={final.x:.3f} y={final.y:.3f} "
                  f"err={ret_err:.3f}m")

    print("[hover] switching to LAND")
    if not set_mode(conn, LAND_MODE):
        print("ERROR: failed to set LAND mode", file=sys.stderr)
        return 1

    print("[hover] waiting for landing...")
    if not wait_landed(conn):
        print("WARNING: landing timeout", file=sys.stderr)

    conn.close()

    if passed:
        print(f"[hover] PASS (max_drift {max_drift:.3f} <= {threshold})")
        return 0
    else:
        print(f"[hover] FAIL (max_drift {max_drift:.3f} > {threshold})")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="GPS-denied hover smoke test")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5764",
                        help="MAVLink connection (default: tcp:127.0.0.1:5764)")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALT,
                        help="Takeoff altitude (default: 20)")
    parser.add_argument("--duration", type=float, default=HOVER_DURATION,
                        help="Hover duration in seconds (default: 30)")
    parser.add_argument("--threshold", type=float, default=DRIFT_THRESHOLD,
                        help="Max allowed drift in meters (default: 1.0)")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="Move this many meters East then return (default: 0, skip)")
    args = parser.parse_args()
    return run(args.mav, args.alt, args.duration, args.threshold, args.offset)


if __name__ == "__main__":
    sys.exit(main())
