#!/usr/bin/env python3
"""PID tracker tuning e2e test (P2.6).

Takes off in GUIDED, moves to offset, switches to LOITER so companion
can acquire the marker and switch to GUIDED tracking. Monitors NED
position relative to marker (at origin) and converts to pixel error.

Prerequisites (all running before this script):
  - ArduCopter SITL in WSL  (EXTRA_DEFAULTS=params/precland_copter.parm,params/gps_denied.parm)
  - Webots with stage1_static.wbt
  - companion.py --stage 2 --track feeding OPTICAL_FLOW + tracker velocity

Usage:
  python scripts/e2e_tracker_tune.py [--alt 10] [--offset 3] [--duration 30]
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
LAND_MODE = 9
MAV_TYPE_QUADROTOR = 2

DEFAULT_ALT = 15.0
DEFAULT_OFFSET = 3.0
DEFAULT_DURATION = 40
PIXEL_THRESHOLD = 50

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FOV_RAD = 0.7854
CAM_FX = CAM_WIDTH / (2.0 * math.tan(CAM_FOV_RAD / 2.0))


def meters_to_pixels(offset_m: float, alt_m: float) -> float:
    """Convert horizontal offset (m) at given altitude to pixel displacement."""
    if alt_m < 0.1:
        return 9999.0
    return abs(offset_m) * CAM_FX / alt_m


def _is_autopilot_hb(hb) -> bool:
    return hb is not None and hb.type == MAV_TYPE_QUADROTOR


def wait_heartbeat(conn, timeout: float = 30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            print(f"[tune] heartbeat: type={hb.type} mode={hb.custom_mode}")
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
        0, 0, 0, 0, 0, 0, 0, alt,
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
            print(f"[tune] altitude reached: {alt:.1f} m")
            return True
    return False


def request_data_stream(conn, rate_hz: int = 10) -> None:
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz, 1,
    )


def send_velocity(conn, vx: float, vy: float, vz: float = 0.0,
                   yaw: float | None = None) -> None:
    if yaw is not None:
        type_mask = 0b0000_1001_1100_0111  # use vx vy vz yaw
    else:
        type_mask = 0b0000_1101_1100_0111  # use vx vy vz, no yaw
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0, vx, vy, vz, 0, 0, 0,
        yaw if yaw is not None else 0, 0,
    )


def get_heading(conn) -> float:
    """Get current heading in radians from ATTITUDE."""
    msg = conn.recv_match(type="ATTITUDE", blocking=True, timeout=2)
    if msg:
        return msg.yaw
    return 0.0


def move_smooth(conn, target_x: float, target_y: float, speed: float = 1.0,
                tolerance: float = 0.5, timeout: float = 60.0,
                yaw: float = 0.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue
        dx = target_x - msg.x
        dy = target_y - msg.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= tolerance:
            send_velocity(conn, 0, 0, yaw=yaw)
            print(f"[tune] reached offset: x={msg.x:.2f} y={msg.y:.2f}")
            return True
        scale = speed / dist
        send_velocity(conn, dx * scale, dy * scale, yaw=yaw)
    return False


def wait_landed(conn, timeout: float = 60.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if _is_autopilot_hb(hb):
            if not bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
    return False


def monitor_tracking(conn, duration: float, alt: float,
                     pixel_threshold: float) -> dict:
    """Monitor position during tracking, compute pixel-equivalent error stats."""
    errors_m = []
    errors_px = []
    start = time.time()
    last_log = start

    while time.time() - start < duration:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue

        cur_alt = -msg.z
        if cur_alt < 0.5:
            continue

        err_north = msg.x
        err_east = msg.y
        err_m = math.sqrt(err_north**2 + err_east**2)
        err_px = meters_to_pixels(err_m, cur_alt)

        errors_m.append(err_m)
        errors_px.append(err_px)

        now = time.time()
        if now - last_log >= 5.0:
            elapsed = now - start
            print(f"[tune] t={elapsed:.0f}s  err={err_m:.3f}m  "
                  f"px={err_px:.0f}  alt={cur_alt:.1f}m  "
                  f"N={err_north:.3f} E={err_east:.3f}")
            last_log = now

    if not errors_px:
        return {"pass": False, "reason": "no samples"}

    mean_px = sum(errors_px) / len(errors_px)
    max_px = max(errors_px)
    p95_px = sorted(errors_px)[int(len(errors_px) * 0.95)]
    mean_m = sum(errors_m) / len(errors_m)
    max_m = max(errors_m)

    return {
        "pass": p95_px <= pixel_threshold,
        "samples": len(errors_px),
        "mean_m": mean_m,
        "max_m": max_m,
        "mean_px": mean_px,
        "max_px": max_px,
        "p95_px": p95_px,
    }


def run(mav_url: str, alt: float, offset: float, duration: float,
        pixel_threshold: float) -> int:
    print(f"[tune] connecting to {mav_url} ...")
    conn = mavutil.mavlink_connection(mav_url, source_system=255)

    wait_heartbeat(conn)
    request_data_stream(conn)
    time.sleep(1)

    print("[tune] GUIDED mode")
    if not set_mode(conn, GUIDED_MODE):
        print("ERROR: failed to set GUIDED", file=sys.stderr)
        return 1

    print("[tune] arming")
    if not arm(conn):
        print("ERROR: failed to arm", file=sys.stderr)
        return 1

    print(f"[tune] takeoff to {alt} m")
    takeoff(conn, alt)
    if not wait_altitude(conn, alt):
        print("ERROR: takeoff timeout", file=sys.stderr)
        return 1

    print("[tune] stabilizing yaw=0°")
    for _ in range(10):
        send_velocity(conn, 0, 0, yaw=0.0)
        time.sleep(0.2)

    if offset > 0:
        print(f"[tune] moving {offset}m East (away from marker)")
        if not move_smooth(conn, 0, offset, yaw=0.0):
            print("ERROR: offset move timeout", file=sys.stderr)
            return 1

    print("[tune] holding position in GUIDED (yaw=0°)")
    for _ in range(10):
        send_velocity(conn, 0, 0, yaw=0.0)
        time.sleep(0.5)
    print("[tune] stopped sending velocity — companion controls now")

    print(f"[tune] monitoring tracking for {duration}s "
          f"(threshold: {pixel_threshold} px)...")
    stats = monitor_tracking(conn, duration, alt, pixel_threshold)

    print("\n--- Tracker Tuning Results ---")
    if "reason" in stats:
        print(f"FAIL: {stats['reason']}")
        return 1

    print(f"  Samples:   {stats['samples']}")
    print(f"  Error (m): mean={stats['mean_m']:.3f}  max={stats['max_m']:.3f}")
    print(f"  Error (px): mean={stats['mean_px']:.0f}  max={stats['max_px']:.0f}  "
          f"p95={stats['p95_px']:.0f}")
    print(f"  Threshold: {pixel_threshold} px (p95)")

    print("[tune] switching to LAND")
    set_mode(conn, LAND_MODE)
    wait_landed(conn)
    conn.close()

    if stats["pass"]:
        print(f"\n[tune] PASS (p95 {stats['p95_px']:.0f} <= {pixel_threshold} px)")
        return 0
    else:
        print(f"\n[tune] FAIL (p95 {stats['p95_px']:.0f} > {pixel_threshold} px)")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="PID tracker tuning e2e test")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5764")
    parser.add_argument("--alt", type=float, default=DEFAULT_ALT,
                        help="Hover altitude (default: 10)")
    parser.add_argument("--offset", type=float, default=DEFAULT_OFFSET,
                        help="Initial East offset from marker (default: 3)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Tracking monitor duration in seconds (default: 30)")
    parser.add_argument("--threshold", type=float, default=PIXEL_THRESHOLD,
                        help="Max allowed p95 pixel error (default: 50)")
    args = parser.parse_args()
    return run(args.mav, args.alt, args.offset, args.duration, args.threshold)


if __name__ == "__main__":
    sys.exit(main())
