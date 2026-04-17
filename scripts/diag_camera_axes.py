#!/usr/bin/env python3
"""Diagnostic: determine camera-to-body axis mapping.

Hovers the drone at known NED positions and logs raw OpenCV tvec
from ArUco detection. Comparing tvec changes with NED position
changes reveals the correct camera→body FRD mapping.

Prerequisites: SITL + Webots + bridge running.

Sequence:
  1. Hover at (0, 0) alt=15m — baseline tvec
  2. Move 2m North (NED +x) — see which tvec axis changes
  3. Move 2m East  (NED +y) — see which tvec axis changes
  4. Return to (0, 0)

Prints a mapping table at the end.
"""
from __future__ import annotations

import math
import os
import sys
import time

os.environ["MAVLINK20"] = "1"

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "companion"))

from pymavlink import mavutil

from camera_receive import CameraReceiver
from detector import ArucoDetector, Detection

GUIDED_MODE = 4
MAV_TYPE_QUADROTOR = 2
ALT = 10.0


def wait_autopilot_hb(conn, timeout=30):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if hb and hb.type == MAV_TYPE_QUADROTOR:
            return hb
    return None


def set_mode(conn, mode_id, timeout=10):
    conn.set_mode(mode_id)
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if hb and hb.type == MAV_TYPE_QUADROTOR and hb.custom_mode == mode_id:
            return True
    return False


def arm_and_takeoff(conn, alt):
    set_mode(conn, GUIDED_MODE)
    conn.arducopter_arm()
    deadline = time.time() + 30
    while time.time() < deadline:
        hb = conn.wait_heartbeat(timeout=2)
        if hb and hb.type == MAV_TYPE_QUADROTOR:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                break

    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt,
    )

    deadline = time.time() + 40
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg and -msg.z >= alt - 1:
            return True
    return False


def send_pos(conn, x, y, z):
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1111_1111_1000,
        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0,
    )


def wait_pos(conn, tx, ty, tol=0.5, timeout=30):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg:
            dx = msg.x - tx
            dy = msg.y - ty
            if math.sqrt(dx*dx + dy*dy) < tol:
                return msg
    return None


def sample_detections(cam, detector, n=20):
    """Collect n detections, return average raw tvec + pixel center."""
    txs, tys, tzs, pxs, pys = [], [], [], [], []
    frames = 0
    while len(txs) < n and frames < 200:
        try:
            frame = cam.read_frame()
        except ConnectionError:
            break
        frames += 1
        dets = detector.detect_raw(frame)
        for mid, tx, ty, tz, px, py in dets:
            if mid == 0:
                txs.append(tx)
                tys.append(ty)
                tzs.append(tz)
                pxs.append(px)
                pys.append(py)
    if not txs:
        return None
    return (sum(txs)/len(txs), sum(tys)/len(tys), sum(tzs)/len(tzs),
            sum(pxs)/len(pxs), sum(pys)/len(pys), len(txs))


def main():
    mav_url = "tcp:127.0.0.1:5764"
    cam_host, cam_port = "127.0.0.1", 5599

    print("[diag] connecting...")
    conn = mavutil.mavlink_connection(mav_url, source_system=255)
    wait_autopilot_hb(conn)
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
    )

    detector = ArucoDetector()

    print("[diag] arming + takeoff")
    arm_and_takeoff(conn, ALT)
    print("[diag] holding position 5s...")
    for _ in range(5):
        send_pos(conn, 0.0, 0.0, -ALT)
        time.sleep(1)

    positions = [
        ("home 00", 0.0, 0.0),
        ("3m_North xNED", 3.0, 0.0),
        ("home 00b", 0.0, 0.0),
        ("3m_East yNED", 0.0, 3.0),
        ("home 00c", 0.0, 0.0),
    ]

    results = []

    cam = CameraReceiver(cam_host, cam_port)
    for attempt in range(20):
        try:
            cam.connect()
            print(f"[diag] camera connected (attempt {attempt+1})")
            break
        except ConnectionError:
            print(f"[diag] camera not ready, retrying ({attempt+1}/20)...")
            time.sleep(1)
    else:
        print("ERROR: camera connection failed after 20 attempts")
        return 1

    try:
        for name, px, py in positions:
            print(f"\n[diag] moving to {name}...")
            send_pos(conn, px, py, -ALT)
            pos = wait_pos(conn, px, py)
            if pos is None:
                print("  TIMEOUT")
                continue
            for _ in range(3):
                send_pos(conn, px, py, -ALT)
                time.sleep(1)

            print(f"  NED: x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}")
            sample = sample_detections(cam, detector)
            if sample is None:
                print("  NO DETECTIONS")
                continue
            tx, ty, tz, px, py, n = sample
            print(f"  tvec: tx={tx:.3f} ty={ty:.3f} tz={tz:.3f}")
            print(f"  pixel: px={px:.1f} py={py:.1f}  (center=320,240)  ({n} samples)")
            results.append((name, pos.x, pos.y, tx, ty, tz, px, py))

            try:
                frame = cam.read_frame()
                import cv2
                fname = name.replace(" ", "_").replace("(", "").replace(")", "").replace(",", "").replace("+", "")
                cv2.imwrite(f"diag_{fname}.png", frame)
                print(f"  saved diag_{fname}.png")
            except Exception:
                pass
    finally:
        cam.close()

    print("\n" + "="*60)
    print("AXIS MAPPING TABLE")
    print("="*60)
    print(f"{'Position':<22} {'NED_x':>6} {'NED_y':>6} | {'px':>7} {'py':>7} | {'cv_tx':>8} {'cv_ty':>8}")
    print("-"*72)
    for r in results:
        name, nx, ny, tx, ty, tz, px, py = r
        print(f"{name:<22} {nx:>6.2f} {ny:>6.2f} | {px:>7.1f} {py:>7.1f} | {tx:>8.3f} {ty:>8.3f}")

    if len(results) >= 4:
        home1 = results[0]
        north = results[1]
        home2 = results[2]
        east  = results[3]

        dpx_n = north[6] - home1[6]
        dpy_n = north[7] - home1[7]
        dpx_e = east[6] - home2[6]
        dpy_e = east[7] - home2[7]

        print(f"\nPIXEL DELTAS (reliable):")
        print(f"  North (+x NED): dpx={dpx_n:+.1f}  dpy={dpy_n:+.1f}")
        print(f"  East  (+y NED): dpx={dpx_e:+.1f}  dpy={dpy_e:+.1f}")
        print()
        print("Mapping (image center = 320, 240):")
        if abs(dpx_n) > abs(dpy_n):
            sign = "+" if dpx_n > 0 else "-"
            print(f"  NED North → pixel X ({sign}px)")
        else:
            sign = "+" if dpy_n > 0 else "-"
            print(f"  NED North → pixel Y ({sign}py)")
        if abs(dpx_e) > abs(dpy_e):
            sign = "+" if dpx_e > 0 else "-"
            print(f"  NED East  → pixel X ({sign}px)")
        else:
            sign = "+" if dpy_e > 0 else "-"
            print(f"  NED East  → pixel Y ({sign}py)")

    conn.close()


if __name__ == "__main__":
    sys.exit(main() or 0)
