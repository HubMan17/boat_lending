#!/usr/bin/env python3
"""End-to-end precision landing test.

Connects to ArduCopter SITL via MAVLink, flies a GUIDED takeoff,
optionally moves by `--offset` East, then navigates to the marker
position `(--marker-x, --marker-y)` in LOCAL_NED and commands LAND.
Verifies that PrecLand brings the copter onto the marker within
the XY error threshold.

Webots (Phase 1 default): marker at world origin (0, 0). Takeoff at
origin = takeoff on marker; offset 5m East tests PrecLand pullback.

Gazebo (Phase 3b, iris_precland.sdf): marker at (5, 0). Pass
`--marker-x 5 --marker-y 0 --offset 0` — drone spawns 5m West of
marker, flies to it, LANDs with PrecLand refinement. Connect via
tcp:127.0.0.1:5765 (serial3); companion stays on 5763 (serial2);
5760 is owned by the UDP bridge, 5762 by SERIAL1.

Prerequisites:
  - ArduCopter SITL (start_arducopter.sh for Webots, run_iris_sim.sh
    for Gazebo; precland_copter.parm overlay)
  - simulator with stage1 world (ArUco marker at declared coords)
  - companion.py feeding LANDING_TARGET (or pass --companion to let
    this script launch it)

Usage (Webots, existing flow):
  python scripts/e2e_precland.py --mav tcp:127.0.0.1:5763

Usage (Gazebo all-in-one):
  python scripts/e2e_precland.py \\
      --mav tcp:127.0.0.1:5762 \\
      --marker-x 5.0 --marker-y 0.0 --offset 0 \\
      --companion --camera-backend gstreamer --cam-fov 1.2 \\
      --gt-csv /tmp/gt.csv
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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
            print(f"[e2e] arrived: x={msg.x:.2f} y={msg.y:.2f} "
                  f"(target=({target_x:.2f},{target_y:.2f}) err={dist:.2f} m)")
            return True
    return False


def wait_landed(conn, timeout: float = 120.0):
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


def _spawn_companion(args) -> subprocess.Popen:
    python = sys.executable
    companion_py = os.path.join(REPO_DIR, "companion", "companion.py")
    cmd = [
        python, companion_py,
        "--stage", "1",
        "--camera-backend", args.camera_backend,
        "--cam-port", str(args.cam_port),
        "--cam-fov", f"{args.cam_fov}",
        "--mav-url", args.companion_mav,
        "--marker-id", str(args.companion_marker_id),
    ]
    print(f"[e2e] launching companion: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=REPO_DIR)


def _spawn_gt_logger(gt_csv: str, world: str,
                      iris_model: str, marker_model: str) -> subprocess.Popen:
    logger_py = os.path.join(REPO_DIR, "gazebo", "scripts", "gz_groundtruth_logger.py")
    cmd = [
        sys.executable, logger_py,
        "--world", world,
        "--iris-model", iris_model,
        "--marker-model", marker_model,
        "--output", gt_csv,
    ]
    print(f"[e2e] launching gt logger: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=REPO_DIR)


def _terminate(proc: subprocess.Popen, label: str, timeout: float = 5.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[e2e] terminating {label} (pid={proc.pid})")
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()


def run(args) -> int:
    mav_url = args.mav
    alt = args.alt
    offset = args.offset
    threshold = args.threshold
    marker_x = args.marker_x
    marker_y = args.marker_y

    companion_proc: subprocess.Popen | None = None
    gt_proc: subprocess.Popen | None = None

    try:
        if args.companion:
            companion_proc = _spawn_companion(args)

        if args.gt_csv:
            gt_proc = _spawn_gt_logger(
                args.gt_csv, args.gt_world,
                args.gt_iris_model, args.gt_marker_model,
            )

        print(f"[e2e] connecting to {mav_url}")
        conn = mavutil.mavlink_connection(mav_url, source_system=255)

        wait_heartbeat(conn)
        request_data_stream(conn)

        print("[e2e] waiting 15s for EKF/GPS init")
        time.sleep(15)

        print("[e2e] setting GUIDED mode")
        if not set_mode(conn, GUIDED_MODE):
            print("ERROR: failed to set GUIDED mode", file=sys.stderr)
            return 1

        print("[e2e] arming")
        if not arm(conn, timeout=60):
            print("ERROR: failed to arm", file=sys.stderr)
            return 1
        print("[e2e] armed")

        print(f"[e2e] takeoff to {alt} m")
        takeoff(conn, alt)

        if not wait_altitude(conn, alt):
            print("ERROR: takeoff timeout", file=sys.stderr)
            return 1

        pos = get_local_position(conn)
        if pos is None:
            print("ERROR: no LOCAL_POSITION_NED", file=sys.stderr)
            return 1
        target_z = pos.z

        if offset > 0:
            off_x = pos.x
            off_y = pos.y + offset
            print(f"[e2e] moving {offset} m East: target=({off_x:.1f}, {off_y:.1f})")
            send_position_target(conn, off_x, off_y, target_z)
            if not wait_position(conn, off_x, off_y):
                print("ERROR: offset move timeout", file=sys.stderr)
                return 1
            time.sleep(2)

        print(f"[e2e] navigating to marker ({marker_x:.2f}, {marker_y:.2f}) in GUIDED")
        send_position_target(conn, marker_x, marker_y, target_z)
        if not wait_position(conn, marker_x, marker_y, tolerance=0.5):
            print("ERROR: approach to marker timeout", file=sys.stderr)
            return 1

        print("[e2e] centered over marker, holding 3s")
        for _ in range(3):
            send_position_target(conn, marker_x, marker_y, target_z)
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

        dx = final.x - marker_x
        dy = final.y - marker_y
        xy_error = math.sqrt(dx * dx + dy * dy)
        print(f"[e2e] landed at x={final.x:.3f} y={final.y:.3f} z={final.z:.3f}")
        print(f"[e2e] marker at  x={marker_x:.3f} y={marker_y:.3f}")
        print(f"[e2e] XY error: {xy_error:.3f} m (threshold: {threshold} m)")

        conn.close()

        if xy_error <= threshold:
            print(f"[e2e] PASS (error {xy_error:.3f} <= {threshold})")
            return 0
        print(f"[e2e] FAIL (error {xy_error:.3f} > {threshold})")
        return 1

    finally:
        _terminate(gt_proc, "gt-logger")
        _terminate(companion_proc, "companion")


def main() -> int:
    parser = argparse.ArgumentParser(description="E2E precision landing test")
    parser.add_argument("--mav", default="tcp:127.0.0.1:5765",
                        help="MAVLink connection string for this script "
                             "(default: tcp:127.0.0.1:5765, serial3)")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALT,
                        help=f"Takeoff altitude (m), default {TAKEOFF_ALT}")
    parser.add_argument("--offset", type=float, default=OFFSET_M,
                        help=f"East offset leg before approach (m), "
                             f"default {OFFSET_M}. 0 disables offset leg.")
    parser.add_argument("--threshold", type=float, default=XY_THRESHOLD,
                        help=f"Max XY error for PASS, default {XY_THRESHOLD}")
    parser.add_argument("--marker-x", type=float, default=0.0,
                        help="Marker X in LOCAL_NED (default 0, Webots)")
    parser.add_argument("--marker-y", type=float, default=0.0,
                        help="Marker Y in LOCAL_NED (default 0, Webots)")

    cg = parser.add_argument_group("companion autolaunch (optional)")
    cg.add_argument("--companion", action="store_true",
                    help="Spawn companion.py subprocess for the run")
    cg.add_argument("--camera-backend", choices=("tcp", "gstreamer"),
                    default="gstreamer",
                    help="Companion camera backend (default gstreamer)")
    cg.add_argument("--cam-port", type=int, default=5600,
                    help="Companion camera port (default 5600 for GST)")
    cg.add_argument("--cam-fov", type=float, default=1.2,
                    help="Companion camera FOV rad (default 1.2 for Gazebo iris_with_downcam; "
                         "use 0.7854 for Webots)")
    cg.add_argument("--companion-mav", default="tcp:127.0.0.1:5763",
                    help="MAVLink URL for companion (default tcp:127.0.0.1:5763, serial2)")
    cg.add_argument("--companion-marker-id", type=int, default=0)

    gg = parser.add_argument_group("gazebo ground-truth logger (optional)")
    gg.add_argument("--gt-csv", default="",
                    help="If set, spawn gz ground-truth logger writing this CSV")
    gg.add_argument("--gt-world", default="iris_precland")
    gg.add_argument("--gt-iris-model", default="iris_with_downcam")
    gg.add_argument("--gt-marker-model", default="aruco_marker_1p5m")

    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
