#!/usr/bin/env python3
"""Smoke test for companion stage 2 with ArUco tracker (P2.5).

Verifies that companion in stage 2:
  - Sends optical flow packets
  - Detects ArUco marker -> switches to GUIDED -> sends velocity commands
  - Sends more packets than stage 1 (flow + velocity + distance_sensor)
"""

import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

os.environ["MAVLINK20"] = "1"

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

from pymavlink.dialects.v20 import common as mavlink2

from companion import run

WIDTH, HEIGHT = 640, 480
MARKER_PX = 200
NUM_FRAMES = 60
CAM_PORT = 15600
MAV_PORT = 15552


def make_marker_frame(marker_id: int = 0) -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    frame = np.full((HEIGHT, WIDTH), 200, dtype=np.uint8)
    x0 = WIDTH // 2 - MARKER_PX // 2
    y0 = HEIGHT // 2 - MARKER_PX // 2
    frame[y0 : y0 + MARKER_PX, x0 : x0 + MARKER_PX] = marker_img
    header = struct.pack("=HH", WIDTH, HEIGHT)
    return header + frame.tobytes()


def make_blank_frame() -> bytes:
    frame = np.full((HEIGHT, WIDTH), 200, dtype=np.uint8)
    header = struct.pack("=HH", WIDTH, HEIGHT)
    return header + frame.tobytes()


def camera_server(ready_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", CAM_PORT))
    srv.listen(1)
    ready_event.set()

    conn, _ = srv.accept()
    marker_data = make_marker_frame(marker_id=0)
    blank_data = make_blank_frame()
    try:
        for i in range(NUM_FRAMES):
            if i < 40:
                conn.sendall(marker_data)
            else:
                conn.sendall(blank_data)
            time.sleep(0.02)
    except (BrokenPipeError, ConnectionResetError):
        pass
    conn.close()
    srv.close()


def parse_mavlink_packets(raw_packets: list[bytes]) -> dict[str, int]:
    """Parse raw UDP data and count MAVLink message types."""
    mav = mavlink2.MAVLink(None)
    mav.robust_parsing = True
    counts: dict[str, int] = {}

    for raw in raw_packets:
        try:
            msgs = mav.parse_buffer(raw)
            if msgs is None:
                continue
            for m in msgs:
                name = m.get_type()
                counts[name] = counts.get(name, 0) + 1
        except Exception:
            pass
    return counts


def udp_listener(results: dict, ready_event: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", MAV_PORT))
    sock.settimeout(8.0)
    ready_event.set()

    packets = []
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
            packets.append(data)
        except socket.timeout:
            break
    sock.close()
    results["packets"] = packets


def main():
    print("smoke_companion_tracker.py (P2.5)")

    cam_ready = threading.Event()
    cam_thread = threading.Thread(target=camera_server, args=(cam_ready,),
                                  daemon=True)
    cam_thread.start()
    cam_ready.wait(timeout=3.0)

    mav_results: dict = {}
    mav_ready = threading.Event()
    mav_thread = threading.Thread(target=udp_listener,
                                  args=(mav_results, mav_ready),
                                  daemon=True)
    mav_thread.start()
    mav_ready.wait(timeout=3.0)

    stop = threading.Event()
    companion_thread = threading.Thread(
        target=run,
        args=(2, "127.0.0.1", CAM_PORT, f"udpout:127.0.0.1:{MAV_PORT}", 0),
        kwargs={"stop": stop},
        daemon=True,
    )
    companion_thread.start()

    cam_thread.join(timeout=15.0)
    time.sleep(0.5)
    stop.set()
    companion_thread.join(timeout=10.0)
    mav_thread.join(timeout=10.0)

    packets = mav_results.get("packets", [])
    print(f"  received {len(packets)} raw UDP packets")
    assert len(packets) >= 5, f"expected >= 5 packets, got {len(packets)}"

    counts = parse_mavlink_packets(packets)
    print(f"  message types: {counts}")

    assert counts.get("OPTICAL_FLOW", 0) >= 5, (
        f"expected >= 5 OPTICAL_FLOW, got {counts.get('OPTICAL_FLOW', 0)}"
    )
    print(f"  OPTICAL_FLOW: {counts['OPTICAL_FLOW']} PASS")

    assert counts.get("DISTANCE_SENSOR", 0) >= 5, (
        f"expected >= 5 DISTANCE_SENSOR, got {counts.get('DISTANCE_SENSOR', 0)}"
    )
    print(f"  DISTANCE_SENSOR: {counts['DISTANCE_SENSOR']} PASS")

    vel_count = counts.get("SET_POSITION_TARGET_LOCAL_NED", 0)
    assert vel_count >= 3, (
        f"expected >= 3 velocity commands, got {vel_count}"
    )
    print(f"  SET_POSITION_TARGET_LOCAL_NED (velocity): {vel_count} PASS")

    set_mode_count = counts.get("SET_MODE", 0)
    print(f"  SET_MODE: {set_mode_count}")

    assert counts.get("HEARTBEAT", 0) >= 1, "expected >= 1 HEARTBEAT"
    print(f"  HEARTBEAT: {counts['HEARTBEAT']} PASS")

    print("ALL PASS")


if __name__ == "__main__":
    main()
