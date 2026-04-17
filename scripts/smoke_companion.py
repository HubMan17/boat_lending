#!/usr/bin/env python3
"""Smoke test for companion/companion.py.

Starts a mock camera server that sends synthetic frames with an ArUco
marker, launches the companion main loop in a thread, and verifies
that MAVLink messages arrive on a UDP listener.
"""

import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

from companion import run

WIDTH, HEIGHT = 640, 480
MARKER_PX = 200
NUM_FRAMES = 40
CAM_PORT = 15599
MAV_PORT = 15551


def make_marker_frame(marker_id: int = 0) -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    frame = np.full((HEIGHT, WIDTH), 200, dtype=np.uint8)
    x0 = WIDTH // 2 - MARKER_PX // 2
    y0 = HEIGHT // 2 - MARKER_PX // 2
    frame[y0 : y0 + MARKER_PX, x0 : x0 + MARKER_PX] = marker_img
    header = struct.pack("=HH", WIDTH, HEIGHT)
    return header + frame.tobytes()


def camera_server(ready_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", CAM_PORT))
    srv.listen(1)
    ready_event.set()

    conn, _ = srv.accept()
    frame_data = make_marker_frame(marker_id=0)
    try:
        for _ in range(NUM_FRAMES):
            conn.sendall(frame_data)
            time.sleep(0.02)
    except (BrokenPipeError, ConnectionResetError):
        pass
    conn.close()
    srv.close()


def udp_listener(results: dict, ready_event: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", MAV_PORT))
    sock.settimeout(5.0)
    ready_event.set()

    packets = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
            packets.append(data)
            if len(packets) >= 10:
                break
        except socket.timeout:
            break
    sock.close()
    results["packets"] = packets


def main():
    print("smoke_companion.py")

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
        args=(1, "127.0.0.1", CAM_PORT, "127.0.0.1", MAV_PORT, 0),
        kwargs={"stop": stop},
        daemon=True,
    )
    companion_thread.start()

    cam_thread.join(timeout=10.0)
    stop.set()
    companion_thread.join(timeout=10.0)
    mav_thread.join(timeout=6.0)

    packets = mav_results.get("packets", [])
    print(f"  received {len(packets)} MAVLink packets")
    assert len(packets) >= 3, (
        f"expected >= 3 MAVLink packets, got {len(packets)}"
    )

    total_bytes = sum(len(p) for p in packets)
    print(f"  total bytes: {total_bytes}")
    assert total_bytes > 0

    has_mavlink = any(p[0:1] == b"\xfd" for p in packets)
    assert has_mavlink, "no MAVLink v2 packets found (expected 0xFD header)"
    print("  MAVLink v2 header confirmed")

    print("ALL PASS")


if __name__ == "__main__":
    main()
