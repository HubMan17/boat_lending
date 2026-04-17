"""Smoke test for CameraReceiver — mock TCP server sends synthetic frames."""

import socket
import struct
import sys
import threading
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent / "companion"))
from camera_receive import CameraReceiver, HEADER_FMT

WIDTH, HEIGHT = 640, 480
NUM_FRAMES = 30
PORT = 15599


def mock_server(ready_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    ready_event.set()

    conn, _ = srv.accept()
    try:
        for i in range(NUM_FRAMES):
            pixel_val = i % 256
            img = bytes([pixel_val] * (WIDTH * HEIGHT))
            header = struct.pack(HEADER_FMT, WIDTH, HEIGHT)
            conn.sendall(header + img)
            time.sleep(0.01)
    finally:
        conn.close()
        srv.close()


def main():
    ready = threading.Event()
    t = threading.Thread(target=mock_server, args=(ready,), daemon=True)
    t.start()
    ready.wait()

    received = 0
    errors = []

    with CameraReceiver("127.0.0.1", PORT) as cam:
        for frame in cam.frames():
            expected_val = received % 256
            if frame.shape != (HEIGHT, WIDTH):
                errors.append(f"frame {received}: shape {frame.shape} != ({HEIGHT}, {WIDTH})")
            if frame[0, 0] != expected_val:
                errors.append(f"frame {received}: pixel {frame[0,0]} != {expected_val}")
            if cam.width != WIDTH or cam.height != HEIGHT:
                errors.append(f"frame {received}: size {cam.width}x{cam.height} != {WIDTH}x{HEIGHT}")
            received += 1

    print(f"Received {received}/{NUM_FRAMES} frames")
    if errors:
        for e in errors:
            print(f"  FAIL: {e}")
        sys.exit(1)
    if received != NUM_FRAMES:
        print(f"  FAIL: expected {NUM_FRAMES} frames")
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
