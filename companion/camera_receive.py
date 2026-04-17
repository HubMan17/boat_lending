"""TCP client for Webots camera stream.

Connects to the ardupilot_vehicle_controller camera server,
decodes grayscale frames (=HH header + raw bytes), and yields
numpy arrays to the caller.
"""

import socket
import struct
import numpy as np

HEADER_FMT = "=HH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


class CameraReceiver:
    """Receive grayscale frames from Webots camera TCP stream."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5599):
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._width = 0
        self._height = 0

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._host, self._port))

    def close(self) -> None:
        if self._sock:
            self._sock.close()
            self._sock = None

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    def _recvall(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("camera stream closed")
            buf.extend(chunk)
        return bytes(buf)

    def read_frame(self) -> np.ndarray:
        """Read one frame. Returns grayscale HxW uint8 numpy array."""
        header = self._recvall(HEADER_SIZE)
        w, h = struct.unpack(HEADER_FMT, header)
        self._width = w
        self._height = h

        data = self._recvall(w * h)
        return np.frombuffer(data, dtype=np.uint8).reshape((h, w))

    def frames(self):
        """Generator yielding frames until connection drops."""
        try:
            while True:
                yield self.read_frame()
        except ConnectionError:
            return

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    import argparse
    import cv2

    parser = argparse.ArgumentParser(description="Webots camera stream viewer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5599)
    args = parser.parse_args()

    with CameraReceiver(args.host, args.port) as cam:
        print(f"Connected to {args.host}:{args.port}")
        for frame in cam.frames():
            print(f"\r{cam.width}x{cam.height}", end="", flush=True)
            cv2.imshow("Webots Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
