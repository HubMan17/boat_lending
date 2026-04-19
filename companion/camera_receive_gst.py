"""GStreamer UDP H.264 camera receiver for Gazebo GstCameraPlugin.

Mirrors the CameraReceiver public API (companion/camera_receive.py) so
companion.py can swap backends behind a single interface. Frames come out
as grayscale HxW uint8 numpy arrays.

Requires OpenCV built with GStreamer support. Ubuntu 22.04 apt
`python3-opencv` satisfies this; Windows PyPI wheels do NOT (use WSL).
"""

import numpy as np

try:
    import cv2
except ImportError as e:
    raise ImportError(
        "cv2 is required for GStreamerCameraReceiver; "
        "install python3-opencv (apt) or opencv with GStreamer"
    ) from e


_PIPELINE_TEMPLATE = (
    "udpsrc port={port} "
    'caps="application/x-rtp,encoding-name=H264,payload=96" '
    "! rtph264depay ! avdec_h264 ! videoconvert "
    "! video/x-raw,format=GRAY8 "
    "! appsink drop=true sync=false max-buffers=2"
)


class GStreamerCameraReceiver:
    """Receive grayscale frames from Gazebo GstCameraPlugin UDP RTP stream."""

    def __init__(self, port: int = 5600, host: str = "127.0.0.1"):
        self._port = port
        self._host = host  # kept for API symmetry; udpsrc binds all ifaces
        self._cap: "cv2.VideoCapture | None" = None
        self._width = 0
        self._height = 0

    def _pipeline(self) -> str:
        return _PIPELINE_TEMPLATE.format(port=self._port)

    def connect(self) -> None:
        pipeline = self._pipeline()
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            info = cv2.getBuildInformation()
            gst_line = next(
                (ln for ln in info.splitlines() if "GStreamer" in ln),
                "GStreamer: ?",
            )
            raise RuntimeError(
                "VideoCapture failed to open GStreamer pipeline.\n"
                f"  pipeline: {pipeline}\n"
                f"  {gst_line.strip()}\n"
                "  (expected 'GStreamer: YES')"
            )

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    def read_frame(self) -> np.ndarray:
        """Read one frame. Returns grayscale HxW uint8 numpy array."""
        if self._cap is None:
            raise ConnectionError("camera stream not connected")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise ConnectionError("camera stream closed")
        if frame.ndim == 3:
            # appsink should negotiate GRAY8, but some builds still deliver BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._height, self._width = frame.shape[:2]
        return frame

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

    parser = argparse.ArgumentParser(description="Gazebo GStreamer camera viewer")
    parser.add_argument("--port", type=int, default=5600)
    args = parser.parse_args()

    with GStreamerCameraReceiver(args.port) as cam:
        print(f"Listening on UDP:{args.port}")
        for frame in cam.frames():
            print(f"\r{cam.width}x{cam.height}", end="", flush=True)
            cv2.imshow("Gazebo Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
