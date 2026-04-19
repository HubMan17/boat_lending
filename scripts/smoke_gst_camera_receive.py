"""Smoke test for GStreamerCameraReceiver — mock cv2.VideoCapture.

Runs fully offline (no real gstreamer pipeline needed): stubs cv2.VideoCapture
with a fake that returns 5 grayscale frames then EOFs. Verifies the API
contract (context manager, read_frame, frames() generator, width/height).

Intended to run in WSL2 where python3-opencv is installed with GStreamer,
but the mock doesn't actually invoke gstreamer so it works anywhere cv2
imports.
"""

import sys
import unittest.mock as mock
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

import camera_receive_gst  # noqa: E402

WIDTH, HEIGHT = 640, 480
NUM_FRAMES = 5


def main() -> int:
    frames = [
        np.full((HEIGHT, WIDTH), i * 40, dtype=np.uint8)
        for i in range(NUM_FRAMES)
    ]
    read_calls = [(True, f) for f in frames] + [(False, None)]

    fake_cap = mock.MagicMock()
    fake_cap.isOpened.return_value = True
    fake_cap.read.side_effect = read_calls

    errors: list[str] = []

    with mock.patch.object(camera_receive_gst.cv2, "VideoCapture",
                           return_value=fake_cap) as vc_mock:
        with camera_receive_gst.GStreamerCameraReceiver(port=5600) as cam:
            vc_mock.assert_called_once()
            pipeline_arg = vc_mock.call_args.args[0]
            if "udpsrc port=5600" not in pipeline_arg:
                errors.append(f"pipeline missing udpsrc port=5600: {pipeline_arg!r}")
            if "rtph264depay" not in pipeline_arg:
                errors.append("pipeline missing rtph264depay")

            received = 0
            for frame in cam.frames():
                if frame.shape != (HEIGHT, WIDTH):
                    errors.append(f"frame {received}: shape {frame.shape}")
                if frame.dtype != np.uint8:
                    errors.append(f"frame {received}: dtype {frame.dtype}")
                expected = received * 40
                if frame[0, 0] != expected:
                    errors.append(f"frame {received}: pixel {frame[0,0]} != {expected}")
                received += 1

            if received != NUM_FRAMES:
                errors.append(f"received {received}/{NUM_FRAMES} frames")
            if cam.width != WIDTH or cam.height != HEIGHT:
                errors.append(f"size {cam.width}x{cam.height} != {WIDTH}x{HEIGHT}")

    if errors:
        for e in errors:
            print(f"  FAIL: {e}")
        print("FAIL")
        return 1
    print(f"Received {NUM_FRAMES} frames {WIDTH}x{HEIGHT} uint8")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
