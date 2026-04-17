"""Precision-landing companion main loop.

Reads camera frames, detects ArUco markers, and sends
LANDING_TARGET + DISTANCE_SENSOR to ArduCopter SITL.
"""

import argparse
import signal
import threading
import time

from camera_receive import CameraReceiver
from detector import ArucoDetector
from mavlink_sender import MavlinkSender

TARGET_HZ = 20
TARGET_DT = 1.0 / TARGET_HZ
HEARTBEAT_INTERVAL = 1.0

_stop_event = threading.Event()


def run(stage: int, cam_host: str, cam_port: int,
        mav_url: str, marker_id: int,
        stop: threading.Event | None = None) -> None:
    if stop is None:
        stop = _stop_event

    detector = ArucoDetector()

    with CameraReceiver(cam_host, cam_port) as cam, \
         MavlinkSender(mav_url) as mav:

        print(f"companion: stage={stage} cam={cam_host}:{cam_port} "
              f"mav={mav_url} marker_id={marker_id}")

        last_hb = 0.0
        frame_count = 0
        detect_count = 0

        while not stop.is_set():
            t0 = time.monotonic()

            try:
                frame = cam.read_frame()
            except ConnectionError:
                print("companion: camera stream closed")
                break

            detections = detector.detect(frame)

            for det in detections:
                if det.marker_id != marker_id:
                    continue
                mav.send_landing_target(det)
                mav.send_distance_sensor(det.distance)
                detect_count += 1
                if detect_count % 10 == 1:
                    print(f"  det: ax={det.angle_x:.3f} ay={det.angle_y:.3f} "
                          f"d={det.distance:.1f} "
                          f"bx={det.x_body:.2f} by={det.y_body:.2f} "
                          f"bz={det.z_body:.2f}")

            now = time.monotonic()
            if now - last_hb >= HEARTBEAT_INTERVAL:
                mav.send_heartbeat()
                last_hb = now

            frame_count += 1
            if frame_count % 100 == 0:
                print(f"companion: frames={frame_count} "
                      f"detections={detect_count}")

            elapsed = time.monotonic() - t0
            sleep_time = TARGET_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    print(f"companion: stopped. frames={frame_count} "
          f"detections={detect_count}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Precision-landing companion computer")
    parser.add_argument("--stage", type=int, default=1,
                        help="scenario stage (1=static, 2=ship, 3=moving)")
    parser.add_argument("--cam-host", default="127.0.0.1")
    parser.add_argument("--cam-port", type=int, default=5599)
    parser.add_argument("--mav-url", default="tcp:127.0.0.1:5763",
                        help="MAVLink connection URL (default: tcp:127.0.0.1:5763)")
    parser.add_argument("--marker-id", type=int, default=0)
    args = parser.parse_args()

    def on_signal(_sig, _frame):
        _stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    run(args.stage, args.cam_host, args.cam_port,
        args.mav_url, args.marker_id)


if __name__ == "__main__":
    main()
