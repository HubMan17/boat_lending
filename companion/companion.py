"""Precision-landing companion main loop.

Stage 1: ArUco detection -> LANDING_TARGET + DISTANCE_SENSOR.
Stage 2: optical flow (EKF velocity) + ArUco pixel tracker.
         Each frame: pixel error from center -> body velocity -> NED via heading.
         No accumulated state. Yaw held at heading captured on acquire.
         Always GUIDED mode — never LOITER in GPS-denied.
"""

import argparse
import math
import os
import signal
import threading
import time

from camera_receive import CameraReceiver
from detector import ArucoDetector, Detection, DEFAULT_FOV_RAD
from mavlink_sender import MavlinkSender
from optical_flow import OpticalFlow

CAMERA_BACKENDS = ("tcp", "gstreamer")


def _make_camera(backend: str, host: str, port: int):
    if backend == "tcp":
        return CameraReceiver(host, port)
    if backend == "gstreamer":
        from camera_receive_gst import GStreamerCameraReceiver
        return GStreamerCameraReceiver(port=port, host=host)
    raise ValueError(f"unknown camera backend: {backend!r}")

TARGET_HZ = 20
TARGET_DT = 1.0 / TARGET_HZ
HEARTBEAT_INTERVAL = 1.0
DEFAULT_ALT = 1.0

GUIDED_MODE_COPTER = 4
LAND_MODE_COPTER = 9
GUIDED_MODE_PLANE = 15
QLAND_MODE_PLANE = 20
ACQUIRE_FRAMES = 3
LOST_FRAMES = 30
TRACK_MAX_ALT = 60.0
CLIMB_RATE_THRESHOLD = 0.3

KP = 0.10
KD = 0.5
MAX_VEL = 0.35
ALIGN_PX = 50
ALIGN_HOLD_SEC = 3.0
DESCENT_RATE = 0.3
LAND_ALT = 2.0

_stop_event = threading.Event()


def run(stage: int, cam_host: str, cam_port: int,
        mav_url: str, marker_id: int,
        track_max_alt: float = TRACK_MAX_ALT,
        track_enabled: threading.Event | None = None,
        stop: threading.Event | None = None,
        camera_backend: str = "tcp",
        cam_fov_rad: float = DEFAULT_FOV_RAD,
        show: bool = False) -> None:
    if show:
        import cv2 as _cv2
        _SHOW_WINDOW = "companion"
        _cv2.namedWindow(_SHOW_WINDOW, _cv2.WINDOW_NORMAL)
    else:
        _cv2 = None
    if stop is None:
        stop = _stop_event
    if track_enabled is None:
        track_enabled = threading.Event()

    detector = ArucoDetector(fov_rad=cam_fov_rad)
    flow = OpticalFlow() if stage >= 2 else None
    last_alt = DEFAULT_ALT
    prev_alt = DEFAULT_ALT
    prev_alt_time: float | None = None
    climb_rate = 0.0

    cm = detector.camera_matrix
    fx, fy = cm[0, 0], cm[1, 1]
    cx, cy = cm[0, 2], cm[1, 2]

    hold_yaw: float | None = None
    align_start: float | None = None
    descending = False

    with _make_camera(camera_backend, cam_host, cam_port) as cam, \
         MavlinkSender(mav_url) as mav:

        deadline_hb = time.monotonic() + 10.0
        while mav.mav_type is None and time.monotonic() < deadline_hb:
            mav.drain()
            time.sleep(0.05)

        if mav.is_plane:
            guided_mode = GUIDED_MODE_PLANE
            land_mode = QLAND_MODE_PLANE
        else:
            guided_mode = GUIDED_MODE_COPTER
            land_mode = LAND_MODE_COPTER

        vtype = "ArduPlane" if mav.is_plane else "ArduCopter"
        print(f"companion: stage={stage} cam[{camera_backend}]={cam_host}:{cam_port} "
              f"mav={mav_url} marker_id={marker_id} vehicle={vtype}")

        last_hb = 0.0
        frame_count = 0
        detect_count = 0
        flow_count = 0
        track_count = 0
        consecutive_detect = 0
        consecutive_lost = 0
        tracking_active = False

        while not stop.is_set():
            t0 = time.monotonic()

            try:
                frame = cam.read_frame()
            except ConnectionError:
                print("companion: camera stream closed")
                break

            mav.drain()

            if flow is not None:
                alt = mav.recv_altitude()
                if alt is not None and alt > 0.1:
                    now_alt = time.monotonic()
                    if prev_alt_time is not None:
                        dt_alt = now_alt - prev_alt_time
                        if dt_alt > 0.01:
                            climb_rate = (alt - prev_alt) / dt_alt
                    prev_alt = alt
                    prev_alt_time = now_alt
                    last_alt = alt
                flow_result = flow.process(frame, last_alt)
                if flow_result is not None:
                    mav.send_optical_flow(flow_result)
                    mav.send_distance_sensor(last_alt)
                    flow_count += 1

            raw_dets = detector.detect_raw(frame)

            target_raw = None
            for raw in raw_dets:
                if raw[0] == marker_id:
                    target_raw = raw
                    break

            found = target_raw is not None

            if _cv2 is not None:
                display = _cv2.cvtColor(frame, _cv2.COLOR_GRAY2BGR) \
                    if frame.ndim == 2 else frame.copy()
                for raw in raw_dets:
                    mid, _tx, _ty, _tz, px, py = raw
                    color = (0, 255, 0) if mid == marker_id else (0, 200, 255)
                    _cv2.circle(display, (int(px), int(py)), 30, color, 2)
                    _cv2.putText(display, f"id={mid}",
                                 (int(px) + 35, int(py) + 5),
                                 _cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                _cv2.drawMarker(display, (int(cx), int(cy)), (255, 255, 255),
                                _cv2.MARKER_CROSS, 20, 1)
                ratio = (detect_count + (1 if found else 0)) / max(1, frame_count + 1)
                label = (f"frame {frame_count + 1}  det {detect_count + (1 if found else 0)}"
                         f"  hit {ratio*100:.0f}%")
                _cv2.putText(display, label, (10, 25),
                             _cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                _cv2.imshow("companion", display)
                if (_cv2.waitKey(1) & 0xFF) == ord("q"):
                    stop.set()

            if found:
                mid, tx, ty, tz, px, py = target_raw
                consecutive_detect += 1
                consecutive_lost = 0

                if flow is None:
                    det_legacy = detector.detect(frame)
                    for d in det_legacy:
                        if d.marker_id == marker_id:
                            mav.send_landing_target(d)
                            mav.send_distance_sensor(d.distance)
                            break

                detect_count += 1

                still_climbing = climb_rate > CLIMB_RATE_THRESHOLD
                if (stage >= 2 and track_enabled.is_set()
                        and last_alt <= track_max_alt and not still_climbing):
                    if not tracking_active and consecutive_detect >= ACQUIRE_FRAMES:
                        current = mav.get_mode()
                        if current == land_mode:
                            pass
                        else:
                            if current is None or current != guided_mode:
                                mav.set_mode(guided_mode)
                            hold_yaw = mav.heading
                            align_start = None
                            descending = False
                            tracking_active = True
                            print(f"companion: marker acquired -> GUIDED "
                                  f"(alt={last_alt:.1f} yaw={mav.yaw_deg:.0f}°)")

                    if tracking_active:
                        err_px_x = px - cx
                        err_px_y = py - cy
                        err_px = math.sqrt(err_px_x**2 + err_px_y**2)

                        body_fwd = (cy - py) / fy * last_alt
                        body_right = (px - cx) / fx * last_alt

                        vx_body = KP * body_fwd
                        vy_body = KP * body_right
                        vx_body = max(-MAX_VEL, min(MAX_VEL, vx_body))
                        vy_body = max(-MAX_VEL, min(MAX_VEL, vy_body))

                        hdg = mav.heading
                        vn = vx_body * math.cos(hdg) - vy_body * math.sin(hdg)
                        ve = vx_body * math.sin(hdg) + vy_body * math.cos(hdg)

                        cur_vn, cur_ve = mav.velocity_ned
                        vn -= KD * cur_vn
                        ve -= KD * cur_ve
                        vn = max(-MAX_VEL, min(MAX_VEL, vn))
                        ve = max(-MAX_VEL, min(MAX_VEL, ve))

                        vd = 0.0
                        now = time.monotonic()
                        if err_px < ALIGN_PX:
                            if align_start is None:
                                align_start = now
                            elif now - align_start >= ALIGN_HOLD_SEC:
                                descending = True
                        else:
                            align_start = None

                        if descending:
                            if last_alt <= LAND_ALT:
                                drone_x, drone_y, _ = mav.position
                                print(f"companion: alt {last_alt:.1f}m <= {LAND_ALT}m, switching to LAND"
                                      f" (x={drone_x:.3f} y={drone_y:.3f})")
                                mav.set_mode(land_mode)
                                result_path = os.path.join(os.path.dirname(__file__),
                                                           "..", "e2e_result.txt")
                                with open(result_path, "w") as f:
                                    f.write(f"{drone_x:.4f} {drone_y:.4f} {last_alt:.2f}\n")
                                tracking_active = False
                                continue
                            if err_px < ALIGN_PX * 2:
                                vd = DESCENT_RATE
                            else:
                                vd = 0.0

                        state = "DESCENDING" if descending else (
                            "ALIGNED" if align_start else "ALIGNING")

                        mav.send_velocity_ned(vn, ve, vd, yaw=hold_yaw)
                        track_count += 1

                        if track_count % 5 == 1:
                            drone_x, drone_y, _ = mav.position
                            print(f"  trk: px=({px:.0f},{py:.0f}) "
                                  f"err_px={err_px:.0f} "
                                  f"body=({vx_body:+.2f},{vy_body:+.2f}) "
                                  f"ned=({vn:+.2f},{ve:+.2f},{vd:+.2f}) "
                                  f"drone=({drone_x:+.1f},{drone_y:+.1f}) "
                                  f"yaw={mav.yaw_deg:.0f}° "
                                  f"[{state}]")
            else:
                consecutive_detect = 0
                consecutive_lost += 1

                if tracking_active:
                    mav.send_velocity_ned(0, 0, 0, yaw=hold_yaw)

                    if consecutive_lost >= LOST_FRAMES:
                        tracking_active = False
                        align_start = None
                        descending = False
                        print(f"companion: marker lost -> hold "
                              f"(yaw={mav.yaw_deg:.0f}°)")

            if tracking_active and not track_enabled.is_set():
                tracking_active = False
                align_start = None
                descending = False
                print("companion: tracking paused")

            now = time.monotonic()
            if now - last_hb >= HEARTBEAT_INTERVAL:
                mav.send_heartbeat()
                last_hb = now

            frame_count += 1
            if frame_count % 100 == 0:
                alt_info = f" alt={last_alt:.1f} vz={climb_rate:.1f}" if flow is not None else ""
                track_info = f" track={track_count}" if stage >= 2 else ""
                mode_info = " GUIDED" if tracking_active else " HOLD"
                yaw_info = f" yaw={mav.yaw_deg:.0f}°"
                det_info = f" det={detect_count}/{frame_count}"
                print(f"companion: frames={frame_count}{det_info} "
                      f"flow={flow_count}{track_info}{mode_info}{alt_info}{yaw_info}")

            elapsed = time.monotonic() - t0
            sleep_time = TARGET_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    print(f"companion: stopped. frames={frame_count} "
          f"detections={detect_count} flow={flow_count} "
          f"track={track_count}")
    if _cv2 is not None:
        _cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Precision-landing companion computer")
    parser.add_argument("--stage", type=int, default=1,
                        help="scenario stage (1=static precland, 2=GPS-denied optical flow)")
    parser.add_argument("--cam-host", default="127.0.0.1")
    parser.add_argument("--cam-port", type=int, default=None,
                        help="camera port (default: 5599 for tcp, 5600 for gstreamer)")
    parser.add_argument("--camera-backend", choices=CAMERA_BACKENDS, default="tcp",
                        help="tcp = Webots TCP header+grayscale; "
                             "gstreamer = Gazebo GstCameraPlugin UDP H.264")
    parser.add_argument("--mav-url", default="tcp:127.0.0.1:5763",
                        help="MAVLink connection URL (default: tcp:127.0.0.1:5763)")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--cam-fov", type=float, default=DEFAULT_FOV_RAD,
                        help=f"Camera horizontal FOV (rad). Default {DEFAULT_FOV_RAD:.4f} "
                             f"matches Webots; Gazebo iris_with_downcam uses 1.2")
    parser.add_argument("--track", action="store_true",
                        help="Enable PID tracker (default: off)")
    parser.add_argument("--track-max-alt", type=float, default=TRACK_MAX_ALT,
                        help="Max altitude for tracking (default: 30)")
    parser.add_argument("--show", action="store_true",
                        help="Display live camera with detection overlay (needs display)")
    args = parser.parse_args()

    if args.cam_port is None:
        args.cam_port = 5600 if args.camera_backend == "gstreamer" else 5599

    te = None
    if args.track:
        te = threading.Event()
        te.set()

    def on_signal(_sig, _frame):
        _stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    run(args.stage, args.cam_host, args.cam_port,
        args.mav_url, args.marker_id, args.track_max_alt,
        track_enabled=te, camera_backend=args.camera_backend,
        cam_fov_rad=args.cam_fov,
        show=args.show)


if __name__ == "__main__":
    main()
