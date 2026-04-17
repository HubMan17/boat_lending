"""MAVLink message sender for precision landing companion.

Packs and sends LANDING_TARGET, DISTANCE_SENSOR, and companion
HEARTBEAT to ArduCopter SITL via UDP.
"""

import os
import time

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from detector import Detection
from optical_flow import FlowResult

LANDING_TARGET_FRAME = mavlink2.MAV_FRAME_BODY_FRD
LANDING_TARGET_TYPE = mavlink2.LANDING_TARGET_TYPE_VISION_FIDUCIAL
DISTANCE_SENSOR_TYPE = mavlink2.MAV_DISTANCE_SENSOR_UNKNOWN
DISTANCE_SENSOR_ORIENT = mavlink2.MAV_SENSOR_ROTATION_PITCH_270


class MavlinkSender:
    """Send MAVLink precision-landing messages to ArduCopter SITL."""

    def __init__(
        self,
        url: str = "tcp:127.0.0.1:5764",
        source_system: int = 1,
        source_component: int = 197,
    ):
        self._url = url
        self._source_system = source_system
        self._source_component = source_component
        self._conn = None
        self._boot_ms = 0

    def connect(self) -> None:
        self._conn = mavutil.mavlink_connection(
            self._url,
            source_system=self._source_system,
            source_component=self._source_component,
        )
        self._boot_ms = int(time.monotonic() * 1000)
        self._request_data_stream()

    def _request_data_stream(self, rate_hz: int = 10) -> None:
        self._conn.mav.request_data_stream_send(
            1, 1,
            mavlink2.MAV_DATA_STREAM_ALL,
            rate_hz, 1,
        )

    def close(self) -> None:
        if self._conn:
            self._conn.close()
            self._conn = None

    def _time_boot_ms(self) -> int:
        return int(time.monotonic() * 1000) - self._boot_ms

    def send_heartbeat(self) -> None:
        self._conn.mav.heartbeat_send(
            mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink2.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def send_landing_target(self, det: Detection, target_num: int = 0) -> None:
        self._conn.mav.landing_target_send(
            self._time_boot_ms(),
            target_num,
            LANDING_TARGET_FRAME,
            det.angle_x,
            det.angle_y,
            det.distance,
            0.0,
            0.0,
            det.x_body,
            det.y_body,
            det.z_body,
            [1.0, 0.0, 0.0, 0.0],
            LANDING_TARGET_TYPE,
            1,
        )

    def send_optical_flow(self, result: FlowResult) -> None:
        self._conn.mav.optical_flow_send(
            self._time_boot_ms() * 1000,
            0,
            int(result.flow_x * 1e4),
            int(result.flow_y * 1e4),
            result.flow_rate_x * result.ground_distance,
            result.flow_rate_y * result.ground_distance,
            result.quality,
            result.ground_distance,
            flow_rate_x=result.flow_rate_x,
            flow_rate_y=result.flow_rate_y,
        )

    def recv_altitude(self) -> float | None:
        msg = self._conn.recv_match(
            type="GLOBAL_POSITION_INT", blocking=False
        )
        if msg is None:
            return None
        return msg.relative_alt / 1000.0

    def send_distance_sensor(self, distance_m: float) -> None:
        dist_cm = max(1, min(int(distance_m * 100), 12000))
        self._conn.mav.distance_sensor_send(
            self._time_boot_ms(),
            20,
            12000,
            dist_cm,
            DISTANCE_SENSOR_TYPE,
            0,
            DISTANCE_SENSOR_ORIENT,
            0,
        )

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
