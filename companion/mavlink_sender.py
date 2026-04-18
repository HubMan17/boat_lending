"""MAVLink message sender for precision landing companion.

Packs and sends LANDING_TARGET, DISTANCE_SENSOR, and companion
HEARTBEAT to ArduCopter SITL via UDP.
"""

import math
import os
import time

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from detector import Detection
from optical_flow import FlowResult
from tracker import VelocityCommand

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
        self._last_heading: float = 0.0
        self._last_roll: float = 0.0
        self._last_pitch: float = 0.0
        self._last_yaw: float = 0.0
        self._last_pos_x: float = 0.0
        self._last_pos_y: float = 0.0
        self._last_pos_z: float = 0.0
        self._last_vel_n: float = 0.0
        self._last_vel_e: float = 0.0
        self._last_alt_m: float = 0.0
        self._got_alt: bool = False
        self._last_mode: int | None = None
        self._mav_type: int | None = None

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

    def drain(self) -> None:
        """Read ALL pending messages, dispatch by type.

        Must be called once per loop iteration BEFORE accessing any state.
        Fixes pymavlink issue where recv_match(type=X) consumes messages
        of other types from the buffer.
        """
        while True:
            msg = self._conn.recv_msg()
            if msg is None:
                break
            t = msg.get_type()
            if t == "ATTITUDE":
                self._last_roll = msg.roll
                self._last_pitch = msg.pitch
                self._last_yaw = msg.yaw
                self._last_heading = msg.yaw
            elif t == "GLOBAL_POSITION_INT":
                self._last_alt_m = msg.relative_alt / 1000.0
                self._got_alt = True
            elif t == "LOCAL_POSITION_NED":
                self._last_pos_x = msg.x
                self._last_pos_y = msg.y
                self._last_pos_z = msg.z
                self._last_vel_n = msg.vx
                self._last_vel_e = msg.vy
            elif t == "HEARTBEAT":
                if hasattr(msg, 'type') and msg.type in (
                    mavlink2.MAV_TYPE_QUADROTOR,
                    mavlink2.MAV_TYPE_FIXED_WING,
                ):
                    self._last_mode = msg.custom_mode
                    if self._mav_type is None:
                        self._mav_type = msg.type

    def recv_altitude(self) -> float | None:
        if self._got_alt:
            self._got_alt = False
            return self._last_alt_m
        return None

    @property
    def position(self) -> tuple[float, float, float]:
        return self._last_pos_x, self._last_pos_y, self._last_pos_z

    @property
    def velocity_ned(self) -> tuple[float, float]:
        return self._last_vel_n, self._last_vel_e

    @property
    def heading(self) -> float:
        return self._last_heading

    @property
    def roll_deg(self) -> float:
        return math.degrees(self._last_roll)

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self._last_pitch)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self._last_yaw)

    @property
    def mav_type(self) -> int | None:
        return self._mav_type

    @property
    def is_plane(self) -> bool:
        return self._mav_type == mavlink2.MAV_TYPE_FIXED_WING

    def get_mode(self) -> int | None:
        return self._last_mode

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

    def send_velocity_ned(self, vn: float, ve: float, vd: float = 0.0,
                          yaw: float | None = None) -> None:
        """Send NED velocity. If yaw is given, hold that heading."""
        if yaw is not None:
            type_mask = (
                0b0000_1001_1100_0111
                # USE vx vy vz yaw, ignore rest
            )
        else:
            type_mask = (
                0b0000_1101_1100_0111
                # USE vx vy vz, ignore yaw/yaw_rate
            )
        self._conn.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            1, 1,
            mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            yaw if yaw is not None else 0, 0,
        )

    def set_mode(self, mode_id: int) -> None:
        self._conn.set_mode(mode_id)

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
