"""PID-based target tracker for GPS-denied visual servoing.

Converts ArUco detection body-frame angles into NED velocity commands
for SET_POSITION_TARGET_LOCAL_NED in GUIDED mode.
"""

import time
from dataclasses import dataclass

from detector import Detection


@dataclass
class VelocityCommand:
    vx: float
    vy: float
    vz: float


class TargetTracker:
    """PID controller: body-frame angular error -> body-frame velocity."""

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.0,
        kd: float = 0.1,
        max_vel: float = 1.0,
        integral_limit: float = 0.5,
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._max_vel = max_vel
        self._integral_limit = integral_limit

        self._int_fwd = 0.0
        self._int_right = 0.0
        self._prev_err_fwd = 0.0
        self._prev_err_right = 0.0
        self._last_time: float | None = None

    def update(self, det: Detection, now: float | None = None) -> VelocityCommand:
        if now is None:
            now = time.monotonic()

        err_fwd = det.angle_y
        err_right = det.angle_x

        if self._last_time is None:
            dt = 0.05
        else:
            dt = now - self._last_time
            if dt <= 0:
                dt = 0.05

        self._int_fwd += err_fwd * dt
        self._int_right += err_right * dt
        self._int_fwd = max(-self._integral_limit,
                            min(self._integral_limit, self._int_fwd))
        self._int_right = max(-self._integral_limit,
                              min(self._integral_limit, self._int_right))

        d_fwd = (err_fwd - self._prev_err_fwd) / dt
        d_right = (err_right - self._prev_err_right) / dt

        vx = self._kp * err_fwd + self._ki * self._int_fwd + self._kd * d_fwd
        vy = self._kp * err_right + self._ki * self._int_right + self._kd * d_right

        vx = max(-self._max_vel, min(self._max_vel, vx))
        vy = max(-self._max_vel, min(self._max_vel, vy))

        self._prev_err_fwd = err_fwd
        self._prev_err_right = err_right
        self._last_time = now

        return VelocityCommand(vx=vx, vy=vy, vz=0.0)

    def reset(self) -> None:
        self._int_fwd = 0.0
        self._int_right = 0.0
        self._prev_err_fwd = 0.0
        self._prev_err_right = 0.0
        self._last_time = None
