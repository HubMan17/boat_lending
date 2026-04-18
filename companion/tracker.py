"""PID-based target tracker for GPS-denied visual servoing.

Converts ArUco detection body-frame offset (meters) into velocity commands.
State machine: STABILIZING -> ALIGNING -> ALIGNED -> DESCENDING.
Descent starts only after marker is centered for `align_hold_sec`.
"""

import time
from dataclasses import dataclass
from enum import Enum, auto

from detector import Detection


class TrackerState(Enum):
    STABILIZING = auto()
    ALIGNING = auto()
    ALIGNED = auto()
    DESCENDING = auto()


@dataclass
class VelocityCommand:
    vx: float
    vy: float
    vz: float


class TargetTracker:
    """PD(I) controller with alignment gate before descent.

    Flow: acquire marker -> stabilize EMA -> align (center marker) ->
    hold alignment for `align_hold_sec` -> descend at `descent_rate`.
    """

    def __init__(
        self,
        kp: float = 0.20,
        ki: float = 0.0,
        kd: float = 0.10,
        max_vel: float = 0.5,
        ema_alpha: float = 0.15,
        stabilize_sec: float = 1.0,
        integral_limit: float = 1.0,
        align_threshold_m: float = 0.5,
        align_hold_sec: float = 2.0,
        descent_rate: float = 0.3,
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._max_vel = max_vel
        self._alpha = ema_alpha
        self._stabilize_sec = stabilize_sec
        self._integral_limit = integral_limit
        self._align_threshold = align_threshold_m
        self._align_hold_sec = align_hold_sec
        self._descent_rate = descent_rate

        self._state = TrackerState.STABILIZING
        self._filt_fwd: float | None = None
        self._filt_right: float | None = None
        self._prev_fwd: float | None = None
        self._prev_right: float | None = None
        self._integral_fwd: float = 0.0
        self._integral_right: float = 0.0
        self._acquire_time: float | None = None
        self._prev_time: float | None = None
        self._align_start: float | None = None

    @property
    def state(self) -> TrackerState:
        return self._state

    def update(self, det: Detection, now: float | None = None) -> VelocityCommand:
        if now is None:
            now = time.monotonic()

        raw_fwd = det.x_body
        raw_right = det.y_body

        if self._filt_fwd is None:
            self._filt_fwd = raw_fwd
            self._filt_right = raw_right
            self._acquire_time = now
            self._prev_time = now
            self._prev_fwd = raw_fwd
            self._prev_right = raw_right
            self._state = TrackerState.STABILIZING
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.0)

        self._filt_fwd += self._alpha * (raw_fwd - self._filt_fwd)
        self._filt_right += self._alpha * (raw_right - self._filt_right)

        if self._state == TrackerState.STABILIZING:
            if now - self._acquire_time >= self._stabilize_sec:
                self._state = TrackerState.ALIGNING
            else:
                self._prev_fwd = self._filt_fwd
                self._prev_right = self._filt_right
                self._prev_time = now
                return VelocityCommand(vx=0.0, vy=0.0, vz=0.0)

        dt = now - self._prev_time if self._prev_time else 0.1
        if dt < 1e-6:
            dt = 0.1

        d_fwd = (self._filt_fwd - self._prev_fwd) / dt
        d_right = (self._filt_right - self._prev_right) / dt

        self._integral_fwd += self._filt_fwd * dt
        self._integral_right += self._filt_right * dt
        self._integral_fwd = max(-self._integral_limit,
                                  min(self._integral_limit, self._integral_fwd))
        self._integral_right = max(-self._integral_limit,
                                    min(self._integral_limit, self._integral_right))

        vx = (self._kp * self._filt_fwd
              + self._ki * self._integral_fwd
              + self._kd * d_fwd)
        vy = (self._kp * self._filt_right
              + self._ki * self._integral_right
              + self._kd * d_right)

        vx = max(-self._max_vel, min(self._max_vel, vx))
        vy = max(-self._max_vel, min(self._max_vel, vy))

        self._prev_fwd = self._filt_fwd
        self._prev_right = self._filt_right
        self._prev_time = now

        import math
        err = math.sqrt(self._filt_fwd**2 + self._filt_right**2)

        vz = 0.0
        if self._state == TrackerState.ALIGNING:
            if err < self._align_threshold:
                if self._align_start is None:
                    self._align_start = now
                elif now - self._align_start >= self._align_hold_sec:
                    self._state = TrackerState.ALIGNED
                    self._align_start = None
            else:
                self._align_start = None

        elif self._state == TrackerState.ALIGNED:
            self._state = TrackerState.DESCENDING

        elif self._state == TrackerState.DESCENDING:
            if err < self._align_threshold * 2:
                vz = self._descent_rate
            else:
                vz = 0.0

        return VelocityCommand(vx=vx, vy=vy, vz=vz)

    def reset(self) -> None:
        self._state = TrackerState.STABILIZING
        self._filt_fwd = None
        self._filt_right = None
        self._prev_fwd = None
        self._prev_right = None
        self._integral_fwd = 0.0
        self._integral_right = 0.0
        self._acquire_time = None
        self._prev_time = None
        self._align_start = None
