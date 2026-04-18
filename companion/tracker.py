"""PID-based target tracker for GPS-denied visual servoing.

Converts ArUco detection body-frame offset (meters) into velocity commands
for SET_POSITION_TARGET_LOCAL_NED in GUIDED mode.

Uses EMA (exponential moving average) filter on body-frame position
to suppress solvePnP noise. Includes stabilization delay after acquire.
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
    """P controller: filtered body-frame error (m) -> body-frame velocity.

    After reset/acquire, waits `stabilize_sec` before emitting non-zero
    velocity to let the EMA filter converge.
    """

    def __init__(
        self,
        kp: float = 0.05,
        max_vel: float = 0.3,
        ema_alpha: float = 0.05,
        stabilize_sec: float = 3.0,
    ):
        self._kp = kp
        self._max_vel = max_vel
        self._alpha = ema_alpha
        self._stabilize_sec = stabilize_sec

        self._filt_fwd: float | None = None
        self._filt_right: float | None = None
        self._acquire_time: float | None = None

    def update(self, det: Detection, now: float | None = None) -> VelocityCommand:
        if now is None:
            now = time.monotonic()

        raw_fwd = det.x_body
        raw_right = det.y_body

        if self._filt_fwd is None:
            self._filt_fwd = raw_fwd
            self._filt_right = raw_right
            self._acquire_time = now
        else:
            self._filt_fwd += self._alpha * (raw_fwd - self._filt_fwd)
            self._filt_right += self._alpha * (raw_right - self._filt_right)

        if now - self._acquire_time < self._stabilize_sec:
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.0)

        vx = self._kp * self._filt_fwd
        vy = self._kp * self._filt_right

        vx = max(-self._max_vel, min(self._max_vel, vx))
        vy = max(-self._max_vel, min(self._max_vel, vy))

        return VelocityCommand(vx=vx, vy=vy, vz=0.0)

    def reset(self) -> None:
        self._filt_fwd = None
        self._filt_right = None
        self._acquire_time = None
