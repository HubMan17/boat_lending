"""Sparse optical flow (Lucas-Kanade) for GPS-denied velocity estimation.

Tracks features between consecutive camera frames and converts pixel
displacement to angular rates for MAVLink OPTICAL_FLOW messages.
"""

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

DEFAULT_FOV_RAD = 0.7854

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)

FEATURE_PARAMS = dict(
    maxCorners=100,
    qualityLevel=0.3,
    minDistance=7,
    blockSize=7,
)

MIN_FEATURES = 30


@dataclass
class FlowResult:
    flow_x: float
    flow_y: float
    flow_rate_x: float
    flow_rate_y: float
    quality: int
    ground_distance: float


class OpticalFlow:
    """Sparse LK optical flow with pixel-to-angular conversion."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fov_rad: float = DEFAULT_FOV_RAD,
    ):
        self._fx = width / (2.0 * math.tan(fov_rad / 2.0))
        self._fy = self._fx
        self._prev_gray: np.ndarray | None = None
        self._prev_pts: np.ndarray | None = None
        self._prev_time: float = 0.0

    def process(
        self, frame: np.ndarray, ground_distance: float
    ) -> FlowResult | None:
        now = time.monotonic()

        if self._prev_gray is None:
            self._prev_gray = frame
            self._prev_pts = cv2.goodFeaturesToTrack(
                frame, **FEATURE_PARAMS
            )
            self._prev_time = now
            return None

        dt = now - self._prev_time
        if dt <= 0:
            dt = 1e-6

        if self._prev_pts is None or len(self._prev_pts) < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(
                self._prev_gray, **FEATURE_PARAMS
            )

        if self._prev_pts is None or len(self._prev_pts) == 0:
            self._prev_gray = frame
            self._prev_time = now
            return None

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, frame, self._prev_pts, None, **LK_PARAMS
        )

        if curr_pts is None or status is None:
            self._prev_gray = frame
            self._prev_pts = None
            self._prev_time = now
            return None

        good_mask = status.ravel() == 1
        n_tracked = int(good_mask.sum())
        n_total = len(self._prev_pts)

        if n_tracked == 0:
            self._prev_gray = frame
            self._prev_pts = None
            self._prev_time = now
            return None

        prev_good = self._prev_pts[good_mask]
        curr_good = curr_pts[good_mask]
        dxy = curr_good - prev_good
        median_dx = float(np.median(dxy[:, 0, 0]))
        median_dy = float(np.median(dxy[:, 0, 1]))

        flow_x = median_dx / self._fx
        flow_y = median_dy / self._fy
        flow_rate_x = flow_x / dt
        flow_rate_y = flow_y / dt

        quality = int(min(255, (n_tracked / max(n_total, 1)) * 255))

        self._prev_gray = frame
        self._prev_time = now
        if n_tracked < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(frame, **FEATURE_PARAMS)
        else:
            self._prev_pts = curr_good.reshape(-1, 1, 2)

        return FlowResult(
            flow_x=flow_x,
            flow_y=flow_y,
            flow_rate_x=flow_rate_x,
            flow_rate_y=flow_rate_y,
            quality=quality,
            ground_distance=ground_distance,
        )
