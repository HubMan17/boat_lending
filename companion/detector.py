"""ArUco marker detector with pinhole pixel-to-body-frame projection.

Detects DICT_4X4_50 markers and computes angular offsets + distance
for MAVLink LANDING_TARGET. Camera intrinsics are derived from Webots
Camera FOV and resolution.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
DEFAULT_FOV_RAD = 0.7854
DEFAULT_MARKER_SIZE_M = 1.0


@dataclass
class Detection:
    marker_id: int
    angle_x: float
    angle_y: float
    distance: float
    x_body: float
    y_body: float
    z_body: float


class ArucoDetector:
    """Detect ArUco markers and project to body frame."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fov_rad: float = DEFAULT_FOV_RAD,
        marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    ):
        self._width = width
        self._height = height
        self._marker_size = marker_size_m

        fx = width / (2.0 * math.tan(fov_rad / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        self._camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
        )
        self._dist_coeffs = np.zeros(5, dtype=np.float64)

        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)

    @property
    def camera_matrix(self) -> np.ndarray:
        return self._camera_matrix

    def detect(self, frame: np.ndarray) -> list[Detection]:
        corners, ids, _ = self._detector.detectMarkers(frame)
        if ids is None:
            return []

        results = []
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i : i + 1],
                self._marker_size,
                self._camera_matrix,
                self._dist_coeffs,
            )
            tx, ty, tz = tvec[0][0]

            angle_x = math.atan2(tx, tz)
            angle_y = math.atan2(ty, tz)
            distance = float(np.linalg.norm(tvec[0][0]))

            x_body = tx
            y_body = ty
            z_body = tz

            results.append(
                Detection(
                    marker_id=int(marker_id),
                    angle_x=angle_x,
                    angle_y=angle_y,
                    distance=distance,
                    x_body=x_body,
                    y_body=y_body,
                    z_body=z_body,
                )
            )

        return results
