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

    def detect_raw(self, frame: np.ndarray) -> list[tuple]:
        """Return raw (marker_id, tx, ty, tz, px, py) without body-frame transform.
        px, py = marker center pixel coordinates."""
        corners, ids, _ = self._detector.detectMarkers(frame)
        if ids is None:
            return []
        results = []
        for i, marker_id in enumerate(ids.flatten()):
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i : i + 1],
                self._marker_size,
                self._camera_matrix,
                self._dist_coeffs,
            )
            tx, ty, tz = tvec[0][0]
            c = corners[i][0]
            px = float(c[:, 0].mean())
            py = float(c[:, 1].mean())
            results.append((int(marker_id), float(tx), float(ty), float(tz), px, py))
        return results

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

            # Camera frame (X=right, Y=down, Z=depth) to
            # body FRD (X=forward, Y=right, Z=down).
            # Empirically measured via diag_camera_axes.py:
            #   NED North → pixel -Y → cam ty maps to body x (forward)
            #   NED East  → pixel -X → cam tx maps to body y (right)
            x_body = ty
            y_body = tx
            z_body = tz
            distance = float(np.linalg.norm(tvec[0][0]))

            angle_x = math.atan2(y_body, z_body)  # rightward angle
            angle_y = math.atan2(x_body, z_body)  # forward angle

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
