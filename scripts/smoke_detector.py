#!/usr/bin/env python3
"""Smoke test for companion/detector.py.

Generates a synthetic grayscale image with an ArUco DICT_4X4_50 marker
at a known position and verifies detection + pose estimation.
"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

from detector import ArucoDetector, Detection

WIDTH, HEIGHT = 640, 480
FOV_RAD = 0.7854
MARKER_SIZE_M = 1.0


def make_test_frame(
    marker_id: int = 0,
    marker_px_size: int = 200,
    center_x: int | None = None,
    center_y: int | None = None,
) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px_size)
    frame = np.full((HEIGHT, WIDTH), 200, dtype=np.uint8)
    if center_x is None:
        center_x = WIDTH // 2
    if center_y is None:
        center_y = HEIGHT // 2
    x0 = center_x - marker_px_size // 2
    y0 = center_y - marker_px_size // 2
    frame[y0 : y0 + marker_px_size, x0 : x0 + marker_px_size] = marker_img
    return frame


def test_centered_detection():
    det = ArucoDetector(WIDTH, HEIGHT, FOV_RAD, MARKER_SIZE_M)
    frame = make_test_frame(marker_id=0, marker_px_size=200)
    results = det.detect(frame)
    assert len(results) == 1, f"expected 1 detection, got {len(results)}"
    r = results[0]
    assert r.marker_id == 0, f"expected id=0, got {r.marker_id}"
    assert abs(r.angle_x) < 0.05, f"angle_x too large: {r.angle_x:.4f}"
    assert abs(r.angle_y) < 0.05, f"angle_y too large: {r.angle_y:.4f}"
    assert r.distance > 0, f"distance must be positive: {r.distance}"
    print(
        f"  centered: id={r.marker_id} angle=({r.angle_x:.4f}, {r.angle_y:.4f}) "
        f"dist={r.distance:.2f} body=({r.x_body:.3f}, {r.y_body:.3f}, {r.z_body:.3f})"
    )


def test_offset_detection():
    det = ArucoDetector(WIDTH, HEIGHT, FOV_RAD, MARKER_SIZE_M)
    frame = make_test_frame(marker_id=0, marker_px_size=150, center_x=400, center_y=300)
    results = det.detect(frame)
    assert len(results) == 1, f"expected 1 detection, got {len(results)}"
    r = results[0]
    assert r.angle_x > 0.05, f"expected positive angle_x for right offset: {r.angle_x:.4f}"
    assert r.angle_y > 0.05, f"expected positive angle_y for down offset: {r.angle_y:.4f}"
    print(
        f"  offset:   id={r.marker_id} angle=({r.angle_x:.4f}, {r.angle_y:.4f}) "
        f"dist={r.distance:.2f} body=({r.x_body:.3f}, {r.y_body:.3f}, {r.z_body:.3f})"
    )


def test_no_marker():
    det = ArucoDetector(WIDTH, HEIGHT, FOV_RAD, MARKER_SIZE_M)
    frame = np.full((HEIGHT, WIDTH), 200, dtype=np.uint8)
    results = det.detect(frame)
    assert len(results) == 0, f"expected 0 detections on blank frame, got {len(results)}"
    print("  blank:    no detections (correct)")


def test_camera_matrix():
    det = ArucoDetector(WIDTH, HEIGHT, FOV_RAD, MARKER_SIZE_M)
    fx = det.camera_matrix[0, 0]
    fy = det.camera_matrix[1, 1]
    cx = det.camera_matrix[0, 2]
    cy = det.camera_matrix[1, 2]
    expected_fx = WIDTH / (2.0 * math.tan(FOV_RAD / 2.0))
    assert abs(fx - expected_fx) < 0.01, f"fx mismatch: {fx} vs {expected_fx}"
    assert abs(fy - expected_fx) < 0.01, f"fy mismatch: {fy} vs {expected_fx}"
    assert abs(cx - WIDTH / 2.0) < 0.01, f"cx mismatch: {cx}"
    assert abs(cy - HEIGHT / 2.0) < 0.01, f"cy mismatch: {cy}"
    print(f"  intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")


def main():
    print("smoke_detector.py")
    test_camera_matrix()
    test_centered_detection()
    test_offset_detection()
    test_no_marker()
    print("ALL PASS")


if __name__ == "__main__":
    main()
