#!/usr/bin/env python3
"""Smoke test for companion/optical_flow.py.

Generates synthetic frame pairs with known pixel shifts and verifies
that OpticalFlow returns correct flow direction, quality, and rates.
"""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

from optical_flow import OpticalFlow, FlowResult

WIDTH, HEIGHT = 640, 480
FOV_RAD = 0.7854


def make_textured_frame(seed: int = 42) -> np.ndarray:
    rng = np.random.RandomState(seed)
    frame = np.full((HEIGHT, WIDTH), 128, dtype=np.uint8)
    for _ in range(200):
        x = rng.randint(20, WIDTH - 20)
        y = rng.randint(20, HEIGHT - 20)
        r = rng.randint(3, 12)
        brightness = rng.randint(0, 255)
        cv2.circle(frame, (x, y), r, int(brightness), -1)
    return frame


def shift_frame(frame: np.ndarray, dx: int, dy: int) -> np.ndarray:
    m = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(frame, m, (WIDTH, HEIGHT), borderValue=128)


def test_first_frame_returns_none():
    of = OpticalFlow(WIDTH, HEIGHT, FOV_RAD)
    frame = make_textured_frame()
    result = of.process(frame, ground_distance=5.0)
    assert result is None, f"first frame should return None, got {result}"
    print("  first_frame: None (correct)")


def test_known_shift():
    of = OpticalFlow(WIDTH, HEIGHT, FOV_RAD)
    base = make_textured_frame(seed=100)
    shifted = shift_frame(base, dx=5, dy=3)

    r1 = of.process(base, ground_distance=5.0)
    assert r1 is None

    r2 = of.process(shifted, ground_distance=5.0)
    assert r2 is not None, "second frame should return FlowResult"
    assert isinstance(r2, FlowResult)

    assert r2.flow_x > 0, f"expected positive flow_x for rightward shift, got {r2.flow_x:.6f}"
    assert r2.flow_y > 0, f"expected positive flow_y for downward shift, got {r2.flow_y:.6f}"
    assert r2.quality > 0, f"expected quality > 0, got {r2.quality}"
    assert r2.ground_distance == 5.0

    print(f"  known_shift: flow=({r2.flow_x:.6f}, {r2.flow_y:.6f}) "
          f"rate=({r2.flow_rate_x:.4f}, {r2.flow_rate_y:.4f}) "
          f"q={r2.quality}")


def test_zero_shift():
    of = OpticalFlow(WIDTH, HEIGHT, FOV_RAD)
    base = make_textured_frame(seed=200)

    of.process(base, ground_distance=10.0)
    r = of.process(base.copy(), ground_distance=10.0)

    assert r is not None, "identical frames should still return a result"
    assert abs(r.flow_x) < 0.001, f"expected near-zero flow_x, got {r.flow_x:.6f}"
    assert abs(r.flow_y) < 0.001, f"expected near-zero flow_y, got {r.flow_y:.6f}"
    print(f"  zero_shift: flow=({r.flow_x:.6f}, {r.flow_y:.6f}) q={r.quality}")


def test_negative_shift():
    of = OpticalFlow(WIDTH, HEIGHT, FOV_RAD)
    base = make_textured_frame(seed=300)
    shifted = shift_frame(base, dx=-4, dy=-6)

    of.process(base, ground_distance=3.0)
    r = of.process(shifted, ground_distance=3.0)

    assert r is not None
    assert r.flow_x < 0, f"expected negative flow_x for leftward shift, got {r.flow_x:.6f}"
    assert r.flow_y < 0, f"expected negative flow_y for upward shift, got {r.flow_y:.6f}"
    print(f"  neg_shift:  flow=({r.flow_x:.6f}, {r.flow_y:.6f}) q={r.quality}")


def test_consecutive_frames():
    of = OpticalFlow(WIDTH, HEIGHT, FOV_RAD)
    base = make_textured_frame(seed=400)

    of.process(base, ground_distance=5.0)
    results = []
    for i in range(1, 4):
        shifted = shift_frame(base, dx=2 * i, dy=1 * i)
        r = of.process(shifted, ground_distance=5.0)
        assert r is not None, f"frame {i+1} should return FlowResult"
        results.append(r)

    for i, r in enumerate(results):
        assert r.quality > 0, f"frame {i+2} quality should be > 0"

    print(f"  consecutive: {len(results)} frames, all with flow")


def main():
    print("smoke_optical_flow.py")
    test_first_frame_returns_none()
    test_known_shift()
    test_zero_shift()
    test_negative_shift()
    test_consecutive_frames()
    print("ALL PASS")


if __name__ == "__main__":
    main()
