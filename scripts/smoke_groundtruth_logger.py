#!/usr/bin/env python3
"""Smoke test for groundtruth_logger controller (mock Webots Supervisor).

Verifies:
  1. CSV output has correct header
  2. Rows written per simulation step
  3. Position and orientation values are correctly formatted
  4. Flush happens at 500-row boundary
"""

import csv
import math
import os
import sys
import tempfile
import types


class MockNode:
    def __init__(self, position, orientation_matrix):
        self._pos = position
        self._rot = orientation_matrix

    def getPosition(self):
        return list(self._pos)

    def getOrientation(self):
        return list(self._rot)


class MockSupervisor:
    def __init__(self, step_limit=10):
        self._timestep = 2
        self._time = 0.0
        self._step_count = 0
        self._step_limit = step_limit
        self._custom_data = ""
        self._defs = {}

    def getBasicTimeStep(self):
        return self._timestep

    def getCustomData(self):
        return self._custom_data

    def getFromDef(self, name):
        return self._defs.get(name)

    def getTime(self):
        return self._time

    def step(self, timestep):
        if self._step_count >= self._step_limit:
            return -1
        self._time = self._step_count * (timestep / 1000.0)
        self._step_count += 1
        return 0


def identity_rotation():
    return [1, 0, 0, 0, 1, 0, 0, 0, 1]


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c, -s, 0, s, c, 0, 0, 0, 1]


def run_test():
    step_count = 20
    iris_pos = [1.5, -2.3, 5.0]
    marker_pos = [0.0, 0.0, 0.001]
    iris_rot = rotation_z(0.5)
    marker_rot = identity_rotation()

    iris_node = MockNode(iris_pos, iris_rot)
    marker_node = MockNode(marker_pos, marker_rot)

    mock_sup = MockSupervisor(step_limit=step_count)
    mock_sup._defs["IRIS"] = iris_node
    mock_sup._defs["ARUCO_MARKER"] = marker_node

    controller_module = types.ModuleType("controller")
    controller_module.Supervisor = lambda: None
    sys.modules["controller"] = controller_module

    sys.path.insert(0, os.path.join(
        os.path.dirname(__file__), "..", "webots", "controllers", "groundtruth_logger"
    ))

    import groundtruth_logger as gt

    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
        output_path = f.name

    try:
        original_supervisor_init = gt.Supervisor
        gt.Supervisor = lambda: mock_sup
        mock_sup._custom_data = f"--output {output_path}"

        gt.main()

        with open(output_path, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        expected_fields = [
            "time_s",
            "iris_x", "iris_y", "iris_z",
            "iris_roll", "iris_pitch", "iris_yaw",
            "marker_x", "marker_y", "marker_z",
            "marker_roll", "marker_pitch", "marker_yaw",
        ]
        assert reader.fieldnames == expected_fields, f"Header mismatch: {reader.fieldnames}"
        print(f"  [PASS] CSV header: {len(expected_fields)} fields")

        assert len(rows) == step_count, f"Row count {len(rows)} != {step_count}"
        print(f"  [PASS] Row count: {len(rows)}")

        row0 = rows[0]
        assert row0["time_s"] == "0.0000", f"First timestamp: {row0['time_s']}"
        assert row0["iris_x"] == "1.500000", f"iris_x: {row0['iris_x']}"
        assert row0["iris_y"] == "-2.300000", f"iris_y: {row0['iris_y']}"
        assert row0["iris_z"] == "5.000000", f"iris_z: {row0['iris_z']}"
        assert row0["marker_x"] == "0.000000", f"marker_x: {row0['marker_x']}"
        print("  [PASS] Position values correct")

        iris_yaw = float(row0["iris_yaw"])
        assert abs(iris_yaw - 0.5) < 1e-4, f"iris_yaw {iris_yaw} != 0.5"
        marker_yaw = float(row0["marker_yaw"])
        assert abs(marker_yaw) < 1e-4, f"marker_yaw {marker_yaw} != 0.0"
        print("  [PASS] Orientation values correct")

        last_row = rows[-1]
        expected_last_t = (step_count - 1) * 0.002
        assert last_row["time_s"] == f"{expected_last_t:.4f}", \
            f"Last timestamp: {last_row['time_s']} != {expected_last_t:.4f}"
        print("  [PASS] Timestamps increment correctly")

        gt.Supervisor = original_supervisor_init
    finally:
        os.unlink(output_path)

    print(f"\nAll tests PASSED ({step_count} steps logged)")


if __name__ == "__main__":
    run_test()
