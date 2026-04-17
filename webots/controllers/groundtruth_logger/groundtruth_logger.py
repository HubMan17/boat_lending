"""Webots Supervisor controller that logs ground-truth poses of tracked nodes.

Writes a CSV file with per-timestep position (x, y, z) and orientation
(roll, pitch, yaw) for the Iris drone and ArucoMarker. The output file
path defaults to ``groundtruth.csv`` in the current working directory
and can be overridden via the ``--output`` controller argument.

Coordinate frame: Webots world (ENU). Downstream scripts that compare
against ArduPilot logs should convert to NED.
"""

import argparse
import csv
import os
import sys

from controller import Supervisor


def _euler_from_rotation(rotation_field):
    """Convert Webots axis-angle rotation to roll/pitch/yaw (radians).

    Webots stores orientation as a 4-element axis-angle (ax, ay, az, angle).
    ``Node.getOrientation()`` returns a 3x3 rotation matrix (row-major list
    of 9 floats), which is more convenient for extracting Euler angles.
    """
    node = rotation_field
    rot = node.getOrientation()
    r00, r01, r02 = rot[0], rot[1], rot[2]
    r10, r11, r12 = rot[3], rot[4], rot[5]
    r20, r21, r22 = rot[6], rot[7], rot[8]

    import math
    sy = math.sqrt(r00 * r00 + r10 * r10)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw = 0.0
    return roll, pitch, yaw


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Ground-truth pose logger")
    parser.add_argument(
        "--output", "-o",
        default="groundtruth.csv",
        help="Output CSV file path (default: groundtruth.csv)",
    )
    parser.add_argument(
        "--iris-def",
        default="IRIS",
        help="DEF name of the Iris node (default: IRIS)",
    )
    parser.add_argument(
        "--marker-def",
        default="ARUCO_MARKER",
        help="DEF name of the ArucoMarker node (default: ARUCO_MARKER)",
    )
    return parser.parse_args(argv)


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    raw_args = []
    arg_str = supervisor.getCustomData()
    if arg_str:
        raw_args = arg_str.split()

    args = parse_args(raw_args)

    iris_node = supervisor.getFromDef(args.iris_def)
    marker_node = supervisor.getFromDef(args.marker_def)

    if iris_node is None:
        print(f"[groundtruth_logger] ERROR: DEF '{args.iris_def}' not found", file=sys.stderr)
        sys.exit(1)
    if marker_node is None:
        print(f"[groundtruth_logger] ERROR: DEF '{args.marker_def}' not found", file=sys.stderr)
        sys.exit(1)

    output_path = os.path.abspath(args.output)
    print(f"[groundtruth_logger] logging to {output_path}")
    print(f"[groundtruth_logger] timestep={timestep} ms")

    fieldnames = [
        "time_s",
        "iris_x", "iris_y", "iris_z",
        "iris_roll", "iris_pitch", "iris_yaw",
        "marker_x", "marker_y", "marker_z",
        "marker_roll", "marker_pitch", "marker_yaw",
    ]

    csv_file = open(output_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    row_count = 0
    try:
        while supervisor.step(timestep) != -1:
            t = supervisor.getTime()
            ip = iris_node.getPosition()
            ir, ipitch, iy = _euler_from_rotation(iris_node)
            mp = marker_node.getPosition()
            mr, mpitch, my = _euler_from_rotation(marker_node)

            writer.writerow({
                "time_s": f"{t:.4f}",
                "iris_x": f"{ip[0]:.6f}",
                "iris_y": f"{ip[1]:.6f}",
                "iris_z": f"{ip[2]:.6f}",
                "iris_roll": f"{ir:.6f}",
                "iris_pitch": f"{ipitch:.6f}",
                "iris_yaw": f"{iy:.6f}",
                "marker_x": f"{mp[0]:.6f}",
                "marker_y": f"{mp[1]:.6f}",
                "marker_z": f"{mp[2]:.6f}",
                "marker_roll": f"{mr:.6f}",
                "marker_pitch": f"{mpitch:.6f}",
                "marker_yaw": f"{my:.6f}",
            })
            row_count += 1

            if row_count % 500 == 0:
                csv_file.flush()
    finally:
        csv_file.close()
        print(f"[groundtruth_logger] wrote {row_count} rows to {output_path}")


if __name__ == "__main__":
    main()
