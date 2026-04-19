#!/usr/bin/env python3
"""Log Gazebo ground-truth poses for iris + ArUco marker to CSV.

Reads `/world/<world>/pose/info` published by the SceneBroadcaster plugin
through the `gz topic` CLI in `--json-output` mode, filters the Pose_V
vector by model name, converts quaternion -> roll/pitch/yaw, and writes
one CSV row per message.

Schema matches webots/controllers/groundtruth_logger/groundtruth_logger.py:
    time_s,
    iris_x, iris_y, iris_z, iris_roll, iris_pitch, iris_yaw,
    marker_x, marker_y, marker_z, marker_roll, marker_pitch, marker_yaw

The SceneBroadcaster publishes many entities (links, sensors) besides the
top-level model; only entries whose `name` equals `--iris-model` or
`--marker-model` exactly are used.

SIGINT/SIGTERM flush and close cleanly.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def extract_pose(entry: dict) -> tuple[float, float, float, float, float, float]:
    pos = entry.get("position", {}) or {}
    ori = entry.get("orientation", {}) or {}
    x = float(pos.get("x", 0.0))
    y = float(pos.get("y", 0.0))
    z = float(pos.get("z", 0.0))
    roll, pitch, yaw = quat_to_euler(
        float(ori.get("x", 0.0)),
        float(ori.get("y", 0.0)),
        float(ori.get("z", 0.0)),
        float(ori.get("w", 1.0)),
    )
    return x, y, z, roll, pitch, yaw


def iter_json_messages(stream):
    """Yield successive JSON objects from a text stream.

    `gz topic -e --json-output` prints one pretty-formatted JSON object per
    message — may span many lines. `raw_decode` consumes one object at a
    time from the accumulated buffer.
    """
    decoder = json.JSONDecoder()
    buf = ""
    while True:
        chunk = stream.read(4096)
        if not chunk:
            if buf.strip():
                try:
                    obj, _ = decoder.raw_decode(buf.strip())
                    yield obj
                except json.JSONDecodeError:
                    pass
            return
        buf += chunk
        while True:
            stripped = buf.lstrip()
            if not stripped:
                buf = ""
                break
            try:
                obj, end = decoder.raw_decode(stripped)
            except json.JSONDecodeError:
                break
            yield obj
            consumed = len(buf) - len(stripped) + end
            buf = buf[consumed:]


def main() -> int:
    parser = argparse.ArgumentParser(description="Gazebo pose/info -> CSV")
    parser.add_argument("--world", required=True,
                        help="Gazebo world name (maps to /world/<name>/pose/info)")
    parser.add_argument("--iris-model", required=True)
    parser.add_argument("--marker-model", required=True)
    parser.add_argument("--output", required=True,
                        help="Output CSV path")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after this many seconds (0 = unlimited)")
    parser.add_argument("--gz-bin", default="gz",
                        help="gz CLI binary (default: gz)")
    args = parser.parse_args()

    topic = f"/world/{args.world}/pose/info"
    cmd = [args.gz_bin, "topic", "-e", "-t", topic, "--json-output"]

    env = os.environ.copy()
    proc = subprocess.Popen(
        ["stdbuf", "-oL"] + cmd,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, env=env,
    )

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    f = open(args.output, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "time_s",
        "iris_x", "iris_y", "iris_z", "iris_roll", "iris_pitch", "iris_yaw",
        "marker_x", "marker_y", "marker_z", "marker_roll", "marker_pitch", "marker_yaw",
    ])

    stopping = {"v": False}

    def on_signal(_sig, _frame):
        stopping["v"] = True
        try:
            proc.terminate()
        except Exception:
            pass

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    t_start = time.monotonic()
    rows = 0
    iris_seen = False
    marker_seen = False
    last_log = t_start

    try:
        for msg in iter_json_messages(proc.stdout):
            if stopping["v"]:
                break
            if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
                break

            poses = msg.get("pose", []) or []
            iris = None
            marker = None
            for entry in poses:
                name = entry.get("name", "")
                if name == args.iris_model:
                    iris = entry
                elif name == args.marker_model:
                    marker = entry
            if iris is None or marker is None:
                continue

            ix, iy, iz, ir, ip, iyaw = extract_pose(iris)
            mx, my, mz, mr, mp, myaw = extract_pose(marker)

            t = time.monotonic() - t_start
            writer.writerow([
                f"{t:.4f}",
                f"{ix:.5f}", f"{iy:.5f}", f"{iz:.5f}",
                f"{ir:.5f}", f"{ip:.5f}", f"{iyaw:.5f}",
                f"{mx:.5f}", f"{my:.5f}", f"{mz:.5f}",
                f"{mr:.5f}", f"{mp:.5f}", f"{myaw:.5f}",
            ])
            rows += 1
            if not iris_seen:
                print(f"[gt] iris pose first seen: ({ix:.2f},{iy:.2f},{iz:.2f})",
                      file=sys.stderr)
                iris_seen = True
            if not marker_seen:
                print(f"[gt] marker pose first seen: ({mx:.2f},{my:.2f},{mz:.2f})",
                      file=sys.stderr)
                marker_seen = True

            now = time.monotonic()
            if now - last_log >= 5.0:
                print(f"[gt] {rows} rows so far, latest t={t:.1f}s",
                      file=sys.stderr)
                last_log = now
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        f.flush()
        f.close()
        print(f"[gt] wrote {rows} rows to {args.output}", file=sys.stderr)

    return 0 if rows > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
