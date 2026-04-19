#!/usr/bin/env python3
"""Analyze ArduCopter .BIN log for precision landing performance."""

import argparse
import csv
import math
import os
import sys

from pymavlink import DFReader
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def parse_args():
    p = argparse.ArgumentParser(description="Analyze precision landing BIN log")
    p.add_argument("binfile", help="Path to .BIN log file")
    p.add_argument("--output-dir", default=None, help="Directory for PNG output (default: same as BIN)")
    p.add_argument("--gt-csv", default=None,
                   help="Optional Gazebo/Webots ground-truth CSV "
                        "(schema: time_s, iris_*, marker_* rows). "
                        "Used to compute an independent 'true' final XY error.")
    p.add_argument("--show", action="store_true", help="Open plots interactively")
    return p.parse_args()


def analyze_gt_csv(path):
    if not os.path.isfile(path):
        print(f"WARN: gt-csv not found: {path}", file=sys.stderr)
        return None
    last = None
    count = 0
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"iris_x", "iris_y", "marker_x", "marker_y"}
        if not required.issubset(set(reader.fieldnames or [])):
            print(f"WARN: gt-csv missing columns {required - set(reader.fieldnames or [])}",
                  file=sys.stderr)
            return None
        for row in reader:
            last = row
            count += 1
    if not last:
        return None
    ix = float(last["iris_x"]); iy = float(last["iris_y"])
    mx = float(last["marker_x"]); my = float(last["marker_y"])
    err = math.sqrt((ix - mx) ** 2 + (iy - my) ** 2)
    return {
        "gt_rows": count,
        "gt_iris_final_xy": (round(ix, 3), round(iy, 3)),
        "gt_marker_xy": (round(mx, 3), round(my, 3)),
        "gt_true_final_xy_error_m": round(err, 3),
    }


def load_messages(path):
    mlog = DFReader.DFReader_binary(path)
    pl_msgs = []
    ctun_msgs = []
    xkf5_msgs = []

    while True:
        m = mlog.recv_match(type=["PL", "CTUN", "XKF5"])
        if m is None:
            break
        mtype = m.get_type()
        if mtype == "PL":
            pl_msgs.append(m)
        elif mtype == "CTUN":
            ctun_msgs.append(m)
        elif mtype == "XKF5":
            xkf5_msgs.append(m)

    return pl_msgs, ctun_msgs, xkf5_msgs


def compute_metrics(pl_msgs, ctun_msgs, xkf5_msgs):
    metrics = {}

    healthy = [m for m in pl_msgs if m.Heal > 0]
    if not healthy:
        metrics["healthy_count"] = 0
        return metrics

    t0 = healthy[0].TimeUS

    def t_sec(msg):
        return (msg.TimeUS - t0) * 1e-6

    errors = []
    for m in healthy:
        err = math.sqrt((m.pX - m.mX) ** 2 + (m.pY - m.mY) ** 2)
        errors.append(err)

    last = healthy[-1]
    final_xy = math.sqrt(last.pX ** 2 + last.pY ** 2)

    metrics["healthy_count"] = len(healthy)
    metrics["final_xy_error_m"] = round(final_xy, 3)
    metrics["max_pl_error_m"] = round(max(errors), 3)
    metrics["mean_pl_error_m"] = round(sum(errors) / len(errors), 3)
    metrics["landing_duration_s"] = round(t_sec(healthy[-1]), 2)

    if ctun_msgs:
        first_acq_time = healthy[0].TimeUS
        closest_ctun = min(ctun_msgs, key=lambda c: abs(c.TimeUS - first_acq_time))
        metrics["first_acquisition_alt_m"] = round(closest_ctun.Alt, 2)

    if xkf5_msgs:
        landing_start = healthy[0].TimeUS
        landing_end = healthy[-1].TimeUS
        nvi_vals = [abs(m.NVI) for m in xkf5_msgs if landing_start <= m.TimeUS <= landing_end]
        if nvi_vals:
            metrics["ekf_max_vel_innov"] = round(max(nvi_vals), 3)

    return metrics


def plot_trajectory(pl_msgs, output_dir):
    healthy = [m for m in pl_msgs if m.Heal > 0]
    if not healthy:
        return
    t0 = healthy[0].TimeUS
    times = [(m.TimeUS - t0) * 1e-6 for m in healthy]
    xs = [m.pX for m in healthy]
    ys = [m.pY for m in healthy]

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(xs, ys, c=times, cmap="viridis", s=10)
    ax.plot(0, 0, "rx", markersize=15, markeredgewidth=3, label="Marker")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory (PrecLand Estimator)")
    ax.legend()
    plt.colorbar(sc, ax=ax, label="Time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "trajectory_xy.png"), dpi=150)
    plt.close(fig)


def plot_altitude(pl_msgs, ctun_msgs, output_dir):
    if not ctun_msgs:
        return
    healthy = [m for m in pl_msgs if m.Heal > 0]
    t0 = ctun_msgs[0].TimeUS
    times = [(m.TimeUS - t0) * 1e-6 for m in ctun_msgs]
    alts = [m.Alt for m in ctun_msgs]
    dalts = [m.DAlt for m in ctun_msgs]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, alts, label="Actual Alt", linewidth=1.5)
    ax.plot(times, dalts, label="Desired Alt", linewidth=1.5, linestyle="--")

    if healthy:
        acq_t = (healthy[0].TimeUS - t0) * 1e-6
        ax.axvline(acq_t, color="green", linestyle=":", label="First PL acquisition")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude vs Time")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "altitude_vs_time.png"), dpi=150)
    plt.close(fig)


def plot_pl_error(pl_msgs, output_dir):
    if not pl_msgs:
        return
    t0 = pl_msgs[0].TimeUS
    times = [(m.TimeUS - t0) * 1e-6 for m in pl_msgs]
    errors = [math.sqrt((m.pX - m.mX) ** 2 + (m.pY - m.mY) ** 2) for m in pl_msgs]
    heals = [m.Heal for m in pl_msgs]

    fig, ax = plt.subplots(figsize=(10, 5))

    for i in range(len(times) - 1):
        color = "#c8e6c9" if heals[i] > 0 else "#ffcdd2"
        ax.axvspan(times[i], times[i + 1], facecolor=color, alpha=0.5)

    ax.plot(times, errors, "k-", linewidth=1.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("PrecLand Estimator Error")

    green_patch = mpatches.Patch(color="#c8e6c9", label="Healthy")
    red_patch = mpatches.Patch(color="#ffcdd2", label="Unhealthy")
    ax.legend(handles=[green_patch, red_patch])

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "pl_error_vs_time.png"), dpi=150)
    plt.close(fig)


def main():
    args = parse_args()

    if not os.path.isfile(args.binfile):
        print(f"Error: file not found: {args.binfile}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir or os.path.dirname(os.path.abspath(args.binfile))
    os.makedirs(output_dir, exist_ok=True)

    print(f"Parsing {args.binfile} ...")
    pl_msgs, ctun_msgs, xkf5_msgs = load_messages(args.binfile)

    if not pl_msgs:
        print("No PL messages found — no precision landing data in this log.", file=sys.stderr)
        sys.exit(1)

    print(f"Found: {len(pl_msgs)} PL, {len(ctun_msgs)} CTUN, {len(xkf5_msgs)} XKF5 messages")

    metrics = compute_metrics(pl_msgs, ctun_msgs, xkf5_msgs)

    print("\n=== Precision Landing Metrics ===")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    if args.gt_csv:
        gt = analyze_gt_csv(args.gt_csv)
        if gt:
            print("\n=== Ground Truth (independent) ===")
            for k, v in gt.items():
                print(f"  {k}: {v}")

    plot_trajectory(pl_msgs, output_dir)
    plot_altitude(pl_msgs, ctun_msgs, output_dir)
    plot_pl_error(pl_msgs, output_dir)

    print(f"\nPlots saved to {output_dir}/")

    if args.show:
        matplotlib.use("TkAgg")
        plt.show()


if __name__ == "__main__":
    main()
