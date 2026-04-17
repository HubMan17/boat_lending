#!/usr/bin/env python3
"""Smoke test for analyze_precland_log.py using a synthetic BIN log."""

import os
import struct
import sys
import tempfile
import math

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "companion"))


def build_fake_bin(path):
    """Build a minimal DataFlash BIN with PL, CTUN, XKF5 messages.

    DataFlash binary format:
      - Header: 0xA3 0x95 <msg_id> <payload>
      - FMT messages (id=128) define the format of other message types.
      - FMT payload: type(B), length(B), name(4s), format(16s), columns(64s)
    """
    HEADER = b"\xa3\x95"
    FMT_ID = 128

    def fmt_msg(msg_id, length, name, fmt_str, columns):
        name_b = name.encode().ljust(4, b"\x00")
        fmt_b = fmt_str.encode().ljust(16, b"\x00")
        col_b = columns.encode().ljust(64, b"\x00")
        payload = struct.pack("<BB4s16s64s", msg_id, length, name_b, fmt_b, col_b)
        return HEADER + struct.pack("<B", FMT_ID) + payload

    def pack_msg(msg_id, fmt_str, *values):
        payload = struct.pack("<" + fmt_str, *values)
        return HEADER + struct.pack("<B", msg_id) + payload

    PL_ID = 10
    CTUN_ID = 11
    XKF5_ID = 12

    # FMT for FMT itself: id=128, len=89, name=FMT, format=BBnNZ, columns=Type,Length,Name,Format,Columns
    fmt_fmt = fmt_msg(FMT_ID, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")

    # PL: TimeUS(Q), pX(f), pY(f), pZ(f), mX(f), mY(f), mZ(f), Heal(B)
    # format: QffffffB, length = 3 + 8 + 6*4 + 1 = 36
    pl_fmt_str = "QffffffB"
    pl_len = 3 + struct.calcsize("<" + pl_fmt_str)
    fmt_pl = fmt_msg(PL_ID, pl_len, "PL", pl_fmt_str, "TimeUS,pX,pY,pZ,mX,mY,mZ,Heal")

    # CTUN: TimeUS(Q), Alt(f), DAlt(f), TAlt(f)
    # format: Qfff, length = 3 + 8 + 3*4 = 23
    ctun_fmt_str = "Qfff"
    ctun_len = 3 + struct.calcsize("<" + ctun_fmt_str)
    fmt_ctun = fmt_msg(CTUN_ID, ctun_len, "CTUN", ctun_fmt_str, "TimeUS,Alt,DAlt,TAlt")

    # XKF5: TimeUS(Q), NVI(f), NPI(f)
    xkf5_fmt_str = "Qff"
    xkf5_len = 3 + struct.calcsize("<" + xkf5_fmt_str)
    fmt_xkf5 = fmt_msg(XKF5_ID, xkf5_len, "XKF5", xkf5_fmt_str, "TimeUS,NVI,NPI")

    with open(path, "wb") as f:
        f.write(fmt_fmt)
        f.write(fmt_pl)
        f.write(fmt_ctun)
        f.write(fmt_xkf5)

        base_t = 1_000_000
        for i in range(20):
            t = base_t + i * 100_000
            # Drone spiraling toward origin
            angle = i * 0.4
            radius = 3.0 * (1.0 - i / 20.0)
            px = radius * math.cos(angle)
            py = radius * math.sin(angle)
            pz = -5.0 + i * 0.2
            mx = px + 0.05
            my = py - 0.03
            mz = pz + 0.02
            heal = 1 if i >= 2 else 0

            f.write(pack_msg(PL_ID, pl_fmt_str, t, px, py, pz, mx, my, mz, heal))

            alt = 10.0 - i * 0.4
            dalt = 10.0 - i * 0.45
            f.write(pack_msg(CTUN_ID, ctun_fmt_str, t, alt, dalt, alt - 0.1))

            nvi = 0.1 * math.sin(i * 0.5)
            npi = 0.05 * math.cos(i * 0.3)
            f.write(pack_msg(XKF5_ID, xkf5_fmt_str, t, nvi, npi))


def test_analyze():
    from analyze_precland_log import load_messages, compute_metrics

    tmpdir = tempfile.mkdtemp()
    try:
        bin_path = os.path.join(tmpdir, "test.BIN")
        build_fake_bin(bin_path)

        pl_msgs, ctun_msgs, xkf5_msgs = load_messages(bin_path)

        assert len(pl_msgs) == 20, f"Expected 20 PL, got {len(pl_msgs)}"
        assert len(ctun_msgs) == 20, f"Expected 20 CTUN, got {len(ctun_msgs)}"
        assert len(xkf5_msgs) == 20, f"Expected 20 XKF5, got {len(xkf5_msgs)}"
        print(f"  parse: {len(pl_msgs)} PL, {len(ctun_msgs)} CTUN, {len(xkf5_msgs)} XKF5 — OK")

        metrics = compute_metrics(pl_msgs, ctun_msgs, xkf5_msgs)

        assert metrics["healthy_count"] == 18, f"Expected 18 healthy, got {metrics['healthy_count']}"
        assert metrics["final_xy_error_m"] < 1.0, f"Final error too large: {metrics['final_xy_error_m']}"
        assert metrics["max_pl_error_m"] > 0, "Max PL error should be > 0"
        assert metrics["mean_pl_error_m"] > 0, "Mean PL error should be > 0"
        assert "first_acquisition_alt_m" in metrics, "Missing first_acquisition_alt_m"
        assert "ekf_max_vel_innov" in metrics, "Missing ekf_max_vel_innov"
        print(f"  metrics: final_xy={metrics['final_xy_error_m']}m, "
              f"max_pl={metrics['max_pl_error_m']}m, "
              f"mean_pl={metrics['mean_pl_error_m']}m — OK")

        from analyze_precland_log import plot_trajectory, plot_altitude, plot_pl_error
        plot_trajectory(pl_msgs, tmpdir)
        plot_altitude(pl_msgs, ctun_msgs, tmpdir)
        plot_pl_error(pl_msgs, tmpdir)

        for fname in ["trajectory_xy.png", "altitude_vs_time.png", "pl_error_vs_time.png"]:
            fpath = os.path.join(tmpdir, fname)
            assert os.path.isfile(fpath), f"Missing plot: {fname}"
            size = os.path.getsize(fpath)
            assert size > 1000, f"Plot {fname} too small: {size} bytes"
            print(f"  plot: {fname} ({size} bytes) — OK")
    finally:
        import shutil
        shutil.rmtree(tmpdir, ignore_errors=True)


def main():
    print("=== smoke_analyze_precland ===")
    sys.path.insert(0, os.path.join(REPO, "scripts"))
    test_analyze()
    print("\nAll checks PASSED")


if __name__ == "__main__":
    main()
