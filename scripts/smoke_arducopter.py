#!/usr/bin/env python3
"""Headless smoke test for ArduCopter + Webots-JSON-SITL wiring.

Launches arducopter with --model webots-python, runs a minimal mock Webots
FDM sender (100 Hz zeroed IMU/GPS, drone resting on ground), connects to
MAVLink TCP :5760 and verifies HEARTBEAT presence, vehicle type, and rate.

Exits 0 on pass, 1 on fail.

This is the SITL-side gate — Webots + Mission Planner path is manual,
documented in docs/setup/arducopter_webots.md.
"""
from __future__ import annotations

import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path


ARDUPILOT_DIR = Path(
    os.environ.get("ARDUPILOT_DIR", str(Path.home() / "boat_lending" / "ardupilot"))
)
ARDU_BIN = ARDUPILOT_DIR / "build" / "sitl" / "bin" / "arducopter"
REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULTS = Path(os.environ.get("DEFAULTS", REPO_DIR / "params" / "iris.parm"))

SITL_HOST = "127.0.0.1"
MAVLINK_PORT = 5760
FDM_FROM_CTRL_PORT = 9002  # SITL -> (mock) Webots
FDM_TO_SITL_PORT = 9003    # (mock) Webots -> SITL

# python struct code per SIM_Webots_Python.h:
#   fdm: 'd'*(1+3+3+3+3+3) = 16 doubles
FDM_FMT = "d" * 16


class FdmMock:
    """Minimal stand-in for ardupilot_vehicle_controller.

    - Binds UDP :9002, drains any incoming servo packets from SITL.
    - Sends 100 Hz zeroed FDM packets to SITL:9003 so its physics ticks.
    """

    def __init__(self) -> None:
        self._sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_in.bind(("0.0.0.0", FDM_FROM_CTRL_PORT))
        self._sock_in.settimeout(0.005)

        self._sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._sock_in.close()
        self._sock_out.close()

    def _run(self) -> None:
        t0 = time.time()
        while not self._stop.is_set():
            # drain any servo packets, ignore content
            try:
                while True:
                    self._sock_in.recv(4096)
            except socket.timeout:
                pass
            except OSError:
                break

            # build a zeroed FDM packet with a monotonically increasing timestamp
            now = time.time() - t0
            pkt = struct.pack(
                FDM_FMT,
                now,
                0.0, 0.0, 0.0,           # imu_angular_velocity_rpy
                0.0, 0.0, -9.81,         # imu_linear_acceleration_xyz (gravity-down, rest)
                0.0, 0.0, 0.0,           # imu_orientation_rpy
                0.0, 0.0, 0.0,           # velocity_xyz
                0.0, 0.0, 0.0,           # position_xyz
            )
            try:
                self._sock_out.sendto(pkt, (SITL_HOST, FDM_TO_SITL_PORT))
            except OSError:
                pass
            time.sleep(0.01)  # 100 Hz


def launch_sitl(log_path: Path) -> subprocess.Popen[bytes]:
    if not ARDU_BIN.is_file() or not os.access(ARDU_BIN, os.X_OK):
        sys.exit(f"ERROR: {ARDU_BIN} not executable — run `./waf copter` first")
    if not DEFAULTS.is_file():
        sys.exit(f"ERROR: defaults file missing: {DEFAULTS}")

    cmd = [
        str(ARDU_BIN), "-S", "-I0",
        "--model", "webots-python",
        "--sim-address", "127.0.0.1",
        "--speedup", "1",
        "--defaults", str(DEFAULTS),
    ]
    print(f"[smoke] launching: {' '.join(cmd)}")
    log = open(log_path, "wb")
    return subprocess.Popen(
        cmd,
        cwd=str(ARDUPILOT_DIR),
        stdout=log,
        stderr=log,
        preexec_fn=os.setsid,
    )


def wait_for_tcp(host: str, port: int, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def run_checks(log_path: Path) -> int:
    if not wait_for_tcp(SITL_HOST, MAVLINK_PORT, timeout=20):
        print(f"ERROR: arducopter did not open TCP {MAVLINK_PORT} within 20s",
              file=sys.stderr)
        print(f"--- last 20 lines of {log_path} ---", file=sys.stderr)
        try:
            for line in log_path.read_text(errors="replace").splitlines()[-20:]:
                print(line, file=sys.stderr)
        except OSError:
            pass
        return 1

    from pymavlink import mavutil  # noqa: WPS433 (import inside fn — venv-agnostic)

    conn = mavutil.mavlink_connection(f"tcp:{SITL_HOST}:{MAVLINK_PORT}",
                                      source_system=255)
    try:
        hb = conn.wait_heartbeat(timeout=20)
        if hb is None:
            print("ERROR: no HEARTBEAT within 20s", file=sys.stderr)
            return 1
        print(f"[smoke] first HB: type={hb.type} autopilot={hb.autopilot} "
              f"mode={hb.custom_mode}")

        if hb.type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
            print(f"ERROR: MAV_TYPE={hb.type}, expected "
                  f"MAV_TYPE_QUADROTOR={mavutil.mavlink.MAV_TYPE_QUADROTOR}",
                  file=sys.stderr)
            return 1
        if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            print(f"ERROR: autopilot={hb.autopilot}, expected "
                  f"ARDUPILOTMEGA={mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA}",
                  file=sys.stderr)
            return 1

        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
        )

        start = time.time()
        hb_count = 0
        types: set[str] = set()
        while time.time() - start < 10:
            msg = conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue
            types.add(msg.get_type())
            if msg.get_type() == "HEARTBEAT":
                hb_count += 1

        print(f"[smoke] 10s: {hb_count} HBs, {len(types)} msg types")

        if hb_count < 5:
            print(f"ERROR: only {hb_count} HBs in 10 s (expected >= 5)",
                  file=sys.stderr)
            return 1

        print("[smoke] PASS")
        return 0
    finally:
        conn.close()


def main() -> int:
    log_path = Path("/tmp/arducopter_smoke.log")
    proc = launch_sitl(log_path)
    fdm = FdmMock()
    fdm.start()
    try:
        return run_checks(log_path)
    finally:
        fdm.stop()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


if __name__ == "__main__":
    sys.exit(main())
