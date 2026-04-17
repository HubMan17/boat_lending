#!/usr/bin/env python3
"""Smoke test for companion/mavlink_sender.py.

Starts a UDP listener, sends MAVLink messages via MavlinkSender,
and verifies that HEARTBEAT, LANDING_TARGET, and DISTANCE_SENSOR
arrive with correct fields.
"""

import os
import socket
import sys
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

from pymavlink import mavutil

from detector import Detection
from mavlink_sender import MavlinkSender

LISTEN_PORT = 14561


def collect_messages(sock: socket.socket, duration: float) -> list[bytes]:
    packets: list[bytes] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            data, _ = sock.recvfrom(4096)
            packets.append(data)
        except socket.timeout:
            break
    return packets


def parse_messages(packets: list[bytes]) -> dict[str, list]:
    from pymavlink.dialects.v20 import common as mavlink2
    mav = mavlink2.MAVLink(None)
    msgs: dict[str, list] = {}
    for pkt in packets:
        parsed = mav.parse_buffer(pkt)
        if parsed:
            for m in parsed:
                name = m.get_type()
                msgs.setdefault(name, []).append(m)
    return msgs


def test_heartbeat():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", LISTEN_PORT))

    try:
        sender = MavlinkSender(f"udpout:127.0.0.1:{LISTEN_PORT}")
        sender.connect()
        for _ in range(3):
            sender.send_heartbeat()
            time.sleep(0.01)
        sender.close()

        packets = collect_messages(sock, 0.5)
        msgs = parse_messages(packets)
        hb_list = msgs.get("HEARTBEAT", [])
        assert len(hb_list) >= 1, f"expected >= 1 HEARTBEAT, got {len(hb_list)}"
        hb = hb_list[0]
        assert hb.type == 18, f"expected MAV_TYPE_ONBOARD_CONTROLLER(18), got {hb.type}"
        assert hb.autopilot == 8, f"expected MAV_AUTOPILOT_INVALID(8), got {hb.autopilot}"
        print(f"  heartbeat: type={hb.type} autopilot={hb.autopilot} count={len(hb_list)}")
    finally:
        sock.close()


def test_landing_target():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", LISTEN_PORT + 1))

    try:
        det = Detection(
            marker_id=0,
            angle_x=0.1,
            angle_y=-0.05,
            distance=3.5,
            x_body=0.35,
            y_body=-0.18,
            z_body=3.48,
        )
        sender = MavlinkSender(f"udpout:127.0.0.1:{LISTEN_PORT + 1}")
        sender.connect()
        for _ in range(3):
            sender.send_landing_target(det)
            time.sleep(0.01)
        sender.close()

        packets = collect_messages(sock, 0.5)
        msgs = parse_messages(packets)
        lt_list = msgs.get("LANDING_TARGET", [])
        assert len(lt_list) >= 1, f"expected >= 1 LANDING_TARGET, got {len(lt_list)}"
        lt = lt_list[0]
        assert abs(lt.angle_x - 0.1) < 0.001, f"angle_x mismatch: {lt.angle_x}"
        assert abs(lt.angle_y - (-0.05)) < 0.001, f"angle_y mismatch: {lt.angle_y}"
        assert abs(lt.distance - 3.5) < 0.01, f"distance mismatch: {lt.distance}"
        assert abs(lt.x - 0.35) < 0.01, f"x mismatch: {lt.x}"
        assert abs(lt.y - (-0.18)) < 0.01, f"y mismatch: {lt.y}"
        assert abs(lt.z - 3.48) < 0.01, f"z mismatch: {lt.z}"
        assert lt.position_valid == 1, f"position_valid: {lt.position_valid}"
        print(
            f"  landing_target: angle=({lt.angle_x:.3f}, {lt.angle_y:.3f}) "
            f"dist={lt.distance:.2f} body=({lt.x:.3f}, {lt.y:.3f}, {lt.z:.3f}) "
            f"count={len(lt_list)}"
        )
    finally:
        sock.close()


def test_distance_sensor():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", LISTEN_PORT + 2))

    try:
        sender = MavlinkSender(f"udpout:127.0.0.1:{LISTEN_PORT + 2}")
        sender.connect()
        for _ in range(3):
            sender.send_distance_sensor(5.0)
            time.sleep(0.01)
        sender.close()

        packets = collect_messages(sock, 0.5)
        msgs = parse_messages(packets)
        ds_list = msgs.get("DISTANCE_SENSOR", [])
        assert len(ds_list) >= 1, f"expected >= 1 DISTANCE_SENSOR, got {len(ds_list)}"
        ds = ds_list[0]
        assert ds.current_distance == 500, f"expected 500 cm, got {ds.current_distance}"
        assert ds.orientation == 25, f"expected PITCH_270(25), got {ds.orientation}"
        print(
            f"  distance_sensor: dist={ds.current_distance}cm orient={ds.orientation} "
            f"count={len(ds_list)}"
        )
    finally:
        sock.close()


def main():
    print("smoke_mavlink_sender.py")
    test_heartbeat()
    test_landing_target()
    test_distance_sensor()
    print("ALL PASS")


if __name__ == "__main__":
    main()
