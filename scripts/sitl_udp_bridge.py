#!/usr/bin/env python3
"""Minimal bidirectional relay: SITL TCP :5760 <-> UDP <win_ip>:14550.

Replaces MAVProxy as a GCS transport when MAVProxy's init hangs on this
WSL box. No MAVLink parsing — just a byte pipe (MAVLink is self-framing).

Keeps the TCP side connected so SITL doesn't exit while MAVLink flows.
"""
import argparse
import select
import socket
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl-host", default="127.0.0.1")
    ap.add_argument("--sitl-port", type=int, default=5760)
    ap.add_argument("--gcs-host", required=True,
                    help="IP of the GCS host (Windows) as seen from WSL")
    ap.add_argument("--gcs-port", type=int, default=14550)
    ap.add_argument("--connect-timeout", type=float, default=20.0,
                    help="how long to keep retrying TCP connect")
    args = ap.parse_args()

    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    deadline = time.time() + args.connect_timeout
    while True:
        try:
            tcp.connect((args.sitl_host, args.sitl_port))
            break
        except (ConnectionRefusedError, OSError):
            if time.time() > deadline:
                print(f"ERROR: could not connect to SITL "
                      f"{args.sitl_host}:{args.sitl_port} "
                      f"after {args.connect_timeout:.0f}s",
                      file=sys.stderr)
                return 1
            time.sleep(0.5)
    print(f"TCP connected to SITL {args.sitl_host}:{args.sitl_port}")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", 0))
    gcs = (args.gcs_host, args.gcs_port)
    print(f"UDP bridge -> {gcs[0]}:{gcs[1]}  (local port {udp.getsockname()[1]})")
    print("Ctrl+C to stop")
    sys.stdout.flush()

    sent_to_gcs = 0
    sent_to_sitl = 0
    t0 = time.time()
    last_stat = t0
    try:
        while True:
            r, _, _ = select.select([tcp, udp], [], [], 1.0)
            if tcp in r:
                data = tcp.recv(4096)
                if not data:
                    print("SITL closed TCP", file=sys.stderr)
                    return 0
                udp.sendto(data, gcs)
                sent_to_gcs += len(data)
            if udp in r:
                data, _src = udp.recvfrom(4096)
                tcp.sendall(data)
                sent_to_sitl += len(data)
            now = time.time()
            if now - last_stat >= 5:
                print(f"[{now-t0:6.1f}s] to-GCS {sent_to_gcs}B   "
                      f"to-SITL {sent_to_sitl}B")
                sys.stdout.flush()
                last_stat = now
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0
    finally:
        tcp.close()
        udp.close()


if __name__ == "__main__":
    sys.exit(main())
