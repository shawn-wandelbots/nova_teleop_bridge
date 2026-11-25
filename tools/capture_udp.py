#!/usr/bin/env python3
"""Capture raw UDP datagrams on a private loopback port into a .cap file.

Used to grab the robot's own PICO sample without touching the robot:

    python3 tools/capture_udp.py --port 39118 sample.cap &
    /opt/agibot/bin/pico_mock_player --host 127.0.0.1 --port 39118 <file.mcap>

Only binds 127.0.0.1 by default and refuses the robot's live teleop ports.
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.recording import CaptureWriter  # noqa: E402

FORBIDDEN_PORTS = (25618, 25619, 15000, 51000)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=39118)
    ap.add_argument("--idle-timeout", type=float, default=3.0, help="stop after this many seconds of silence")
    a = ap.parse_args()
    if a.port in FORBIDDEN_PORTS or a.port < 39000:
        ap.error("use a private port >= 39000 (the robot's teleop ports are off limits)")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((a.host, a.port))
    s.settimeout(a.idle_timeout)
    w = CaptureWriter(a.out)
    n, t0 = 0, None
    try:
        while True:
            try:
                data, _ = s.recvfrom(65535)
            except socket.timeout:
                if n:
                    break
                continue
            t = time.monotonic_ns()
            if t0 is None:
                t0 = t
            w.write(t - t0, data)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        w.close()
        s.close()
    print("captured %d datagrams -> %s" % (n, a.out))


if __name__ == "__main__":
    main()
