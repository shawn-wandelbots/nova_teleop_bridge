#!/usr/bin/env python3
"""Stand in for the headset app: open (or close) the control page from a terminal.

    python3 tools/session_hello.py --word TEST            # keeps saying hello until Ctrl-C, then goodbye
    python3 tools/session_hello.py --word TEST --once     # one hello, no goodbye
"""
import argparse
import os
import random
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import session  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=session.DEFAULT_PORT)
    ap.add_argument("--word", required=True)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()
    sid = random.getrandbits(32)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.robot_ip, a.port)
    s.sendto(session.pack(session.HELLO, sid, a.word), dest)
    print("hello sent, word %s" % a.word.upper())
    if a.once:
        return
    try:
        while True:
            time.sleep(1.0)
            s.sendto(session.pack(session.HELLO, sid, a.word), dest)
    except KeyboardInterrupt:
        for _ in range(3):
            s.sendto(session.pack(session.BYE, sid, a.word), dest)
            time.sleep(0.05)
        print("goodbye sent")


if __name__ == "__main__":
    main()
