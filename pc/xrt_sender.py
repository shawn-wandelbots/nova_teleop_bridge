#!/usr/bin/env python3
"""PC-side sender: xrobotoolkit_sdk -> one NTB1 UDP datagram per tick -> robot bridge.

Runs on the Ubuntu PC next to the XRoboToolkit PC Service (Python >= 3.8, stdlib + the SDK).

    python3 pc/xrt_sender.py --robot-ip 192.168.1.20                 # real headset
    python3 pc/xrt_sender.py --robot-ip 127.0.0.1 --fake --count 300   # no headset

Safety behaviour:
  * sends raw SDK data only; if an SDK call fails or returns NaN the tick is skipped
  * if the SDK timestamp stops advancing for --stale-ms the sender goes silent (a frozen
    headset must not look like a live operator); the robot bridge watchdog then stops publishing
  * on exit it simply stops sending - no "release" datagrams are invented
"""
from __future__ import annotations

import argparse
import importlib
import math
import os
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pc.xrt_adapter import XrtAdapter  # noqa: E402
from teleop_common import wire  # noqa: E402
from teleop_common.transform import load_config  # noqa: E402

# The robot's own live teleop ports: never send our datagrams there.
FORBIDDEN_PORTS = (25618, 25619, 15000, 51000)
DEFAULT_PORT = 39500
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "config", "transform.json")


def log(msg: str) -> None:
    print("[sender %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def frame_is_finite(f: wire.Frame) -> bool:
    vals = [f.left.axis_x, f.left.axis_y, f.left.trigger, f.left.grip,
            f.right.axis_x, f.right.axis_y, f.right.trigger, f.right.grip]
    for p in (f.body or []):
        vals.extend(p)
    return all(math.isfinite(v) for v in vals)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", required=True)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--rate", type=float, default=100.0, help="ticks per second")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--fake", action="store_true", help="use pc/fake_xrt.py instead of xrobotoolkit_sdk")
    ap.add_argument("--fake-replay", default=None, metavar="CAP",
                    help="with --fake: replay recorded A3 frames (inverse-transformed) instead of the synthetic figure")
    ap.add_argument("--count", type=int, default=0, help="stop after this many ticks (0 = until signalled)")
    ap.add_argument("--stale-ms", type=float, default=100.0,
                    help="go silent when the SDK timestamp has not advanced for this long (0 = disable)")
    ap.add_argument("--stats-interval", type=float, default=5.0)
    a = ap.parse_args(argv)
    if a.port in FORBIDDEN_PORTS:
        ap.error("port %d belongs to the robot's own teleop stack" % a.port)
    if not 1.0 <= a.rate <= 500.0:
        ap.error("--rate must be within 1..500")

    cfg = load_config(a.config)
    if a.fake:
        from pc import fake_xrt as sdk
        sdk.configure(replay_capture=a.fake_replay, config=cfg)
    else:
        sdk = importlib.import_module(cfg.get("sdk", {}).get("module", "xrobotoolkit_sdk"))
    adapter = XrtAdapter(sdk, cfg, synthetic=a.fake)

    stop = {"flag": False}

    def _on_signal(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.robot_ip, a.port)
    session = int.from_bytes(os.urandom(4), "little")
    period = 1.0 / a.rate
    seq = sent = skipped = stale = errors = ticks = 0
    last_sdk_ts, last_sdk_change = None, time.monotonic()
    was_stale = False
    adapter.open()
    log("session %08x -> %s:%d at %.0f Hz (%s)" % (session, dest[0], dest[1], a.rate,
                                                   "FAKE SDK" if a.fake else "xrobotoolkit_sdk"))
    next_t = time.monotonic()
    t_stats, sent_stats = next_t, 0
    try:
        while not stop["flag"] and (a.count == 0 or ticks < a.count):
            ticks += 1
            now = time.monotonic()
            try:
                frame = adapter.snapshot()
            except Exception as e:  # noqa: BLE001 - any SDK failure means "no data this tick"
                errors += 1
                if errors in (1, 10, 100) or errors % 1000 == 0:
                    log("SDK read failed (%d so far): %r" % (errors, e))
                frame = None
            if frame is not None:
                if frame.t_sdk_ns != last_sdk_ts:
                    last_sdk_ts, last_sdk_change = frame.t_sdk_ns, now
                is_stale = bool(a.stale_ms) and (now - last_sdk_change) * 1000.0 > a.stale_ms
                if is_stale != was_stale:
                    log("SDK timestamp frozen -> sending paused" if is_stale else "SDK timestamp advancing -> sending resumed")
                    was_stale = is_stale
                if is_stale:
                    stale += 1
                elif not frame_is_finite(frame):
                    skipped += 1
                else:
                    seq = (seq + 1) & 0xFFFFFFFF
                    frame.session, frame.seq = session, seq
                    try:
                        sock.sendto(wire.pack(frame), dest)
                        sent += 1
                    except OSError as e:
                        errors += 1
                        if errors in (1, 10, 100):
                            log("send failed: %r" % (e,))
            if a.stats_interval and now - t_stats >= a.stats_interval:
                log("sent %.1f Hz | total %d | skipped(nan) %d | stale ticks %d | errors %d | body %s" % (
                    (sent - sent_stats) / (now - t_stats), sent, skipped, stale, errors,
                    "yes" if frame is not None and frame.body is not None else "NO"))
                t_stats, sent_stats = now, sent
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()      # fell behind: do not burst to catch up
    finally:
        try:
            adapter.close()
        finally:
            sock.close()
        log("exit: ticks=%d sent=%d skipped=%d stale=%d errors=%d" % (ticks, sent, skipped, stale, errors))
    return 0


if __name__ == "__main__":
    sys.exit(main())
