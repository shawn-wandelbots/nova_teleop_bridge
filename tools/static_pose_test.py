#!/usr/bin/env python3
"""Small-motion teleop test input for the A3 (nobody wearing the headset).

Publishes a STILL operator pose taken from the vendor PICO sample (already in the robot's
input frame), plus scripted controller buttons and an optional small, slow offset of one
hand.  It is steered at run time through a small JSON control file, so each phase
(idle -> buttons -> grips -> tiny hand offset -> back) can be stepped by hand while the
robot's logs are watched.

DEFAULT IS --dry-run.  --publish really publishes /remote/vr_data and /remote/vr_button.
--udp HOST:PORT instead sends each XRFrameData as one datagram (experimental: the format
hal_remote expects on its UDP port is not confirmed).

Control file (re-read every 100 ms; missing keys keep their defaults):
    {"grip": 0.0,            both hand_trig values, 0..1
     "trig": 0.0,            both index_trig values, 0..1
     "hold": [],             buttons held: any of a b x y lclick rclick
     "offset": [0, 0, 0],    target offset of the hand in metres, robot frame (x fwd, y left, z up)
     "side": "right",        which hand the offset applies to
     "stop": false}          true: stop publishing and exit (silence = input lost)

Safety: the offset is clamped to --max-offset (default 0.05 m) and slewed at --slew
(default 0.02 m/s); grips and triggers slew too.  Stopping the program means silence.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import a3_proto  # noqa: E402
from robot.recording import read_capture  # noqa: E402

RIGHT = {"shoulder": 17, "elbow": 19, "wrist": 21, "hand": 23}
LEFT = {"shoulder": 16, "elbow": 18, "wrist": 20, "hand": 22}
DEFAULT_CTL = {"grip": 0.0, "trig": 0.0, "hold": [], "offset": [0.0, 0.0, 0.0], "side": "right", "stop": False}


def log(msg):
    print("[pose-test %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def load_pose(path, index):
    frames = read_capture(path)
    f = a3_proto.decode_xr_frame(frames[index][1])
    if len(f.body) != a3_proto.BODY_JOINT_COUNT:
        raise SystemExit("sample frame has %d body joints, expected 24" % len(f.body))
    return f.body


def read_ctl(path, last):
    try:
        with open(path) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return last
    out = dict(DEFAULT_CTL)
    out.update({k: v for k, v in d.items() if k in DEFAULT_CTL})
    return out


def slew(cur, target, step):
    d = target - cur
    if abs(d) <= step:
        return target
    return cur + (step if d > 0 else -step)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(argv=None):
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true", help="encode only, send nothing (DEFAULT)")
    g.add_argument("--publish", action="store_true", help="REALLY publish to the robot's ROS 2 topics")
    g.add_argument("--udp", metavar="HOST:PORT", help="send XRFrameData datagrams instead (experimental)")
    ap.add_argument("--sample", default=os.path.join(here, "tests", "data", "pico_sample_contiguous.cap"))
    ap.add_argument("--frame", type=int, default=0, help="index of the sample frame used as the still pose")
    ap.add_argument("--ctl", required=True, help="path of the JSON control file")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--max-offset", type=float, default=0.05, help="hard clamp per axis, metres")
    ap.add_argument("--slew", type=float, default=0.02, help="offset slew rate, m/s")
    ap.add_argument("--duration", type=float, default=600.0, help="exit after this many seconds")
    ap.add_argument("--external-device", default="GENIELINK", choices=sorted(a3_proto.EXTERNAL_DEVICE_NAMES))
    ap.add_argument("--timestamp", default="unix_ms", choices=("unix_ms", "unix_us", "unix_ns", "const_u32max"))
    ap.add_argument("--no-controllers-in-frame", action="store_true",
                    help="leave controller_raw_datas out of XRFrameData (buttons only on /remote/vr_button)")
    args = ap.parse_args(argv)
    if args.max_offset > 0.10:
        raise SystemExit("--max-offset above 0.10 m is refused by this test tool")

    base = load_pose(args.sample, args.frame)
    ext = a3_proto.EXTERNAL_DEVICE_NAMES[args.external_device]

    pub = sock = dest = None
    if args.publish:
        from robot.teleop_bridge import RosPublisher  # noqa: PLC0415
        pub = RosPublisher(allow_other_publishers=True)
        log("PUBLISHING to ROS topics")
    elif args.udp:
        host, port = args.udp.rsplit(":", 1)
        dest = (host, int(port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log("SENDING XRFrameData datagrams to %s:%d" % dest)
    else:
        log("dry run: nothing is sent")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    ctl = dict(DEFAULT_CTL)
    off = [0.0, 0.0, 0.0]
    grip = trig = 0.0
    period = 1.0 / args.rate
    t0 = time.monotonic()
    nxt = t0
    last_ctl = last_log = 0.0
    sent = 0
    while not stop["flag"]:
        now = time.monotonic()
        if now - t0 > args.duration:
            log("duration reached")
            break
        if now - last_ctl >= 0.1:
            ctl = read_ctl(args.ctl, ctl)
            last_ctl = now
            if ctl.get("stop"):
                log("stop requested by control file")
                break
        try:
            tgt = [clamp(float(v), -args.max_offset, args.max_offset) for v in ctl["offset"]][:3]
            tgrip = clamp(float(ctl["grip"]), 0.0, 1.0)
            ttrig = clamp(float(ctl["trig"]), 0.0, 1.0)
            hold = set(str(b).lower() for b in ctl["hold"])
            side = RIGHT if str(ctl["side"]).lower() != "left" else LEFT
        except (TypeError, ValueError, KeyError):
            tgt, tgrip, ttrig, hold, side = [0.0, 0.0, 0.0], 0.0, 0.0, set(), RIGHT
        off = [slew(off[i], tgt[i], args.slew * period) for i in range(3)]
        grip = slew(grip, tgrip, 4.0 * period)      # 0 -> 1 in 0.25 s
        trig = slew(trig, ttrig, 1.0 * period)      # 0 -> 1 in 1 s

        weights = {side["hand"]: 1.0, side["wrist"]: 1.0, side["elbow"]: 0.5}
        body = []
        for j in base:
            w = weights.get(j.code, 0.0)
            pos = j.pos if not w else tuple(j.pos[i] + w * off[i] for i in range(3))
            body.append(a3_proto.Joint(code=j.code, pos=pos, quat=j.quat))
        controllers = [
            a3_proto.Controller(code=0, axis_click="lclick" in hold, index_trig=trig, hand_trig=grip,
                                key_one="x" in hold, key_two="y" in hold),
            a3_proto.Controller(code=1, axis_click="rclick" in hold, index_trig=trig, hand_trig=grip,
                                key_one="a" in hold, key_two="b" in hold),
        ]
        wall = time.time()
        ts = {"unix_ms": int(wall * 1e3), "unix_us": int(wall * 1e6), "unix_ns": int(wall * 1e9),
              "const_u32max": 4294967295}[args.timestamp]
        xr = a3_proto.encode_xr_frame(a3_proto.XRFrame(
            timestamp=ts, body=body, controllers=[] if args.no_controllers_in_frame else controllers))
        btn = a3_proto.encode_vr_buttons(a3_proto.VRButtons(timestamp=ts, controllers=controllers,
                                                           external_device=ext))
        if pub:
            pub.publish(xr, btn)
        elif sock:
            sock.sendto(xr, dest)
        sent += 1
        if now - last_log >= 5.0:
            log("sent %d | grip %.2f trig %.2f hold %s | offset %s m" % (
                sent, grip, trig, sorted(hold), [round(v, 3) for v in off]))
            last_log = now
        nxt += period
        delay = nxt - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            nxt = time.monotonic()
    if pub:
        pub.close()
    log("stopped after %d frames (silence from here on)" % sent)
    return 0


if __name__ == "__main__":
    sys.exit(main())
