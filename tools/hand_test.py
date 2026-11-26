#!/usr/bin/env python3
"""Careful finger test for the 10-motor hands.  Needs the robot's ROS 2 environment:

    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=232

    python3 tools/hand_test.py read
        Print the hand type and every motor position.  Moves nothing.

    python3 tools/hand_test.py nudge --motor left_index_bent_1 --to 2000 --go
        Move ONE motor slowly to a position, hold, and move it back.  All other motors are
        commanded to stay where they are.

    python3 tools/hand_test.py pose --side left --fingers two --go
        Slowly close the chosen fingers of one hand to the 'closed' pose of config/hand.json
        (scaled by --amount), hold, and open to the 'open' pose again.

Without --go the motion is only printed.  Ctrl-C stops sending at once (the fingers stay put).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import hand_control as hc  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wait_ready(hand, seconds=5.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not hand.ready():
        hand.spin(0.1)
    if not hand.ready():
        raise SystemExit("no usable hand state on %s (hand type '%s')" % (hand.cfg["state_topic"], hand.hand_type))


def show(hand):
    print("hand type: %s   other publishers on the command topic: %d" % (hand.hand_type, hand.other_publishers()))
    for n, v in zip(hand.names, hand.state):
        print("  %-24s %6.0f" % (n, v))


def run_path(hand, start, target, seconds, hold, go, back=None):
    """start -> target over `seconds`, hold, then -> back (default: start)."""
    back = list(start) if back is None else back
    legs = [(start, target, seconds), (target, target, hold), (target, back, seconds)]
    for a, b, dur in legs:
        t0 = time.monotonic()
        while True:
            k = min(1.0, (time.monotonic() - t0) / dur) if dur > 0 else 1.0
            pos = [x + (y - x) * k for x, y in zip(a, b)]
            if go:
                hand.publish_positions(pos)
            hand.spin(0.0)
            if k >= 1.0:
                break
            time.sleep(0.02)
        moved = [(hand.names[i], round(a[i]), round(b[i])) for i in range(len(a)) if round(a[i]) != round(b[i])]
        print("%s %s" % ("moved" if go else "would move", moved or "nothing (hold)"), flush=True)
    hand.spin(0.3)
    if go:
        print("now at:")
        show(hand)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("read", "nudge", "pose"))
    ap.add_argument("--config", default=os.path.join(HERE, "config", "hand.json"))
    ap.add_argument("--motor", help="nudge: motor name as printed by 'read'")
    ap.add_argument("--to", type=float, help="nudge: position 0..4095")
    ap.add_argument("--side", choices=hc.SIDES, default="left")
    ap.add_argument("--fingers", default="two", choices=sorted(hc.PRESETS))
    ap.add_argument("--amount", type=float, default=0.5, help="pose: how far to close, 0..1 (default 0.5)")
    ap.add_argument("--seconds", type=float, default=3.0, help="time for each move (minimum 1)")
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--stay", action="store_true", help="nudge: do not move back")
    ap.add_argument("--go", action="store_true", help="REALLY send commands to the fingers")
    a = ap.parse_args()
    a.seconds = max(1.0, a.seconds)

    cfg = hc.load_config(a.config)
    hand = hc.HandPublisher(cfg, a.fingers, a.amount, require_verified=False)
    try:
        wait_ready(hand)
        show(hand)
        if a.action == "read":
            return 0
        start = list(hand.state)
        if a.action == "nudge":
            if a.motor not in hand.names or a.to is None or not 0 <= a.to <= 4095:
                raise SystemExit("nudge needs --motor (a name from the list above) and --to 0..4095")
            target = list(start)
            target[hand.names.index(a.motor)] = a.to
            run_path(hand, start, target, a.seconds, a.hold, a.go, back=target if a.stay else None)
        else:
            off = 0 if a.side == "left" else 10
            fm = hc.FingerMap(cfg, a.fingers, a.amount)
            target, back = list(start), list(start)
            target[off:off + 10] = fm.pose(a.side, 1.0)
            back[off:off + 10] = fm.pose(a.side, 0.0)
            run_path(hand, start, target, a.seconds, a.hold, a.go, back=back)
        if not a.go:
            print("dry run: nothing was sent (add --go)")
    except KeyboardInterrupt:
        print("stopped: nothing more is sent")
    finally:
        hand.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
