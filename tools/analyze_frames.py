#!/usr/bin/env python3
"""Offline analysis of captured XRFrameData datagrams (see tools/capture_udp.py).

Prints the numbers used in docs/A3_FRAME_ANALYSIS.md.  Read-only, no network, no ROS.

    python3 tools/analyze_frames.py <capture.cap>
"""
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.a3_proto import BODY_JOINT_NAMES, decode_xr_frame, encode_xr_frame  # noqa: E402
from robot.recording import read_capture  # noqa: E402
from teleop_common.transform import q_norm, q_rotate  # noqa: E402

N = {n: i for i, n in enumerate(BODY_JOINT_NAMES)}


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def norm(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def pct(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def main():
    recs = read_capture(sys.argv[1])
    raw = [r[1] for r in recs]
    t_ns = [r[0] for r in recs]
    frames = [decode_xr_frame(x) for x in raw]
    print("frames: %d   sizes: %s" % (len(raw), sorted(set(map(len, raw)))))
    print("re-encode byte-identical: %d / %d" % (sum(encode_xr_frame(f) == x for f, x in zip(frames, raw)), len(raw)))
    print("timestamp values: %s" % sorted(set(f.timestamp for f in frames)))
    print("counts left_hand/right_hand/body/controllers: %s" % sorted(set(
        (len(f.left_hand), len(f.right_hand), len(f.body), len(f.controllers)) for f in frames)))
    dts = [(b - a) / 1e6 for a, b in zip(t_ns, t_ns[1:])]
    print("arrival dt ms: mean %.3f median %.3f p99 %.3f max %.3f" % (st.mean(dts), st.median(dts), pct(dts, 99), max(dts)))

    P = lambda f, n: f.body[N[n]].pos   # noqa: E731
    Q = lambda f, n: f.body[N[n]].quat  # noqa: E731

    print("\n-- per-axis statistics (m) --")
    for n in ("HEAD", "PELVIS", "LEFT_ANKLE", "RIGHT_ANKLE", "LEFT_FOOT", "RIGHT_FOOT"):
        for ax in range(3):
            v = [P(f, n)[ax] for f in frames]
            print("  %-12s %s: mean %+.3f  min %+.3f  max %+.3f" % (n, "xyz"[ax], st.mean(v), min(v), max(v)))
    lo = [min(P(f, "LEFT_FOOT")[2], P(f, "RIGHT_FOOT")[2]) for f in frames]
    print("  lowest foot z per frame: mean %+.3f min %+.3f max %+.3f" % (st.mean(lo), min(lo), max(lo)))
    h = [P(f, "HEAD")[2] - min(P(f, "LEFT_FOOT")[2], P(f, "RIGHT_FOOT")[2]) for f in frames]
    print("  HEAD z - lowest FOOT z: median %.3f  max %.3f" % (st.median(h), max(h)))

    print("\n-- frame 0 (T-pose) positions --")
    for i, n in enumerate(BODY_JOINT_NAMES):
        p = frames[0].body[i].pos
        q = frames[0].body[i].quat
        print("  %2d %-16s p=(%+.3f %+.3f %+.3f)  q(xyzw)=(%+.3f %+.3f %+.3f %+.3f)" % ((i, n) + p + q))

    print("\n-- handedness / facing from positions --")
    up_err, hand = [], []
    for f in frames:
        right = sub(P(f, "RIGHT_HIP"), P(f, "LEFT_HIP"))
        up = sub(P(f, "SPINE3"), P(f, "PELVIS"))
        toes = sub(P(f, "LEFT_FOOT"), P(f, "LEFT_ANKLE"))
        fwd = cross(up, right)          # right-handed: up x right = forward
        hand.append(dot(unit(fwd), unit((toes[0], toes[1], 0.0))))
        up_err.append(ang(up, (0, 0, 1)))
    print("  spine (PELVIS->SPINE3) vs +z: median %.1f deg, p95 %.1f deg" % (st.median(up_err), pct(up_err, 95)))
    print("  cos(angle) between (up x right) and ankle->toe direction: median %+.3f, p5 %+.3f  (+1 => right-handed)"
          % (st.median(hand), pct(hand, 5)))

    print("\n-- quaternion convention --")
    qn = [q_norm(j.quat) for f in frames for j in f.body]
    print("  norm: min %.7f max %.7f" % (min(qn), max(qn)))
    for jn, a, b in (("PELVIS", "LEFT_HIP", "RIGHT_HIP"), ("SPINE3", "LEFT_COLLAR", "RIGHT_COLLAR")):
        ex, ey, ez = [], [], []
        for f in frames:
            right = sub(P(f, b), P(f, a))
            up = sub(P(f, "SPINE3"), P(f, "PELVIS"))
            fwd = cross(up, right)
            q = Q(f, jn)
            ex.append(ang(q_rotate(q, (1, 0, 0)), fwd))
            ey.append(ang(q_rotate(q, (0, 1, 0)), (-right[0], -right[1], -right[2])))
            ez.append(ang(q_rotate(q, (0, 0, 1)), up))
        print("  %-7s local +x vs forward: median %.1f p95 %.1f deg | +y vs left: median %.1f p95 %.1f | +z vs up: median %.1f p95 %.1f"
              % (jn, st.median(ex), pct(ex, 95), st.median(ey), pct(ey, 95), st.median(ez), pct(ez, 95)))
    yaws = [math.degrees(math.atan2(*reversed(q_rotate(Q(f, "PELVIS"), (1, 0, 0))[:2]))) for f in frames]
    print("  pelvis yaw (deg): first %.1f  min %.1f  max %.1f" % (yaws[0], min(yaws), max(yaws)))
    spread = []
    for f in frames[:50]:
        q0 = Q(f, "PELVIS")
        for j in f.body:
            d = abs(sum(a * b for a, b in zip(q0, j.quat)))
            spread.append(math.degrees(2 * math.acos(min(1.0, d))))
    print("  first 50 frames (T-pose): angle between each joint quat and pelvis quat: median %.1f max %.1f deg"
          % (st.median(spread), max(spread)))
    flips = sum(1 for a, b in zip(frames, frames[1:]) for ja, jb in zip(a.body, b.body)
                if sum(x * y for x, y in zip(ja.quat, jb.quat)) < 0)
    wneg = sum(1 for f in frames for j in f.body if j.quat[3] < 0)
    print("  sign flips between consecutive frames: %d ; samples with w<0: %d / %d" % (flips, wneg, len(qn)))

    print("\n-- bone lengths (m), median over the recording --")
    for a, b in (("PELVIS", "SPINE3"), ("LEFT_HIP", "LEFT_KNEE"), ("LEFT_KNEE", "LEFT_ANKLE"),
                 ("LEFT_SHOULDER", "LEFT_ELBOW"), ("LEFT_ELBOW", "BODY_LEFT_WRIST"),
                 ("LEFT_SHOULDER", "RIGHT_SHOULDER"), ("LEFT_HIP", "RIGHT_HIP")):
        v = [norm(sub(P(f, a), P(f, b))) for f in frames]
        print("  %-14s -> %-16s median %.3f  (min %.3f max %.3f)" % (a, b, st.median(v), min(v), max(v)))

    print("\n-- per-frame joint displacement (m per 10 ms frame) --")
    allmax, wrist = [], []
    for a, b in zip(frames, frames[1:]):
        d = [norm(sub(ja.pos, jb.pos)) for ja, jb in zip(a.body, b.body)]
        allmax.append(max(d))
        wrist.append(max(d[N["BODY_LEFT_HAND"]], d[N["BODY_RIGHT_HAND"]]))
    print("  max over joints: median %.4f p99 %.4f max %.4f  (max => %.1f m/s at 100 Hz)"
          % (st.median(allmax), pct(allmax, 99), max(allmax), max(allmax) * 100))
    print("  hands only:      median %.4f p99 %.4f max %.4f" % (st.median(wrist), pct(wrist, 99), max(wrist)))
    stat = sum(1 for d in allmax if d == 0.0)
    print("  consecutive identical frames: %d" % stat)


if __name__ == "__main__":
    main()
