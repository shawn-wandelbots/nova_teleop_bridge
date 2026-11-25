"""Fake ``xrobotoolkit_sdk`` module for testing without a headset (stdlib only).

Same function names as the real SDK.  Two data sources:

* synthetic (default): a standing figure in the *assumed* XRoboToolkit frame (+x right, +y up,
  -z forward, origin on the floor) with gently swinging arms and a scripted, deliberately
  HARMLESS controller pattern: grips never exceed 0.3 (robot dead-man threshold is 0.9) and
  the stick clicks are never pressed (both together are the robot's emergency stop).
* replay: ``configure(replay_capture=<.cap file>, config=<transform config dict>)`` plays
  recorded A3 XRFrameData frames, mapped back into the SDK frame with the inverse of the
  configured transform.  Sending them through the bridge must reproduce the recording.

Time base: every call to ``get_time_stamp_ns()`` advances the fake by one 10 ms tick, so the
output is deterministic and independent of wall-clock timing.
"""
from __future__ import annotations

import math
import os
import sys

TICK_NS = 10_000_000
GRIP_MAX = 0.3

_state = {"tick": -1, "open": False, "replay": None, "body_available": True}

# name, position (x right, y up, z back) in metres for a ~1.7 m person facing -z, T-pose.
_REST = [
    ("PELVIS", (0.00, 0.95, 0.00)), ("LEFT_HIP", (-0.09, 0.90, 0.00)), ("RIGHT_HIP", (0.09, 0.90, 0.00)),
    ("SPINE1", (0.00, 1.07, 0.00)), ("LEFT_KNEE", (-0.10, 0.50, 0.00)), ("RIGHT_KNEE", (0.10, 0.50, 0.00)),
    ("SPINE2", (0.00, 1.20, 0.00)), ("LEFT_ANKLE", (-0.10, 0.08, 0.00)), ("RIGHT_ANKLE", (0.10, 0.08, 0.00)),
    ("SPINE3", (0.00, 1.27, 0.00)), ("LEFT_FOOT", (-0.10, 0.02, -0.12)), ("RIGHT_FOOT", (0.10, 0.02, -0.12)),
    ("NECK", (0.00, 1.47, 0.00)), ("LEFT_COLLAR", (-0.07, 1.40, 0.00)), ("RIGHT_COLLAR", (0.07, 1.40, 0.00)),
    ("HEAD", (0.00, 1.57, 0.00)), ("LEFT_SHOULDER", (-0.18, 1.42, 0.00)), ("RIGHT_SHOULDER", (0.18, 1.42, 0.00)),
    ("LEFT_ELBOW", (-0.42, 1.42, 0.00)), ("RIGHT_ELBOW", (0.42, 1.42, 0.00)),
    ("BODY_LEFT_WRIST", (-0.66, 1.42, 0.00)), ("BODY_RIGHT_WRIST", (0.66, 1.42, 0.00)),
    ("BODY_LEFT_HAND", (-0.74, 1.42, 0.00)), ("BODY_RIGHT_HAND", (0.74, 1.42, 0.00)),
]
_ARM = {18: 0.24, 19: 0.24, 20: 0.48, 21: 0.48, 22: 0.56, 23: 0.56}   # distance from the shoulder


def configure(replay_capture=None, config=None, body_available=True):
    """Select the data source. Call before init()."""
    _state["body_available"] = bool(body_available)
    _state["replay"] = None
    if replay_capture:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root not in sys.path:
            sys.path.insert(0, root)
        from robot.a3_proto import decode_xr_frame
        from robot.recording import read_capture
        from teleop_common.transform import FrameTransform
        tf = FrameTransform.from_config(config or {})
        frames = []
        for _, data in read_capture(replay_capture):
            fr = decode_xr_frame(data)
            if len(fr.body) != 24:
                continue
            frames.append([tuple(tf.inverse_position(j.pos)) + tuple(tf.inverse_orientation(j.quat))
                           for j in fr.body])
        if not frames:
            raise ValueError("no 24-joint frames in %s" % replay_capture)
        _state["replay"] = frames


def tick_index() -> int:
    return max(_state["tick"], 0)


def init():
    _state["open"] = True
    _state["tick"] = -1


def close():
    _state["open"] = False


def get_time_stamp_ns():
    _state["tick"] += 1
    return 1_000_000_000 + _state["tick"] * TICK_NS


def _t():
    return tick_index() * TICK_NS / 1e9


def _swing():
    return math.radians(25.0) * math.sin(2.0 * math.pi * 0.5 * _t())   # +-25 deg at 0.5 Hz


def is_body_data_available():
    return _state["body_available"]


def get_body_joints_pose():
    rep = _state["replay"]
    if rep is not None:
        return [list(p) for p in rep[tick_index() % len(rep)]]
    a = _swing()
    out = []
    for i, (_, p) in enumerate(_REST):
        x, y, z = p
        if i in _ARM:                      # rotate the arm forward/back about the vertical axis
            side = -1.0 if x < 0 else 1.0
            r = _ARM[i]
            x = side * (0.18 + r * math.cos(a))
            z = -r * math.sin(a)
        out.append([x, y, z, 0.0, 0.0, 0.0, 1.0])
    return out


def get_headset_pose():
    rep = _state["replay"]
    if rep is not None:
        return list(rep[tick_index() % len(rep)][15])
    return [0.0, 1.62 + 0.005 * math.sin(_t()), -0.08, 0.0, 0.0, 0.0, 1.0]


def _hand(i):
    rep = _state["replay"]
    if rep is not None:
        return list(rep[tick_index() % len(rep)][i])
    return get_body_joints_pose()[i]


def get_left_controller_pose():
    return _hand(22)


def get_right_controller_pose():
    return _hand(23)


# ---- scripted, harmless controller pattern (period 200 ticks = 2 s) ----------------------
def _phase():
    return tick_index() % 200


def get_left_trigger():
    return min(_phase(), 100) / 200.0             # ramps 0 .. 0.5


def get_right_trigger():
    return 0.5 - min(_phase(), 100) / 200.0       # ramps 0.5 .. 0


def get_left_grip():
    return 0.1


def get_right_grip():
    return GRIP_MAX if _phase() >= 100 else 0.2


def get_left_axis():
    return [0.25 * math.sin(2.0 * math.pi * _phase() / 200.0), 0.0]


def get_right_axis():
    return [0.0, -0.25]


def get_left_axis_click():
    return False        # never: both stick clicks together = robot emergency stop


def get_right_axis_click():
    return False


def get_A_button():
    return 20 <= _phase() < 30


def get_B_button():
    return False


def get_X_button():
    return False


def get_Y_button():
    return 60 <= _phase() < 70
