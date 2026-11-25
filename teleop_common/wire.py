"""PC -> robot intermediate UDP datagram ("NTB1", version 1).  See docs/WIRE_FORMAT.md.

One datagram per tick, fixed binary layout, little-endian, stdlib only (Python >= 3.8).
Poses are 7 float32: x, y, z, qx, qy, qz, qw in the *XRoboToolkit* frame, untransformed.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

MAGIC = b"NTB1"
VERSION = 1

FLAG_HEAD = 0x01        # head pose valid
FLAG_LEFT = 0x02        # left controller pose + inputs valid
FLAG_RIGHT = 0x04       # right controller pose + inputs valid
FLAG_BODY = 0x08        # 24 body joint poses follow
FLAG_SYNTHETIC = 0x80   # produced by the fake SDK, never by a headset
_KNOWN_FLAGS = FLAG_HEAD | FLAG_LEFT | FLAG_RIGHT | FLAG_BODY | FLAG_SYNTHETIC

BTN_AXIS_CLICK = 0x01
BTN_KEY_ONE = 0x02      # A (right) / X (left)
BTN_KEY_TWO = 0x04      # B (right) / Y (left)
_KNOWN_BTNS = BTN_AXIS_CLICK | BTN_KEY_ONE | BTN_KEY_TWO

BODY_JOINTS = 24

_HEADER = struct.Struct("<4sBBBBIIQQ")          # 32 bytes
_POSES = struct.Struct("<21f")                   # head, left, right: 84 bytes
_CTRL = struct.Struct("<4fB3x")                  # axis_x, axis_y, trigger, grip, buttons: 20 bytes
_BODY = struct.Struct("<%df" % (BODY_JOINTS * 7))  # 672 bytes
_CRC = struct.Struct("<I")

HEADER_SIZE = _HEADER.size
SIZE_NO_BODY = _HEADER.size + _POSES.size + 2 * _CTRL.size + _CRC.size            # 160
SIZE_WITH_BODY = SIZE_NO_BODY + _BODY.size                                         # 832

Pose = Tuple[float, float, float, float, float, float, float]
ZERO_POSE: Pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


class WireError(ValueError):
    """Raised by unpack(); ``reason`` is a short machine-readable tag."""

    def __init__(self, reason: str, detail: str = ""):
        super().__init__("%s %s" % (reason, detail))
        self.reason = reason


@dataclass
class ControllerState:
    axis_x: float = 0.0
    axis_y: float = 0.0
    trigger: float = 0.0     # index trigger 0..1
    grip: float = 0.0        # hand trigger 0..1
    axis_click: bool = False
    key_one: bool = False
    key_two: bool = False


@dataclass
class Frame:
    session: int = 0
    seq: int = 0
    t_send_ns: int = 0       # sender wall clock (time.time_ns) when the datagram was packed
    t_sdk_ns: int = 0        # xrobotoolkit_sdk.get_time_stamp_ns()
    flags: int = 0
    head: Pose = ZERO_POSE
    left_pose: Pose = ZERO_POSE
    right_pose: Pose = ZERO_POSE
    left: ControllerState = field(default_factory=ControllerState)
    right: ControllerState = field(default_factory=ControllerState)
    body: Optional[List[Pose]] = None   # 24 poses when FLAG_BODY is set


def _buttons(c: ControllerState) -> int:
    return ((BTN_AXIS_CLICK if c.axis_click else 0) | (BTN_KEY_ONE if c.key_one else 0)
            | (BTN_KEY_TWO if c.key_two else 0))


def pack(f: Frame) -> bytes:
    flags = f.flags & _KNOWN_FLAGS
    has_body = f.body is not None
    if has_body:
        if len(f.body) != BODY_JOINTS or any(len(p) != 7 for p in f.body):
            raise ValueError("body must be %d poses of 7 floats" % BODY_JOINTS)
        flags |= FLAG_BODY
    else:
        flags &= ~FLAG_BODY
    out = bytearray(_HEADER.pack(MAGIC, VERSION, flags, BODY_JOINTS if has_body else 0, 0,
                                 f.session & 0xFFFFFFFF, f.seq & 0xFFFFFFFF,
                                 f.t_send_ns & 0xFFFFFFFFFFFFFFFF, f.t_sdk_ns & 0xFFFFFFFFFFFFFFFF))
    out += _POSES.pack(*f.head, *f.left_pose, *f.right_pose)
    for c in (f.left, f.right):
        out += _CTRL.pack(c.axis_x, c.axis_y, c.trigger, c.grip, _buttons(c))
    if has_body:
        out += _BODY.pack(*[v for p in f.body for v in p])
    out += _CRC.pack(zlib.crc32(bytes(out)) & 0xFFFFFFFF)
    return bytes(out)


def unpack(data: bytes) -> Frame:
    n = len(data)
    if n < HEADER_SIZE:
        raise WireError("short", "%d bytes" % n)
    magic, version, flags, body_count, reserved, session, seq, t_send, t_sdk = _HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise WireError("bad_magic")
    if version != VERSION:
        raise WireError("bad_version", str(version))
    if flags & ~_KNOWN_FLAGS or reserved != 0:
        raise WireError("bad_flags")
    has_body = bool(flags & FLAG_BODY)
    if body_count != (BODY_JOINTS if has_body else 0):
        raise WireError("bad_body_count", str(body_count))
    expected = SIZE_WITH_BODY if has_body else SIZE_NO_BODY
    if n != expected:
        raise WireError("bad_length", "%d != %d" % (n, expected))
    (crc,) = _CRC.unpack_from(data, n - 4)
    if crc != zlib.crc32(data[:n - 4]) & 0xFFFFFFFF:
        raise WireError("bad_crc")
    pos = HEADER_SIZE
    p = _POSES.unpack_from(data, pos)
    pos += _POSES.size
    ctrls = []
    for _ in range(2):
        ax, ay, trig, grip, btn = _CTRL.unpack_from(data, pos)
        pos += _CTRL.size
        if btn & ~_KNOWN_BTNS:
            raise WireError("bad_buttons")
        ctrls.append(ControllerState(ax, ay, trig, grip, bool(btn & BTN_AXIS_CLICK),
                                     bool(btn & BTN_KEY_ONE), bool(btn & BTN_KEY_TWO)))
    body = None
    if has_body:
        b = _BODY.unpack_from(data, pos)
        body = [tuple(b[i * 7:i * 7 + 7]) for i in range(BODY_JOINTS)]
    return Frame(session=session, seq=seq, t_send_ns=t_send, t_sdk_ns=t_sdk, flags=flags,
                 head=tuple(p[0:7]), left_pose=tuple(p[7:14]), right_pose=tuple(p[14:21]),
                 left=ctrls[0], right=ctrls[1], body=body)
