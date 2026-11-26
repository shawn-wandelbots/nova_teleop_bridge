"""Hand-rolled protobuf codec for the AgiBot A3 teleop input messages.

The robot's system Python has no ``protobuf`` package, so (like the robot's own
helper scripts) the wire format is produced by hand.  Only what is needed for

    aimdk.protocol.XRFrameData       (topic /remote/vr_data)
    aimdk.protocol.VRButtonChannel   (topic /remote/vr_button)

and, further down, aimdk.protocol.RobotFrameData (what a headset sends to the robot's
own teleop input port) are implemented.  Schemas (proto3, recovered from the robot's binaries):

    XRFrameData      { uint64 timestamp=1; repeated Joint left_hand=2;
                       repeated Joint right_hand=3; repeated Joint body_data=4;
                       repeated ControllerData controller_raw_datas=5; }
    Joint            { int32 joint_code_name=1; Vector3 position=2;
                       Quaternion orientation=3; }
    Vector3          { float x=1; float y=2; float z=3; }
    Quaternion       { float x=1; float y=2; float z=3; float w=4; }
    ControllerData   { int32 controller_code_name=1; bool axis_click=2;
                       float axis_x=3; float axis_y=4; float index_trig=5;
                       float hand_trig=6; bool key_one=7; bool key_two=8; }
    VRButtonChannel  { uint64 timestamp=1;
                       repeated VRControllerButton controller_raw_datas=2;
                       ExternalDevice external_device=3; }

Encoding style: a real PICO stream writes *every* scalar, including zeros
(``08 00`` for joint code 0), i.e. proto2-style explicit presence.  That is
valid input for a proto3 parser too, so ``explicit=True`` (the default) is used
everywhere; it makes decode -> encode byte-identical to recorded PICO frames.
``explicit=False`` gives canonical proto3 output (zero scalars omitted).

Stdlib only.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ---- names ---------------------------------------------------------------------------
BODY_JOINT_NAMES = (
    "PELVIS", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE", "SPINE2",
    "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT", "NECK",
    "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "BODY_LEFT_WRIST", "BODY_RIGHT_WRIST",
    "BODY_LEFT_HAND", "BODY_RIGHT_HAND",
)
BODY_JOINT_COUNT = len(BODY_JOINT_NAMES)  # 24

EXTERNAL_DEVICE_UNKNOWN = 0
EXTERNAL_DEVICE_YUANJI_APP = 1
EXTERNAL_DEVICE_GENIELINK = 2
EXTERNAL_DEVICE_NAMES = {"UNKNOWN": 0, "YUANJI_APP": 1, "GENIELINK": 2}

XR_FRAME_TYPE = "aimdk.protocol.XRFrameData"
VR_BUTTON_TYPE = "aimdk.protocol.VRButtonChannel"


def ros_topic_name(topic: str, pb_type: str) -> str:
    """AimRT ros2 backend topic naming: ``<topic>/pb_3A<type with '.' -> '_2E'>``."""
    return "%s/pb_3A%s" % (topic, pb_type.replace(".", "_2E"))


# ---- data model ----------------------------------------------------------------------
@dataclass
class Joint:
    code: int = 0
    pos: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)            # x, y, z
    quat: Optional[Tuple[float, float, float, float]] = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w


@dataclass
class Controller:
    code: int = 0            # controller_code_name (0 = left, 1 = right on the A3)
    axis_click: bool = False
    axis_x: float = 0.0
    axis_y: float = 0.0
    index_trig: float = 0.0
    hand_trig: float = 0.0
    key_one: bool = False    # A (right) / X (left)
    key_two: bool = False    # B (right) / Y (left)


@dataclass
class XRFrame:
    timestamp: int = 0
    left_hand: List[Joint] = field(default_factory=list)
    right_hand: List[Joint] = field(default_factory=list)
    body: List[Joint] = field(default_factory=list)
    controllers: List[Controller] = field(default_factory=list)


@dataclass
class VRButtons:
    timestamp: int = 0
    controllers: List[Controller] = field(default_factory=list)
    external_device: int = EXTERNAL_DEVICE_UNKNOWN


class ProtoError(ValueError):
    pass


# ---- low-level encode ----------------------------------------------------------------
_F32 = struct.Struct("<f")


def _varint(n: int) -> bytes:
    n &= (1 << 64) - 1
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _key(fieldno: int, wire: int) -> bytes:
    return _varint(fieldno << 3 | wire)


def _ld(fieldno: int, payload: bytes) -> bytes:
    return _key(fieldno, 2) + _varint(len(payload)) + payload


def _f32(fieldno: int, v: float, explicit: bool) -> bytes:
    packed = _F32.pack(v)
    if not explicit and packed == b"\x00\x00\x00\x00":
        return b""
    return _key(fieldno, 5) + packed


def _vint(fieldno: int, v: int, explicit: bool) -> bytes:
    if not explicit and not v:
        return b""
    return _key(fieldno, 0) + _varint(int(v))


def _encode_joint(j: Joint, explicit: bool) -> bytes:
    out = _vint(1, j.code, explicit)
    if j.pos is not None:
        out += _ld(2, b"".join(_f32(i + 1, v, explicit) for i, v in enumerate(j.pos)))
    if j.quat is not None:
        out += _ld(3, b"".join(_f32(i + 1, v, explicit) for i, v in enumerate(j.quat)))
    return out


def _encode_controller(c: Controller, explicit: bool) -> bytes:
    return (_vint(1, c.code, explicit)
            + _vint(2, 1 if c.axis_click else 0, explicit)
            + _f32(3, c.axis_x, explicit)
            + _f32(4, c.axis_y, explicit)
            + _f32(5, c.index_trig, explicit)
            + _f32(6, c.hand_trig, explicit)
            + _vint(7, 1 if c.key_one else 0, explicit)
            + _vint(8, 1 if c.key_two else 0, explicit))


def encode_xr_frame(f: XRFrame, explicit: bool = True) -> bytes:
    out = bytearray(_vint(1, f.timestamp, explicit))
    for j in f.left_hand:
        out += _ld(2, _encode_joint(j, explicit))
    for j in f.right_hand:
        out += _ld(3, _encode_joint(j, explicit))
    for j in f.body:
        out += _ld(4, _encode_joint(j, explicit))
    for c in f.controllers:
        out += _ld(5, _encode_controller(c, explicit))
    return bytes(out)


def encode_vr_buttons(b: VRButtons, explicit: bool = True) -> bytes:
    out = bytearray(_vint(1, b.timestamp, explicit))
    for c in b.controllers:
        out += _ld(2, _encode_controller(c, explicit))
    # external_device: UNKNOWN (0) is omitted unless explicit, like any proto3 enum.
    out += _vint(3, b.external_device, explicit)
    return bytes(out)


# ---- low-level decode ----------------------------------------------------------------
def _read_varint(buf: bytes, pos: int) -> Tuple[int, int]:
    result = 0
    shift = 0
    while True:
        if pos >= len(buf):
            raise ProtoError("truncated varint")
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, pos
        shift += 7
        if shift > 63:
            raise ProtoError("varint too long")


def _fields(buf: bytes):
    """Yield (field_number, wire_type, value) for one message body."""
    pos = 0
    n = len(buf)
    while pos < n:
        key, pos = _read_varint(buf, pos)
        fieldno, wire = key >> 3, key & 7
        if fieldno == 0:
            raise ProtoError("field number 0")
        if wire == 0:
            val, pos = _read_varint(buf, pos)
        elif wire == 1:
            if pos + 8 > n:
                raise ProtoError("truncated fixed64")
            val = buf[pos:pos + 8]
            pos += 8
        elif wire == 2:
            ln, pos = _read_varint(buf, pos)
            if pos + ln > n:
                raise ProtoError("truncated length-delimited field")
            val = buf[pos:pos + ln]
            pos += ln
        elif wire == 5:
            if pos + 4 > n:
                raise ProtoError("truncated fixed32")
            val = buf[pos:pos + 4]
            pos += 4
        else:
            raise ProtoError("unsupported wire type %d" % wire)
        yield fieldno, wire, val


def _signed32(v: int) -> int:
    v &= (1 << 64) - 1
    if v >= 1 << 63:
        v -= 1 << 64
    return v


def _decode_floats(buf: bytes, count: int, defaults) -> tuple:
    vals = list(defaults)
    for fieldno, wire, val in _fields(buf):
        if wire == 5 and 1 <= fieldno <= count:
            vals[fieldno - 1] = _F32.unpack(val)[0]
    return tuple(vals)


def _decode_joint(buf: bytes) -> Joint:
    j = Joint(code=0, pos=None, quat=None)
    for fieldno, wire, val in _fields(buf):
        if fieldno == 1 and wire == 0:
            j.code = _signed32(val)
        elif fieldno == 2 and wire == 2:
            j.pos = _decode_floats(val, 3, (0.0, 0.0, 0.0))
        elif fieldno == 3 and wire == 2:
            # proto default for an absent w is 0.0 (not 1.0): report what is on the wire
            j.quat = _decode_floats(val, 4, (0.0, 0.0, 0.0, 0.0))
    return j


def _decode_controller(buf: bytes) -> Controller:
    c = Controller()
    for fieldno, wire, val in _fields(buf):
        if wire == 0:
            if fieldno == 1:
                c.code = _signed32(val)
            elif fieldno == 2:
                c.axis_click = bool(val)
            elif fieldno == 7:
                c.key_one = bool(val)
            elif fieldno == 8:
                c.key_two = bool(val)
        elif wire == 5:
            v = _F32.unpack(val)[0]
            if fieldno == 3:
                c.axis_x = v
            elif fieldno == 4:
                c.axis_y = v
            elif fieldno == 5:
                c.index_trig = v
            elif fieldno == 6:
                c.hand_trig = v
    return c


def decode_xr_frame(buf: bytes) -> XRFrame:
    f = XRFrame()
    for fieldno, wire, val in _fields(buf):
        if fieldno == 1 and wire == 0:
            f.timestamp = val
        elif wire == 2 and fieldno == 2:
            f.left_hand.append(_decode_joint(val))
        elif wire == 2 and fieldno == 3:
            f.right_hand.append(_decode_joint(val))
        elif wire == 2 and fieldno == 4:
            f.body.append(_decode_joint(val))
        elif wire == 2 and fieldno == 5:
            f.controllers.append(_decode_controller(val))
    return f


def decode_vr_buttons(buf: bytes) -> VRButtons:
    b = VRButtons()
    for fieldno, wire, val in _fields(buf):
        if fieldno == 1 and wire == 0:
            b.timestamp = val
        elif fieldno == 2 and wire == 2:
            b.controllers.append(_decode_controller(val))
        elif fieldno == 3 and wire == 0:
            b.external_device = val
    return b


# ======================================================================================
# aimdk.protocol.RobotFrameData: one message per UDP datagram to the robot's teleop input
# port (hal_remote).  Schema (proto3 with explicit presence, from the robot's own binary):
#
#   RobotFrameData  { uint64 timestamp=1 (unix ms); uint64 frame_index=2;
#                     repeated Joint left_hand=100; repeated Joint right_hand=101;
#                     repeated Joint body_data=102; repeated DeviceInputData device_inputs=103;
#                     Vector3 chassis_movement=104; }
#   Joint           { joint_code_types joint_code=1; PointPose point_pose=2; }
#   PointPose       { Vector3 position=1; Quaternion quaternion=2; uint32 quaternion_compressed=3; }
#   DeviceInputData { DeviceType device_type=1 (0 left, 1 right, 2 head); bool axis_click=2;
#                     float axis_x=3; float axis_y=4; float index_trig=5; float hand_trig=6;
#                     bool key_one=7; bool key_two=8; PointPose point_pose=9; }
#
# quaternion_compressed ("smallest three", worked out from recorded traffic):
#   bits 31..30  index of the component with the largest magnitude (0=x 1=y 2=z 3=w); it is
#                left out and taken as positive
#   3 x 10 bits  the other components in x,y,z,w order: 1 sign bit (1 = negative) and a
#                9-bit magnitude, 511 = 1/sqrt(2)
# ======================================================================================
DEVICE_LEFT, DEVICE_RIGHT, DEVICE_HEAD = 0, 1, 2
_Q_SCALE = 0.7071067811865476
QUAT_MODES = ("compressed", "plain", "both")


@dataclass
class DeviceInput:
    device_type: int = DEVICE_LEFT
    axis_click: bool = False
    axis_x: float = 0.0
    axis_y: float = 0.0
    index_trig: float = 0.0
    hand_trig: float = 0.0
    key_one: bool = False
    key_two: bool = False
    pos: Optional[Tuple[float, float, float]] = None
    quat: Optional[Tuple[float, float, float, float]] = None   # x, y, z, w


@dataclass
class RobotFrame:
    timestamp: int = 0
    frame_index: int = 0
    left_hand: List[Joint] = field(default_factory=list)
    right_hand: List[Joint] = field(default_factory=list)
    body: List[Joint] = field(default_factory=list)
    devices: List[DeviceInput] = field(default_factory=list)
    chassis: Optional[Tuple[float, float, float]] = None


def compress_quat(q) -> int:
    """x, y, z, w -> the 32-bit packed form described above."""
    n = sum(v * v for v in q) ** 0.5
    if not n > 0.0:
        raise ProtoError("zero quaternion")
    q = [v / n for v in q]
    big = max(range(4), key=lambda i: abs(q[i]))
    if q[big] < 0.0:
        q = [-v for v in q]
    out = big
    for i in range(4):
        if i == big:
            continue
        mag = min(511, int(round(abs(q[i]) / _Q_SCALE * 511.0)))
        out = out << 10 | ((0x200 if q[i] < 0.0 and mag else 0) | mag)
    return out


def decompress_quat(u: int) -> Tuple[float, float, float, float]:
    big = (u >> 30) & 3
    parts = [(u >> 20) & 0x3FF, (u >> 10) & 0x3FF, u & 0x3FF]
    vals = [(-1.0 if p & 0x200 else 1.0) * (p & 0x1FF) / 511.0 * _Q_SCALE for p in parts]
    rest = max(0.0, 1.0 - sum(v * v for v in vals)) ** 0.5
    q = []
    for i in range(4):
        q.append(rest if i == big else vals.pop(0))
    return tuple(q)


def _encode_point_pose(pos, quat, quat_mode: str) -> bytes:
    out = b""
    if pos is not None:
        out += _ld(1, b"".join(_f32(i + 1, v, True) for i, v in enumerate(pos)))
    if quat is not None:
        if quat_mode in ("plain", "both"):
            out += _ld(2, b"".join(_f32(i + 1, v, True) for i, v in enumerate(quat)))
        if quat_mode in ("compressed", "both"):
            out += _vint(3, quat if isinstance(quat, int) else compress_quat(quat), True)
    return out


def encode_robot_frame(f: RobotFrame, quat_mode: str = "compressed") -> bytes:
    """Every scalar is written, zeros included, exactly as the vendor headset app does.
    A quaternion given as an int is taken as already packed (used to re-encode captures)."""
    if quat_mode not in QUAT_MODES:
        raise ValueError("quat_mode must be one of %s" % (QUAT_MODES,))
    out = bytearray(_vint(1, f.timestamp, True) + _vint(2, f.frame_index, True))
    for fieldno, joints in ((100, f.left_hand), (101, f.right_hand), (102, f.body)):
        for j in joints:
            out += _ld(fieldno, _vint(1, j.code, True) + _ld(2, _encode_point_pose(j.pos, j.quat, quat_mode)))
    for d in f.devices:
        out += _ld(103, _vint(1, d.device_type, True)
                   + _vint(2, 1 if d.axis_click else 0, True)
                   + _f32(3, d.axis_x, True) + _f32(4, d.axis_y, True)
                   + _f32(5, d.index_trig, True) + _f32(6, d.hand_trig, True)
                   + _vint(7, 1 if d.key_one else 0, True) + _vint(8, 1 if d.key_two else 0, True)
                   + _ld(9, _encode_point_pose(d.pos, d.quat, quat_mode)))
    if f.chassis is not None:
        out += _ld(104, b"".join(_f32(i + 1, v, True) for i, v in enumerate(f.chassis)))
    return bytes(out)


def _decode_point_pose(buf: bytes, keep_packed: bool):
    pos = quat = None
    for fieldno, wire, val in _fields(buf):
        if fieldno == 1 and wire == 2:
            pos = _decode_floats(val, 3, (0.0, 0.0, 0.0))
        elif fieldno == 2 and wire == 2:
            quat = _decode_floats(val, 4, (0.0, 0.0, 0.0, 0.0))
        elif fieldno == 3 and wire == 0 and quat is None:
            quat = val if keep_packed else decompress_quat(val)
    return pos, quat


def decode_robot_frame(buf: bytes, keep_packed: bool = False) -> RobotFrame:
    """keep_packed=True leaves packed quaternions as ints, so encode gives the same bytes back."""
    f = RobotFrame()
    for fieldno, wire, val in _fields(buf):
        if wire == 0 and fieldno == 1:
            f.timestamp = val
        elif wire == 0 and fieldno == 2:
            f.frame_index = val
        elif wire == 2 and fieldno in (100, 101, 102):
            j = Joint(code=0, pos=None, quat=None)
            for ff, ww, vv in _fields(val):
                if ff == 1 and ww == 0:
                    j.code = _signed32(vv)
                elif ff == 2 and ww == 2:
                    j.pos, j.quat = _decode_point_pose(vv, keep_packed)
            {100: f.left_hand, 101: f.right_hand, 102: f.body}[fieldno].append(j)
        elif wire == 2 and fieldno == 103:
            d = DeviceInput()
            for ff, ww, vv in _fields(val):
                if ww == 0:
                    if ff == 1:
                        d.device_type = _signed32(vv)
                    elif ff == 2:
                        d.axis_click = bool(vv)
                    elif ff == 7:
                        d.key_one = bool(vv)
                    elif ff == 8:
                        d.key_two = bool(vv)
                elif ww == 5 and 3 <= ff <= 6:
                    setattr(d, ("axis_x", "axis_y", "index_trig", "hand_trig")[ff - 3], _F32.unpack(vv)[0])
                elif ww == 2 and ff == 9:
                    d.pos, d.quat = _decode_point_pose(vv, keep_packed)
            f.devices.append(d)
        elif wire == 2 and fieldno == 104:
            f.chassis = _decode_floats(val, 3, (0.0, 0.0, 0.0))
    return f
