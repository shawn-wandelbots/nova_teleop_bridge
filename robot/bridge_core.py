"""Pure logic of the robot-side bridge: validate -> transform -> encode.  No sockets, no ROS.

Everything here is deterministic given (datagram, source ip, monotonic time), which is what
the unit tests drive.
"""
from __future__ import annotations

import math
import time
from collections import Counter
from dataclasses import dataclass
from typing import List, Optional, Tuple

from robot import a3_proto
from teleop_common import wire
from teleop_common.transform import FrameTransform, q_mul, q_norm, q_rotate

TIMESTAMP_MODES = ("unix_ms", "unix_us", "unix_ns", "sender_ns", "sdk_ns", "const_u32max")


@dataclass
class Limits:
    watchdog_ms: float = 100.0        # no valid frame for this long -> stop publishing
    resume_frames: int = 3            # consecutive valid frames needed to resume after a stop
    resync_ms: float = 600.0          # silence after which a new session / pose baseline is accepted
    max_abs_position_m: float = 10.0  # any |coordinate| above this is implausible
    max_speed_mps: float = 15.0       # per-joint speed limit between accepted frames
    min_jump_m: float = 0.25          # jumps below this are always tolerated (tracker noise, dup frames)
    quat_norm_tol: float = 0.1        # |norm - 1| above this -> reject
    require_body: bool = True         # drop frames without the 24 body joints
    frozen_body_frames: int = 50      # this many frames with a bit-identical body = tracking is frozen
    allow_synthetic: bool = True      # accept frames flagged as coming from the fake SDK


@dataclass
class Output:
    xr_frame: bytes       # encoded aimdk.protocol.XRFrameData
    vr_button: bytes      # encoded aimdk.protocol.VRButtonChannel
    seq: int
    t_send_ns: int
    robot_frame: Optional[bytes] = None   # encoded aimdk.protocol.RobotFrameData (robot_frame mode)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class BridgeCore:
    """Feed datagrams with :meth:`handle`, call :meth:`tick` regularly for the watchdog."""

    def __init__(self, config: dict, limits: Optional[Limits] = None, sender_ip: Optional[str] = None,
                 wall_clock_ns=time.time_ns, log=None, robot_frame: bool = False):
        self.limits = limits or Limits()
        self.transform = FrameTransform.from_config(config)
        c = config.get("controllers", {})
        self.left_code = int(c.get("left_code", 0))
        self.right_code = int(c.get("right_code", 1))
        if self.left_code == self.right_code:
            raise ValueError("left_code and right_code must differ")
        self.axis_x_sign = float(c.get("axis_x_sign", 1.0))
        self.axis_y_sign = float(c.get("axis_y_sign", 1.0))
        self.include_in_xr = bool(c.get("include_in_xr_frame", True))
        o = config.get("output", {})
        self.timestamp_mode = o.get("timestamp_mode", "unix_ms")
        if self.timestamp_mode not in TIMESTAMP_MODES:
            raise ValueError("timestamp_mode must be one of %s" % (TIMESTAMP_MODES,))
        ext = o.get("external_device", "UNKNOWN")
        if ext not in a3_proto.EXTERNAL_DEVICE_NAMES:
            raise ValueError("external_device must be one of %s" % sorted(a3_proto.EXTERNAL_DEVICE_NAMES))
        self.external_device = a3_proto.EXTERNAL_DEVICE_NAMES[ext]
        self.explicit = bool(o.get("explicit_defaults", True))
        self.pose_joints = bool(o.get("controller_pose_as_hand_joint", False))
        # RobotFrameData output (what the robot's own teleop input port takes), see a3_proto.
        self.robot_frame = bool(robot_frame)
        self.quat_mode = o.get("robot_frame_quaternion", "compressed")
        if self.quat_mode not in a3_proto.QUAT_MODES:
            raise ValueError("robot_frame_quaternion must be one of %s" % (a3_proto.QUAT_MODES,))
        # "first_head": positions are measured from where the head was when the stream started
        # (the vendor headset app does this: head height is z = 0).  "fixed": use them as they come.
        # "body": the reference is the operator's own body: positions are measured from the
        # tracked head joint and turned so that the operator faces +x.  It is taken when the
        # stream starts and again every time X+Y+A+B is pressed with both grips released (the
        # robot's calibration gesture).  Needs body tracking.  Head and controller positions are
        # then taken from the body's head and hand joints, because the headset reports the body
        # and the devices in frames that do not always agree.
        self.origin_mode = o.get("origin", "fixed")
        if self.origin_mode not in ("fixed", "first_head", "body"):
            raise ValueError("origin must be 'fixed', 'first_head' or 'body'")
        self._origin: Optional[Tuple[float, float, float]] = None
        self._yaw_q = (0.0, 0.0, 0.0, 1.0)       # rotation that turns the operator's heading to +x
        self._combo_was_down = False
        self._last_body = None
        self._same_body = 0
        self._frame_index = 0
        self._last_ts = 0

        self.sender_ip = sender_ip          # None -> lock onto the first sender of a fully valid datagram
        self.sender_locked_by_first_seen = False
        self._wall_ns = wall_clock_ns
        self._log = log or (lambda msg: None)

        self.stats = Counter()
        self.session: Optional[int] = None
        self.last_seq: Optional[int] = None
        self.last_valid_t: Optional[float] = None   # monotonic time of the last accepted frame
        self.active = False                          # True while frames are being passed on
        self._good_run = 0
        self._baseline: Optional[List[Tuple[float, float, float]]] = None
        self._baseline_t_ns = 0
        self._jump_dt: Optional[float] = None
        self.last_reason = ""

    # ---- watchdog ---------------------------------------------------------------------
    def tick(self, now: float) -> bool:
        """Returns True exactly when the watchdog trips (active -> stopped)."""
        if self.active and self.last_valid_t is not None \
                and (now - self.last_valid_t) * 1000.0 > self.limits.watchdog_ms:
            self.active = False
            self._good_run = 0
            self.stats["watchdog_trips"] += 1
            self._log("WATCHDOG: no valid frame for %.0f ms -> publishing stopped"
                      % ((now - self.last_valid_t) * 1000.0))
            return True
        return False

    def _reject(self, reason: str) -> None:
        self.stats["reject_" + reason] += 1
        self.last_reason = reason
        self._good_run = 0
        return None

    # ---- main entry -------------------------------------------------------------------
    def handle(self, data: bytes, src_ip: str, now: float) -> Optional[Output]:
        """Validate one datagram. Returns an Output to publish, or None (reason in last_reason)."""
        self.stats["rx"] += 1
        self.tick(now)
        if self.sender_ip is not None and src_ip != self.sender_ip:
            return self._reject("bad_source")
        try:
            f = wire.unpack(data)
        except wire.WireError as e:
            return self._reject(e.reason)
        if self.sender_ip is None:
            self.sender_ip = src_ip
            self.sender_locked_by_first_seen = True
            self._log("sender locked to first-seen address %s" % src_ip)

        stale_stream = self.last_valid_t is None or (now - self.last_valid_t) * 1000.0 > self.limits.resync_ms
        if self.session is None or f.session != self.session:
            if self.session is not None and not stale_stream:
                return self._reject("foreign_session")
            if self.session is not None:
                self._log("new sender session %08x accepted after silence" % f.session)
            self.session, self.last_seq = f.session, None
        if self.last_seq is not None:
            diff = (f.seq - self.last_seq) & 0xFFFFFFFF
            if diff == 0 or diff >= 0x80000000:
                return self._reject("stale_seq")
            if diff > 1:
                self.stats["seq_gaps"] += diff - 1
        self.last_seq = f.seq

        if f.flags & wire.FLAG_SYNTHETIC:
            self.stats["synthetic"] += 1
            if not self.limits.allow_synthetic:
                return self._reject("synthetic")
        if f.body is None and self.limits.require_body:
            return self._reject("no_body")
        if self.robot_frame and not f.flags & wire.FLAG_HEAD:
            return self._reject("no_head")
        # A living body never gives the very same 24 poses twice in a row for long (tracking
        # noise alone prevents it).  Bit-identical frames mean the headset's body tracking has
        # stopped updating: passing that on would send the robot to one stale pose.
        if f.body is not None and self.limits.frozen_body_frames > 0:
            if f.body == self._last_body:
                self._same_body += 1
            else:
                self._last_body, self._same_body = f.body, 0
            if self._same_body >= self.limits.frozen_body_frames:
                return self._reject("body_frozen")

        # ---- content checks on everything that will be emitted --------------------------
        used = []
        if f.body is not None:
            used.extend(f.body)
        if self.pose_joints or self.robot_frame:
            if f.flags & wire.FLAG_LEFT:
                used.append(f.left_pose)
            if f.flags & wire.FLAG_RIGHT:
                used.append(f.right_pose)
        if self.robot_frame or (self.origin_mode == "first_head" and f.flags & wire.FLAG_HEAD):
            used.append(f.head)
        ctrl_in = []
        if f.flags & wire.FLAG_LEFT:
            ctrl_in.append((self.left_code, f.left))
        if f.flags & wire.FLAG_RIGHT:
            ctrl_in.append((self.right_code, f.right))
        for pose in used:
            if not all(math.isfinite(v) for v in pose):
                return self._reject("nonfinite")
        for _, c in ctrl_in:
            if not all(math.isfinite(v) for v in (c.axis_x, c.axis_y, c.trigger, c.grip)):
                return self._reject("nonfinite")
        for pose in used:
            if abs(q_norm(pose[3:7]) - 1.0) > self.limits.quat_norm_tol:
                return self._reject("bad_quat")

        # ---- origin ---------------------------------------------------------------------
        reanchored = False
        if self.origin_mode == "body":
            if f.body is None:
                return self._reject("no_body")
            raw = [self.transform.pose(p)[0] for p in f.body]
            combo = (f.flags & wire.FLAG_LEFT and f.flags & wire.FLAG_RIGHT
                     and f.left.key_one and f.left.key_two and f.right.key_one and f.right.key_two)
            grips_free = f.left.grip < 0.1 and f.right.grip < 0.1
            pressed = bool(combo) and not self._combo_was_down and grips_free
            self._combo_was_down = bool(combo)
            if stale_stream or self._origin is None or pressed:
                # heading: the direction the shoulders and hips face, flattened to the floor
                lx = raw[16][0] - raw[17][0] + raw[1][0] - raw[2][0]
                ly = raw[16][1] - raw[17][1] + raw[1][1] - raw[2][1]
                if math.hypot(lx, ly) < 0.05:
                    return self._reject("no_heading")
                yaw = math.atan2(-lx, ly)
                self._yaw_q = (0.0, 0.0, math.sin(-yaw / 2.0), math.cos(-yaw / 2.0))
                self._origin = raw[15]
                self._baseline = None            # the frame moved on purpose: not a tracking jump
                reanchored = True
                self.stats["reanchors"] += 1
                self._log("reference set to the operator's body (%s): heading %.0f deg"
                          % ("X+Y+A+B" if pressed else "stream start", math.degrees(yaw)))
            ox, oy, oz = self._origin
        elif self.origin_mode == "first_head":
            if stale_stream or self._origin is None:
                if not f.flags & wire.FLAG_HEAD:
                    return self._reject("no_head")
                self._origin = self.transform.pose(f.head)[0]
            ox, oy, oz = self._origin
        else:
            ox = oy = oz = 0.0

        yaw_q = self._yaw_q if self.origin_mode == "body" else None

        def tf(pose):
            p, q = self.transform.pose(pose)
            p = (p[0] - ox, p[1] - oy, p[2] - oz)
            if yaw_q is not None:
                p, q = q_rotate(yaw_q, p), q_mul(yaw_q, q)
            return p, q

        body_joints: List[a3_proto.Joint] = []
        positions: List[Tuple[float, float, float]] = []
        if f.body is not None:
            for code, pose in enumerate(f.body):
                p, q = tf(pose)
                positions.append(p)
                body_joints.append(a3_proto.Joint(code=code, pos=p, quat=q))
        lim = self.limits.max_abs_position_m
        for p in positions:
            if abs(p[0]) > lim or abs(p[1]) > lim or abs(p[2]) > lim:
                return self._reject("out_of_range")

        # ---- implausible jump vs. the last accepted frame --------------------------------
        if positions:
            if self._baseline is not None and not stale_stream and len(self._baseline) == len(positions):
                # Allowance grows with the time since the last accepted frame (packet loss), but it
                # is frozen at the first rejection: a pose that stays "teleported" must not become
                # acceptable just because time passes. It is only re-accepted after resync_ms of
                # silence on our outputs, by which time the robot has dropped the session itself.
                if self._jump_dt is None:
                    dt = (f.t_send_ns - self._baseline_t_ns) / 1e9
                    self._jump_dt = _clamp(dt, 0.0, self.limits.resync_ms / 1000.0)
                allowed = max(self.limits.min_jump_m, self.limits.max_speed_mps * self._jump_dt)
                for a, b in zip(self._baseline, positions):
                    if math.dist(a, b) > allowed:
                        return self._reject("jump")
            self._jump_dt = None
            self._baseline, self._baseline_t_ns = positions, f.t_send_ns

        # ---- accepted --------------------------------------------------------------------
        self.stats["valid"] += 1
        self.last_valid_t = now
        self.last_reason = ""
        self._good_run += 1
        if not self.active:
            if self._good_run < self.limits.resume_frames:
                self.stats["held_resuming"] += 1
                self.last_reason = "resuming"
                return None
            self.active = True
            self._log("stream healthy -> publishing enabled (session %08x, seq %d)" % (f.session, f.seq))

        controllers = []
        for code, c in ctrl_in:
            trig = _clamp(c.trigger, 0.0, 1.0)
            grip = _clamp(c.grip, 0.0, 1.0)
            ax = _clamp(c.axis_x * self.axis_x_sign, -1.0, 1.0)
            ay = _clamp(c.axis_y * self.axis_y_sign, -1.0, 1.0)
            if (trig, grip) != (c.trigger, c.grip) or abs(c.axis_x) > 1.0 or abs(c.axis_y) > 1.0:
                self.stats["clamped"] += 1
            controllers.append(a3_proto.Controller(code=code, axis_click=c.axis_click, axis_x=ax, axis_y=ay,
                                                   index_trig=trig, hand_trig=grip,
                                                   key_one=c.key_one, key_two=c.key_two))
        controllers.sort(key=lambda c: c.code)

        ts = self._timestamp(f)
        xr = a3_proto.XRFrame(timestamp=ts, body=body_joints,
                              controllers=list(controllers) if self.include_in_xr else [])
        if self.pose_joints:
            if f.flags & wire.FLAG_LEFT:
                p, q = tf(f.left_pose)
                xr.left_hand.append(a3_proto.Joint(code=0, pos=p, quat=q))
            if f.flags & wire.FLAG_RIGHT:
                p, q = tf(f.right_pose)
                xr.right_hand.append(a3_proto.Joint(code=0, pos=p, quat=q))
        btn = a3_proto.VRButtons(timestamp=ts, controllers=controllers, external_device=self.external_device)
        rf = None
        if self.robot_frame:
            devices = []
            poses = {self.left_code: f.left_pose, self.right_code: f.right_pose}
            body_pos = {j.code: j.pos for j in body_joints}
            for c in controllers:
                p, q = tf(poses[c.code])
                if self.origin_mode == "body":
                    p = body_pos[22 if c.code == self.left_code else 23]
                devices.append(a3_proto.DeviceInput(
                    device_type=a3_proto.DEVICE_LEFT if c.code == self.left_code else a3_proto.DEVICE_RIGHT,
                    axis_click=c.axis_click, axis_x=c.axis_x, axis_y=c.axis_y, index_trig=c.index_trig,
                    hand_trig=c.hand_trig, key_one=c.key_one, key_two=c.key_two, pos=p, quat=q))
            devices.sort(key=lambda d: d.device_type)
            p, q = tf(f.head)
            if self.origin_mode == "body":
                p = body_pos[15]
            devices.append(a3_proto.DeviceInput(device_type=a3_proto.DEVICE_HEAD, pos=p, quat=q))
            # The robot drops a datagram whose timestamp it has already seen: keep it rising.
            ts_ms = max(self._wall_ns() // 1_000_000, self._last_ts + 1)
            self._last_ts = ts_ms
            rf = a3_proto.encode_robot_frame(a3_proto.RobotFrame(
                timestamp=ts_ms, frame_index=self._frame_index, body=body_joints, devices=devices),
                self.quat_mode)
            self._frame_index += 1
        return Output(xr_frame=a3_proto.encode_xr_frame(xr, self.explicit),
                      vr_button=a3_proto.encode_vr_buttons(btn, self.explicit),
                      seq=f.seq, t_send_ns=f.t_send_ns, robot_frame=rf)

    def _timestamp(self, f: wire.Frame) -> int:
        m = self.timestamp_mode
        if m == "const_u32max":
            return 0xFFFFFFFF
        if m == "sender_ns":
            return f.t_send_ns
        if m == "sdk_ns":
            return f.t_sdk_ns
        ns = self._wall_ns()
        return ns // 1_000_000 if m == "unix_ms" else ns // 1_000 if m == "unix_us" else ns
