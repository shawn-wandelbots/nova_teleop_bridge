"""The ONLY module that talks to the XRoboToolkit Python SDK (``xrobotoolkit_sdk``).

Every SDK function name comes from the "sdk" section of config/transform.json, so a renamed
or differently shaped SDK call is fixed in the config (or here) and nowhere else.
Python >= 3.8, stdlib only.
"""
from __future__ import annotations

import math
import time
from typing import Optional

from teleop_common import wire

_DEFAULT_SDK = {
    "init": "init", "close": "close", "timestamp_ns": "get_time_stamp_ns",
    "headset_pose": "get_headset_pose",
    "left_pose": "get_left_controller_pose", "right_pose": "get_right_controller_pose",
    "left_trigger": "get_left_trigger", "right_trigger": "get_right_trigger",
    "left_grip": "get_left_grip", "right_grip": "get_right_grip",
    "left_axis": "get_left_axis", "right_axis": "get_right_axis",
    "left_axis_click": "get_left_axis_click", "right_axis_click": "get_right_axis_click",
    "button_a": "get_A_button", "button_b": "get_B_button",
    "button_x": "get_X_button", "button_y": "get_Y_button",
    "body_available": "is_body_data_available", "body_joints": "get_body_joints_pose",
}


class SdkError(RuntimeError):
    pass


class XrtAdapter:
    def __init__(self, sdk_module, config: Optional[dict] = None, synthetic: bool = False):
        self._sdk = sdk_module
        names = dict(_DEFAULT_SDK)
        sdk_cfg = (config or {}).get("sdk", {})
        names.update({k: v for k, v in sdk_cfg.items() if k in _DEFAULT_SDK})
        self._order = list(sdk_cfg.get("pose_order", range(7)))
        if sorted(self._order) != list(range(7)):
            raise ValueError("sdk.pose_order must be a permutation of 0..6")
        self._fn = {}
        for key, name in names.items():
            fn = getattr(sdk_module, name, None)
            if fn is None:
                raise SdkError("SDK module has no function %r (config key sdk.%s)" % (name, key))
            self._fn[key] = fn
        self._synthetic = synthetic
        self._opened = False

    def open(self) -> None:
        self._fn["init"]()
        self._opened = True

    def close(self) -> None:
        if self._opened:
            self._opened = False
            self._fn["close"]()

    # ---- helpers ------------------------------------------------------------------------
    def _pose(self, raw) -> wire.Pose:
        vals = [float(v) for v in raw]
        if len(vals) != 7:
            raise SdkError("pose has %d values, expected 7" % len(vals))
        return tuple(vals[i] for i in self._order)  # type: ignore[return-value]

    @staticmethod
    def _pose_ok(p) -> bool:
        if not all(math.isfinite(v) for v in p):
            return False
        n = math.sqrt(p[3] ** 2 + p[4] ** 2 + p[5] ** 2 + p[6] ** 2)
        return abs(n - 1.0) < 0.1          # an all-zero pose means "not tracked"

    def _controller(self, side: str, key_one: str, key_two: str) -> wire.ControllerState:
        axis = self._fn[side + "_axis"]()
        return wire.ControllerState(
            axis_x=float(axis[0]), axis_y=float(axis[1]),
            trigger=float(self._fn[side + "_trigger"]()), grip=float(self._fn[side + "_grip"]()),
            axis_click=bool(self._fn[side + "_axis_click"]()),
            key_one=bool(self._fn[key_one]()), key_two=bool(self._fn[key_two]()))

    # ---- one tick -----------------------------------------------------------------------
    def snapshot(self) -> wire.Frame:
        """Read everything once. Raises on SDK errors; never invents data."""
        f = wire.Frame(t_sdk_ns=int(self._fn["timestamp_ns"]()))
        f.head = self._pose(self._fn["headset_pose"]())
        f.left_pose = self._pose(self._fn["left_pose"]())
        f.right_pose = self._pose(self._fn["right_pose"]())
        f.left = self._controller("left", "button_x", "button_y")      # left:  key_one = X, key_two = Y
        f.right = self._controller("right", "button_a", "button_b")    # right: key_one = A, key_two = B
        flags = wire.FLAG_SYNTHETIC if self._synthetic else 0
        if self._pose_ok(f.head):
            flags |= wire.FLAG_HEAD
        # Controller inputs are forwarded only together with a tracked controller pose: an
        # untracked controller is reported as absent rather than as "everything released".
        for flag, pose, state in ((wire.FLAG_LEFT, f.left_pose, f.left), (wire.FLAG_RIGHT, f.right_pose, f.right)):
            inputs_ok = all(math.isfinite(v) for v in (state.axis_x, state.axis_y, state.trigger, state.grip))
            if self._pose_ok(pose) and inputs_ok:
                flags |= flag
        if self._fn["body_available"]():
            body = [self._pose(j) for j in self._fn["body_joints"]()]
            if len(body) == wire.BODY_JOINTS and all(self._pose_ok(p) for p in body):
                f.body = body
        f.flags = flags
        f.t_send_ns = time.time_ns()
        return f
