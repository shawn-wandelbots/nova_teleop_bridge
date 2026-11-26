"""Finger control: the index trigger closes only the chosen fingers, the rest stay open.

Pure logic first (FingerMap, no ROS), then HandPublisher which talks to the robot's hand
topics.  rclpy is imported only when HandPublisher is created.

Rules:
  * Each hand has an 'open' and a 'closed' pose of 10 motor positions (config/hand.json).
  * Trigger 0..1 moves the motors of the SELECTED fingers from open to closed, scaled by
    'amount'.  Motors of all other fingers are commanded to 'open'.
  * A hand only follows its trigger while that hand's grip is held.  When the grip is released
    the fingers keep their last commanded position (so a held object is not dropped).
  * Commanded positions change at most max_speed_per_s, so nothing snaps.
  * No input -> nothing is published.  Silence, never a made-up pose.
"""
from __future__ import annotations

import json
import time
from typing import Dict, List, Optional, Sequence

MOTORS = ("thumb_rotation_0", "thumb_wiggles_1", "thumb_bent_2", "index_wiggles_0", "index_bent_1",
          "middle_bent", "ring_wiggles_0", "ring_bent_1", "pinky_wiggles_0", "pinky_bent_1")
FINGER_MOTORS = {"thumb": (0, 1, 2), "index": (3, 4), "middle": (5,), "ring": (6, 7), "pinky": (8, 9)}
FINGERS = tuple(FINGER_MOTORS)
PRESETS = {"two": ("thumb", "index"), "three": ("thumb", "index", "middle"),
           "all": FINGERS}
SIDES = ("left", "right")
POS_MIN, POS_MAX = 0.0, 4095.0


def load_config(path: str) -> dict:
    with open(path) as fh:
        cfg = json.load(fh)
    for side in SIDES:
        for pose in ("open", "closed"):
            v = cfg[side][pose]
            if len(v) != len(MOTORS) or not all(isinstance(x, (int, float)) and POS_MIN <= x <= POS_MAX for x in v):
                raise ValueError("%s.%s must be %d numbers in 0..4095" % (side, pose, len(MOTORS)))
    if not 100 <= cfg.get("max_speed_per_s", 0) <= 40000:
        raise ValueError("max_speed_per_s must be 100..40000")
    return cfg


def fingers_for(selection) -> tuple:
    """'two' | 'three' | 'all' | a list of finger names -> tuple of finger names."""
    if isinstance(selection, str):
        if selection not in PRESETS:
            raise ValueError("finger selection must be one of %s" % sorted(PRESETS))
        return PRESETS[selection]
    sel = tuple(selection)
    if not sel or any(f not in FINGER_MOTORS for f in sel):
        raise ValueError("fingers must be from %s" % (FINGERS,))
    return sel


class FingerMap:
    """trigger + grip per hand -> 20 motor targets (left 10, right 10)."""

    def __init__(self, cfg: dict, selection="two", amount: float = 1.0):
        self.cfg = cfg
        self.amount = min(1.0, max(0.0, float(amount)))
        self.fingers = fingers_for(selection)
        self.moving = sorted(i for f in self.fingers for i in FINGER_MOTORS[f])
        self.max_speed = float(cfg["max_speed_per_s"])
        self.grip_hold = float(cfg.get("grip_hold_threshold", 0.5))
        self._target: Dict[str, List[float]] = {s: list(map(float, cfg[s]["open"])) for s in SIDES}
        self._cmd: Optional[Dict[str, List[float]]] = None
        self._t: Optional[float] = None

    def pose(self, side: str, trigger: float) -> List[float]:
        t = min(1.0, max(0.0, trigger)) * self.amount
        o, c = self.cfg[side]["open"], self.cfg[side]["closed"]
        out = [float(v) for v in o]
        for i in self.moving:
            out[i] = o[i] + (c[i] - o[i]) * t
        return out

    def seed(self, left: Sequence[float], right: Sequence[float]) -> None:
        """Start the speed limiter from where the fingers really are."""
        self._cmd = {"left": [float(v) for v in left], "right": [float(v) for v in right]}

    def update(self, now: float, trig_l: float, grip_l: float, trig_r: float, grip_r: float) -> List[float]:
        for side, trig, grip in (("left", trig_l, grip_l), ("right", trig_r, grip_r)):
            if grip >= self.grip_hold:
                self._target[side] = self.pose(side, trig)
        if self._cmd is None:
            self._cmd = {s: list(self._target[s]) for s in SIDES}
        dt = 0.0 if self._t is None else min(0.1, max(0.0, now - self._t))
        self._t = now
        step = self.max_speed * dt
        for side in SIDES:
            cur, tgt = self._cmd[side], self._target[side]
            for i in range(len(MOTORS)):
                d = tgt[i] - cur[i]
                cur[i] = tgt[i] if abs(d) <= step else cur[i] + (step if d > 0 else -step)
                cur[i] = min(POS_MAX, max(POS_MIN, cur[i]))
        return self._cmd["left"] + self._cmd["right"]


class HandPublisher:
    """Publishes finger commands. Waits for the robot's hand state first: it supplies the motor
    names and the real positions, and proves the expected hand type is fitted."""

    def __init__(self, cfg: dict, selection="two", amount: float = 1.0, log=print, require_verified=True):
        if require_verified and not cfg.get("verified"):
            raise SystemExit("finger control: config/hand.json is not verified on this robot yet "
                             "(run tools/hand_test.py first)")
        import rclpy  # noqa: PLC0415  (lazy on purpose)
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState

        self._rclpy, self._msg = rclpy, JointState
        self.cfg, self.log = cfg, log
        self.map = FingerMap(cfg, selection, amount)
        self.names: Optional[List[str]] = None
        self.state: Optional[List[float]] = None
        self.hand_type = ""
        self.count = 0
        self._period = 1.0 / float(cfg.get("publish_hz", 50))
        self._last_pub = 0.0
        self._seeded = False
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node("nova_teleop_hand")
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self._sub = self._node.create_subscription(JointState, cfg["state_topic"], self._on_state, qos)
        self._pub = self._node.create_publisher(JointState, cfg["command_topic"], qos)

    def _on_state(self, msg) -> None:
        if len(msg.name) != 2 * len(MOTORS) or len(msg.position) != 2 * len(MOTORS):
            return
        self.hand_type = msg.header.frame_id
        self.names = list(msg.name)
        self.state = [float(v) for v in msg.position]

    def spin(self, timeout: float = 0.0) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=timeout)

    def ready(self) -> bool:
        return self.names is not None and self.hand_type == self.cfg.get("expected_hand", "O10Hand")

    def other_publishers(self) -> int:
        return self._node.count_publishers(self.cfg["command_topic"]) - 1

    def publish_positions(self, positions: Sequence[float]) -> None:
        m = self._msg()
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.header.frame_id = self.hand_type
        m.name = list(self.names)
        m.position = [float(round(v)) for v in positions]
        self._pub.publish(m)
        self.count += 1

    def update(self, trig_l: float, grip_l: float, trig_r: float, grip_r: float) -> None:
        """Call once per accepted headset frame."""
        self.spin(0.0)
        if not self.ready():
            return
        now = time.monotonic()
        if not self._seeded:
            self.map.seed(self.state[:10], self.state[10:])
            self._seeded = True
            self.log("finger control: %s found, fingers %s" % (self.hand_type, "+".join(self.map.fingers)))
        if now - self._last_pub < self._period:
            return
        self._last_pub = now
        self.publish_positions(self.map.update(now, trig_l, grip_l, trig_r, grip_r))

    def close(self) -> None:
        self._node.destroy_node()
