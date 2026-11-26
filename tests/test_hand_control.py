"""Finger mapping (pure logic; no ROS, no robot)."""
import os
import unittest

from robot import hand_control as hc
from tests.util import ROOT

CFG = os.path.join(ROOT, "config", "hand.json")


class TestFingerMap(unittest.TestCase):
    def setUp(self):
        self.cfg = hc.load_config(CFG)
        self.cfg["max_speed_per_s"] = 40000        # fast enough not to matter unless a test sets it

    def run_to(self, m, trig_l, grip_l, trig_r, grip_r, t0=0.0, seconds=2.0):
        out = None
        for i in range(int(seconds * 100) + 1):
            out = m.update(t0 + i * 0.01, trig_l, grip_l, trig_r, grip_r)
        return out

    def test_shipped_config_is_not_marked_verified(self):
        self.assertFalse(hc.load_config(CFG)["verified"])

    def test_released_trigger_is_the_open_pose(self):
        out = self.run_to(hc.FingerMap(self.cfg, "two"), 0, 1, 0, 1)
        self.assertEqual(out, [float(v) for v in self.cfg["left"]["open"] + self.cfg["right"]["open"]])

    def test_two_fingers_only_thumb_and_index_move(self):
        out = self.run_to(hc.FingerMap(self.cfg, "two"), 1, 1, 1, 1)
        for side, off in (("left", 0), ("right", 10)):
            o, c = self.cfg[side]["open"], self.cfg[side]["closed"]
            for i in range(10):
                want = c[i] if i in (0, 1, 2, 3, 4) else o[i]
                self.assertEqual(out[off + i], want, (side, hc.MOTORS[i]))

    def test_three_fingers_adds_the_middle_finger(self):
        m = hc.FingerMap(self.cfg, "three")
        self.assertEqual(m.moving, [0, 1, 2, 3, 4, 5])
        out = self.run_to(m, 1, 1, 0, 1)
        self.assertEqual(out[5], self.cfg["left"]["closed"][5])
        for i in (6, 7, 8, 9):                      # ring and little finger stay open
            self.assertEqual(out[i], self.cfg["left"]["open"][i])
        self.assertEqual(out[10:], [float(v) for v in self.cfg["right"]["open"]])   # other hand untouched

    def test_amount_limits_how_far_they_close(self):
        out = self.run_to(hc.FingerMap(self.cfg, "two", amount=0.5), 1, 1, 0, 0)
        o, c = self.cfg["left"]["open"], self.cfg["left"]["closed"]
        self.assertAlmostEqual(out[4], (o[4] + c[4]) / 2)

    def test_fingers_hold_when_the_grip_is_released(self):
        m = hc.FingerMap(self.cfg, "two")
        closed = self.run_to(m, 1, 1, 0, 1)
        held = self.run_to(m, 0, 0, 0, 0, t0=10.0)          # trigger and grip let go
        self.assertEqual(held, closed)
        opened = self.run_to(m, 0, 1, 0, 1, t0=20.0)        # grip again, trigger released -> opens
        self.assertEqual(opened[:10], [float(v) for v in self.cfg["left"]["open"]])

    def test_speed_is_limited_and_starts_from_the_real_position(self):
        self.cfg["max_speed_per_s"] = 1000
        m = hc.FingerMap(self.cfg, "two")
        m.seed(self.cfg["left"]["open"], self.cfg["right"]["open"])
        a = m.update(0.0, 1, 1, 0, 0)
        b = m.update(0.01, 1, 1, 0, 0)
        self.assertEqual(a[:10], [float(v) for v in self.cfg["left"]["open"]])      # first call: no time passed
        for i in range(10):
            self.assertLessEqual(abs(b[i] - a[i]), 10.0 + 1e-9)
        c = m.update(5.0, 1, 1, 0, 0)                                               # a long gap is capped at 0.1 s
        for i in range(10):
            self.assertLessEqual(abs(c[i] - b[i]), 100.0 + 1e-9)

    def test_bad_input_is_refused(self):
        with self.assertRaises(ValueError):
            hc.FingerMap(self.cfg, "four")
        with self.assertRaises(ValueError):
            hc.fingers_for(["thumb", "toe"])
        out = self.run_to(hc.FingerMap(self.cfg, "all"), 7.0, 1, -3.0, 1)           # trigger out of range is clamped
        self.assertEqual(out[:10], [float(v) for v in self.cfg["left"]["closed"]])
        self.assertEqual(out[10:], [float(v) for v in self.cfg["right"]["open"]])

    def test_unverified_config_cannot_go_live(self):
        with self.assertRaises(SystemExit):
            hc.HandPublisher(hc.load_config(CFG), "two")


if __name__ == "__main__":
    unittest.main()
