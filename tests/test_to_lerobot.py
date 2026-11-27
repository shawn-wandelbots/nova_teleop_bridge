"""The decisions of the LeRobot converter that need no ffmpeg and no recording."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
try:
    import numpy as np
    from server import to_lerobot as tl
except ImportError:                                   # numpy missing on this computer
    np = tl = None

VPS, SPS, PPS = b"\x00\x00\x00\x01\x40\x01", b"\x00\x00\x00\x01\x42\x01", b"\x00\x00\x00\x01\x44\x01"
IDR, P = b"\x00\x00\x00\x01\x26\x01\xaa", b"\x00\x00\x00\x01\x02\x01\xbb"
KEY = VPS + SPS + PPS + IDR


def joints(t, n=2, value=None, names=None):
    t = np.asarray(t, dtype=float)
    pos = np.tile((t - 1.7e9)[:, None], (1, n)) if value is None else np.full((len(t), n), float(value))
    return {"stamp": list(t), "log": list(t + 0.001), "names": names or ["j%d" % i for i in range(n)],
            "position": pos.tolist(), "velocity": np.zeros((len(t), n)).tolist(), "effort": np.zeros((len(t), n)).tolist()}


def video(t, key_every=30, drop=()):
    t = [x for i, x in enumerate(t) if i not in drop]
    idx = [i for i in range(len(t) + len(drop)) if i not in drop]
    return {"stamp": t, "log": [x + 0.04 for x in t], "format": "h265",
            "data": [KEY if i % key_every == 0 else P for i in idx]}


@unittest.skipIf(tl is None, "numpy is not installed")
class TestConverterDecisions(unittest.TestCase):
    T0 = 1.7e9

    def test_key_frames_are_recognised(self):
        self.assertEqual(tl.h265_nal_types(KEY), [32, 33, 34, 19])
        self.assertTrue(tl.is_keyframe(KEY))
        self.assertFalse(tl.is_keyframe(P))
        self.assertFalse(tl.is_keyframe(IDR))                 # no parameter sets: cannot start here
        self.assertEqual(tl.first_keyframe([P, P, KEY, P]), 2)
        self.assertIsNone(tl.first_keyframe([P, P]))

    def test_sender_clock_is_used_only_when_it_agrees(self):
        log = self.T0 + np.arange(5) * 0.1
        self.assertEqual(tl.pick_clock(log - 0.04, log)[1], "stamp")
        self.assertEqual(tl.pick_clock(log - 30.0, log)[1], "log")       # clocks apart
        self.assertEqual(tl.pick_clock(np.zeros(5), log)[1], "log")       # no stamp at all

    def test_nearest_and_hold(self):
        t = np.array([0.0, 1.0, 2.0])
        self.assertEqual(list(tl.nearest_indices(t, [-5, 0.4, 0.6, 1.5, 9])), [0, 0, 1, 1, 2])
        self.assertEqual(list(tl.hold_indices(t, [-5, 0.99, 1.0, 1.99, 9])), [0, 0, 1, 1, 2])  # never the future

    def test_a_lost_picture_spoils_the_rest_of_its_chain(self):
        t = [0, 1, 2, 4, 5, 6, 7]                               # picture 3 was lost
        key = [1, 0, 0, 0, 0, 1, 0]
        self.assertEqual(list(tl.damaged_pictures(np.array(t) / 30.0, key, 30)), [0, 0, 0, 1, 1, 0, 0])
        idx, held, longest = tl.hold_last_good([0, 1, 2, 3, 4, 5, 6], [0, 0, 0, 1, 1, 0, 0])
        self.assertEqual((list(idx), held, longest), ([0, 1, 2, 2, 2, 5, 6], 2, 2))

    def align(self, videos, js, **kw):
        return tl.align_episode(videos, js, 30, ["arm"], **kw)

    def test_grid_starts_at_the_latest_first_key_frame(self):
        t = self.T0 + np.arange(120) / 30.0
        late = video(list(t))
        late["data"] = [P] * 45 + late["data"][45:]              # first key frame is picture 60
        al = self.align({"/hal/a_camera/stream": video(list(t)), "/hal/b_camera/stream": late},
                        {"/m/arm_joint_state": joints(self.T0 + np.arange(400) / 100.0)})
        self.assertAlmostEqual(al["ticks"][0], t[60], places=6)
        self.assertEqual(len(al["ticks"]), 60)
        self.assertEqual(list(al["cams"]["a"]["index"][:3]), [60, 61, 62])
        self.assertEqual(list(al["cams"]["b"]["index"][:3]), [0, 1, 2])
        self.assertTrue(any("not in the dataset" in n for n in al["notes"]))

    def test_state_never_comes_from_the_future_and_action_falls_back_to_next_state(self):
        t = self.T0 + np.arange(60) / 30.0
        js = joints(self.T0 - 0.005 + np.arange(300) / 100.0)
        al = self.align({"/hal/a_camera/stream": video(list(t))}, {"/m/arm_joint_state": js})
        rel = al["ticks"] - self.T0                             # joint values in this test are their own time
        self.assertTrue(np.all(al["state"][:, 0] <= rel + 1e-4))
        self.assertTrue(np.all(rel - al["state"][:, 0] < 0.0101))
        self.assertEqual(al["action_source"], {"arm": "next_state"})
        np.testing.assert_array_equal(al["action"][:-1], al["state"][1:])
        self.assertEqual(al["names"], ["arm.j0", "arm.j1"])

    def test_commands_become_the_action(self):
        t = self.T0 + np.arange(60) / 30.0
        al = self.align({"/hal/a_camera/stream": video(list(t))},
                        {"/m/arm_joint_state": joints(self.T0 + np.arange(300) / 100.0, value=1.0),
                         "/m/arm_joint_command": joints(self.T0 + 0.5 + np.arange(200) / 100.0, value=7.0)})
        self.assertEqual(al["action_source"], {"arm": "command"})
        self.assertEqual(al["action"][0, 0], 1.0)                 # before the first command: the state
        self.assertEqual(al["action"][-1, 0], 7.0)

    def test_short_loss_is_bridged_long_loss_skips_the_episode(self):
        t = list(self.T0 + np.arange(90) / 30.0)
        js = {"/m/arm_joint_state": joints(self.T0 + np.arange(400) / 100.0)}
        al = self.align({"/hal/a_camera/stream": video(t, drop={50})}, js)         # pictures 51..59 are spoiled
        self.assertEqual(al["cams"]["a"]["rows_held"], 9)
        with self.assertRaisesRegex(tl.Skip, "cannot be"):
            self.align({"/hal/a_camera/stream": video(t, key_every=1000, drop={5})}, js)

    def test_camera_choice(self):
        t = list(self.T0 + np.arange(60) / 30.0)
        v = {"/hal/a_camera/stream": video(t), "/hal/b_camera/stream": video(t, key_every=1000, drop={5})}
        js = {"/m/arm_joint_state": joints(self.T0 + np.arange(300) / 100.0)}
        self.assertEqual(list(self.align(v, js, cameras=["a"])["cams"]), ["a"])
        with self.assertRaisesRegex(tl.Skip, "not in the recording"):
            self.align(v, js, cameras=["a", "zzz"])

    def test_names(self):
        self.assertEqual(tl.camera_key("/hal/chest_front_d457_camera/stream"), "chest_front_d457")
        self.assertEqual(tl.group_of("/motion/control/arm_joint_state", "_joint_state"), "arm")
        self.assertIsNone(tl.group_of("/motion/control/arm_joint_command", "_joint_state"))


if __name__ == "__main__":
    unittest.main()
