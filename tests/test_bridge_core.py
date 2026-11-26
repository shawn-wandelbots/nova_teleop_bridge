import copy
import math
import unittest

from pc import fake_xrt
from robot import a3_proto as pb
from robot.bridge_core import BridgeCore, Limits
from robot.recording import read_capture
from teleop_common import wire
from teleop_common.transform import FrameTransform, load_config
from tests.util import CONFIG, SAMPLE_CONTIGUOUS, needs_sample

IP = "10.0.0.5"
DT = 0.01


def base_frame(seq=1, session=1, t=0.0):
    f = wire.Frame(session=session, seq=seq, t_send_ns=int(1e18 + t * 1e9), t_sdk_ns=seq,
                   flags=wire.FLAG_HEAD | wire.FLAG_LEFT | wire.FLAG_RIGHT)
    f.body = [(0.1 * i, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0) for i in range(24)]
    f.left = wire.ControllerState(0.1, 0.2, 0.3, 0.4, False, False, False)
    f.right = wire.ControllerState(-0.1, -0.2, 0.6, 0.7, False, False, False)
    return f


class Harness:
    """Drives a BridgeCore with a simulated clock."""

    def __init__(self, limits=None, sender_ip=None, cfg_edit=None):
        cfg = load_config(CONFIG)
        cfg["output"]["origin"] = "fixed"
        if cfg_edit:
            cfg_edit(cfg)
        self.logs = []
        self.core = BridgeCore(cfg, limits or Limits(resume_frames=1), sender_ip=sender_ip,
                               wall_clock_ns=lambda: 1_700_000_000_123_456_789, log=self.logs.append)
        self.now = 100.0
        self.seq = 0

    def send(self, frame=None, ip=IP, dt=DT, mutate=None, raw=None):
        self.now += dt
        if raw is None:
            if frame is None:
                self.seq += 1
                frame = base_frame(self.seq, t=self.now)
            if mutate:
                mutate(frame)
            raw = wire.pack(frame)
        return self.core.handle(raw, ip, self.now)

    def warm(self, n=3):
        out = None
        for _ in range(n):
            out = self.send()
        return out


class TestAcceptAndConvert(unittest.TestCase):
    def test_valid_frame_produces_both_messages(self):
        h = Harness()
        out = h.send()
        self.assertIsNotNone(out)
        xr = pb.decode_xr_frame(out.xr_frame)
        btn = pb.decode_vr_buttons(out.vr_button)
        self.assertEqual([j.code for j in xr.body], list(range(24)))
        self.assertEqual(xr.timestamp, 1_700_000_000_123)                 # unix_ms default
        self.assertEqual(btn.timestamp, xr.timestamp)
        self.assertEqual(btn.external_device, pb.EXTERNAL_DEVICE_GENIELINK)
        self.assertEqual([c.code for c in btn.controllers], [0, 1])
        self.assertEqual(xr.controllers, btn.controllers)
        self.assertAlmostEqual(btn.controllers[0].index_trig, 0.3, places=6)
        self.assertAlmostEqual(btn.controllers[0].hand_trig, 0.4, places=6)
        self.assertAlmostEqual(btn.controllers[1].hand_trig, 0.7, places=6)
        self.assertAlmostEqual(btn.controllers[1].axis_y, -0.2, places=6)
        # default transform: xrt (x, y, z) -> robot (-z, -x, y)
        for a, b in zip(xr.body[5].pos, (0.0, -0.5, 1.0)):
            self.assertAlmostEqual(a, b, places=6)
        self.assertEqual((xr.left_hand, xr.right_hand), ([], []))

    def test_button_mapping_and_no_synthesis(self):
        h = Harness()

        def press(f):
            f.left.key_one = True       # X
            f.right.key_two = True      # B
        btn = pb.decode_vr_buttons(h.send(mutate=press).vr_button)
        left, right = btn.controllers
        self.assertEqual((left.key_one, left.key_two, left.axis_click), (True, False, False))
        self.assertEqual((right.key_one, right.key_two, right.axis_click), (False, True, False))
        # the next frame without presses carries no presses
        btn = pb.decode_vr_buttons(h.send().vr_button)
        self.assertFalse(any(c.key_one or c.key_two or c.axis_click for c in btn.controllers))

    def test_estop_chord_is_passed_through_untouched(self):
        h = Harness()

        def both(f):
            f.left.axis_click = f.right.axis_click = True
        btn = pb.decode_vr_buttons(h.send(mutate=both).vr_button)
        self.assertTrue(all(c.axis_click for c in btn.controllers))

    def test_clamping(self):
        h = Harness()

        def wild(f):
            f.left = wire.ControllerState(3.0, -3.0, 1.7, -0.2)
            f.right = wire.ControllerState(-1.5, 0.5, -1.0, 9.0)
        btn = pb.decode_vr_buttons(h.send(mutate=wild).vr_button)
        l, r = btn.controllers
        self.assertEqual((l.axis_x, l.axis_y, l.index_trig, l.hand_trig), (1.0, -1.0, 1.0, 0.0))
        self.assertEqual((r.axis_x, r.axis_y, r.index_trig, r.hand_trig), (-1.0, 0.5, 0.0, 1.0))
        self.assertEqual(h.core.stats["clamped"], 2)

    def test_untracked_controller_is_omitted_not_zeroed(self):
        h = Harness()

        def lose_left(f):
            f.flags &= ~wire.FLAG_LEFT
        btn = pb.decode_vr_buttons(h.send(mutate=lose_left).vr_button)
        self.assertEqual([c.code for c in btn.controllers], [1])

    def test_controller_codes_configurable(self):
        def swap(cfg):
            cfg["controllers"]["left_code"], cfg["controllers"]["right_code"] = 1, 0
        h = Harness(cfg_edit=swap)
        btn = pb.decode_vr_buttons(h.send().vr_button)
        self.assertEqual([c.code for c in btn.controllers], [0, 1])
        self.assertAlmostEqual(btn.controllers[0].hand_trig, 0.7, places=6)   # right controller now code 0

    def test_timestamp_modes_and_external_device(self):
        for mode, expect in (("const_u32max", 4294967295), ("unix_ns", 1_700_000_000_123_456_789),
                             ("unix_us", 1_700_000_000_123_456), ("sdk_ns", 1)):
            def edit(cfg, mode=mode):
                cfg["output"]["timestamp_mode"] = mode
                cfg["output"]["external_device"] = "UNKNOWN"
            out = Harness(cfg_edit=edit).send()
            self.assertEqual(pb.decode_xr_frame(out.xr_frame).timestamp, expect)
            self.assertEqual(pb.decode_vr_buttons(out.vr_button).external_device, 0)

    def test_pose_joints_opt_in(self):
        def edit(cfg):
            cfg["output"]["controller_pose_as_hand_joint"] = True
        xr = pb.decode_xr_frame(Harness(cfg_edit=edit).send().xr_frame)
        self.assertEqual((len(xr.left_hand), len(xr.right_hand)), (1, 1))

    def test_bad_config_rejected(self):
        for edit in (lambda c: c["output"].update(timestamp_mode="nope"),
                     lambda c: c["output"].update(external_device="PICO"),
                     lambda c: c["controllers"].update(left_code=1)):
            with self.assertRaises(ValueError):
                Harness(cfg_edit=edit)


class TestRejection(unittest.TestCase):
    def assertRejected(self, h, reason, **kw):
        self.assertIsNone(h.send(**kw))
        self.assertEqual(h.core.last_reason, reason)

    def test_garbage_and_wrong_magic(self):
        h = Harness()
        self.assertRejected(h, "short", raw=b"\x00" * 10)
        self.assertRejected(h, "bad_magic", raw=b"\x08\xff\xff\xff\xff\x0f" + b"\x00" * 1032)   # raw PICO frame
        raw = bytearray(wire.pack(base_frame(1)))
        raw[4] = 9
        self.assertRejected(h, "bad_version", raw=bytes(raw))
        raw = bytearray(wire.pack(base_frame(1)))
        raw[200] ^= 0xFF
        self.assertRejected(h, "bad_crc", raw=bytes(raw))
        self.assertIsNone(h.core.sender_ip)          # invalid datagrams never lock the sender

    def test_first_seen_sender_lock(self):
        h = Harness()
        self.assertIsNotNone(h.send(ip="10.0.0.5"))
        self.assertEqual(h.core.sender_ip, "10.0.0.5")
        self.assertRejected(h, "bad_source", ip="10.0.0.99")
        self.assertIsNotNone(h.send(ip="10.0.0.5"))
        self.assertEqual(h.core.stats["reject_bad_source"], 1)

    def test_configured_sender(self):
        h = Harness(sender_ip="10.0.0.7")
        self.assertRejected(h, "bad_source", ip="10.0.0.5")
        self.assertIsNotNone(h.send(ip="10.0.0.7"))

    def test_duplicate_and_out_of_order(self):
        h = Harness()
        for seq, ok in ((10, True), (10, False), (9, False), (11, True), (13, True), (12, False), (14, True)):
            out = h.send(frame=base_frame(seq, t=h.now))
            self.assertEqual(out is not None, ok, "seq %d" % seq)
            if not ok:
                self.assertEqual(h.core.last_reason, "stale_seq")
        self.assertEqual(h.core.stats["reject_stale_seq"], 3)
        self.assertEqual(h.core.stats["seq_gaps"], 1)

    def test_sequence_wraparound(self):
        h = Harness()
        self.assertIsNotNone(h.send(frame=base_frame(0xFFFFFFFE)))
        self.assertIsNotNone(h.send(frame=base_frame(0xFFFFFFFF)))
        self.assertIsNotNone(h.send(frame=base_frame(0)))
        self.assertIsNone(h.send(frame=base_frame(0xFFFFFFFF)))

    def test_new_session_only_after_silence(self):
        h = Harness()
        self.assertIsNotNone(h.send(frame=base_frame(500, session=1)))
        self.assertRejected(h, "foreign_session", frame=base_frame(1, session=2))
        self.assertIsNotNone(h.send(frame=base_frame(501, session=1)))
        # sender restarts: after > resync_ms of silence the new session (seq restarts at 1) is accepted
        self.assertIsNotNone(h.send(frame=base_frame(1, session=2), dt=0.7))
        self.assertRejected(h, "foreign_session", frame=base_frame(502, session=1))

    def test_nan_inf(self):
        for bad in (math.nan, math.inf, -math.inf):
            h = Harness()

            def m(f, bad=bad):
                f.body[7] = (0.0, bad, 0.0, 0.0, 0.0, 0.0, 1.0)
            self.assertRejected(h, "nonfinite", mutate=m)

            def mq(f, bad=bad):
                f.body[7] = (0.0, 0.0, 0.0, 0.0, bad, 0.0, 1.0)
            self.assertRejected(h, "nonfinite", mutate=mq)

            def mc(f, bad=bad):
                f.right.grip = bad
            self.assertRejected(h, "nonfinite", mutate=mc)
            self.assertEqual(h.core.stats["valid"], 0)

    def test_bad_quaternion(self):
        h = Harness()

        def m(f):
            f.body[0] = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertRejected(h, "bad_quat", mutate=m)

    def test_out_of_range(self):
        h = Harness()

        def m(f):
            f.body[0] = (0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.assertRejected(h, "out_of_range", mutate=m)

    def test_no_body(self):
        h = Harness()

        def m(f):
            f.body = None
        self.assertRejected(h, "no_body", mutate=m)
        h2 = Harness(limits=Limits(resume_frames=1, require_body=False))
        out = h2.send(mutate=m)
        self.assertEqual(pb.decode_xr_frame(out.xr_frame).body, [])

    def test_synthetic_gate(self):
        def m(f):
            f.flags |= wire.FLAG_SYNTHETIC
        self.assertIsNotNone(Harness().send(mutate=m))
        h = Harness(limits=Limits(resume_frames=1, allow_synthetic=False))
        self.assertRejected(h, "synthetic", mutate=m)

    def test_jump_rejected_and_baseline_kept(self):
        h = Harness()
        h.warm()

        def jump(f):
            f.body[22] = (f.body[22][0] + 0.6,) + f.body[22][1:]
        self.assertRejected(h, "jump", mutate=jump)
        self.assertRejected(h, "jump", mutate=jump)          # still compared with the last GOOD frame
        self.assertIsNotNone(h.send())                        # back to the old pose: fine

    def test_small_and_duplicate_motion_accepted(self):
        h = Harness()
        h.warm()

        def step(f):
            f.body[22] = (f.body[22][0] + 0.14,) + f.body[22][1:]      # 14 cm/frame occurs in the real sample
        self.assertIsNotNone(h.send(mutate=step))
        self.assertIsNotNone(h.send(mutate=step))             # identical consecutive frames are normal

    def test_jump_accepted_after_resync_silence(self):
        h = Harness()
        h.warm()

        def jump(f):
            f.body[22] = (f.body[22][0] + 0.6,) + f.body[22][1:]
        self.assertIsNotNone(h.send(mutate=jump, dt=0.7))     # > resync_ms: robot has dropped the session

    def test_persistent_jump_resyncs_only_after_resync_ms(self):
        h = Harness()
        h.warm()

        def jump(f):
            f.body[22] = (f.body[22][0] + 0.6,) + f.body[22][1:]
        results = [h.send(mutate=jump) is not None for _ in range(70)]
        first_ok = results.index(True)
        self.assertGreaterEqual(first_ok * DT * 1000.0, 590.0)       # silent for ~resync_ms first
        self.assertTrue(all(results[first_ok:]))

    @needs_sample

    def test_real_sample_passes_all_limits(self):
        """The fastest 1.2 s of the robot's PICO sample must not trip the plausibility limits."""
        cfg = load_config(CONFIG)
        tf = FrameTransform.from_config(cfg)
        h = Harness()
        n = 0
        for _, raw in read_capture(SAMPLE_CONTIGUOUS):
            src = pb.decode_xr_frame(raw)
            f = base_frame(n + 1, t=h.now + DT)
            f.body = [tuple(tf.inverse_position(j.pos)) + tuple(tf.inverse_orientation(j.quat)) for j in src.body]
            out = h.send(frame=f)
            self.assertIsNotNone(out, "frame %d rejected: %s" % (n, h.core.last_reason))
            got = pb.decode_xr_frame(out.xr_frame)
            for a, b in zip(src.body, got.body):
                self.assertLess(math.dist(a.pos, b.pos), 2e-6)
                self.assertGreater(abs(sum(x * y for x, y in zip(a.quat, b.quat))), 1.0 - 1e-6)
            n += 1
        self.assertEqual(n, 120)


class TestWatchdog(unittest.TestCase):
    def test_trips_after_timeout_and_stays_silent(self):
        h = Harness(limits=Limits(resume_frames=3))
        self.assertIsNone(h.send())
        self.assertEqual(h.core.last_reason, "resuming")
        self.assertIsNone(h.send())
        self.assertIsNotNone(h.send())            # third consecutive valid frame enables output
        self.assertTrue(h.core.active)
        self.assertFalse(h.core.tick(h.now + 0.099))
        self.assertTrue(h.core.active)
        self.assertTrue(h.core.tick(h.now + 0.101))       # 100 ms default
        self.assertFalse(h.core.active)
        self.assertFalse(h.core.tick(h.now + 5.0))        # trips once, then nothing
        self.assertEqual(h.core.stats["watchdog_trips"], 1)
        self.assertTrue(any("WATCHDOG" in m for m in h.logs))

    def test_resume_needs_consecutive_valid_frames(self):
        h = Harness(limits=Limits(resume_frames=3))
        h.warm(3)
        self.assertIsNone(h.send(dt=0.3))                 # watchdog tripped inside handle()
        self.assertFalse(h.core.active)
        self.assertIsNone(h.send())
        self.assertIsNone(h.send(raw=b"junk"))            # an invalid datagram restarts the count
        self.assertIsNone(h.send())
        self.assertIsNone(h.send())
        self.assertIsNotNone(h.send())
        self.assertTrue(h.core.active)

    def test_invalid_frames_do_not_feed_the_watchdog(self):
        h = Harness()
        h.warm()

        def nan(f):
            f.body[0] = (math.nan,) * 3 + (0.0, 0.0, 0.0, 1.0)
        for _ in range(12):
            self.assertIsNone(h.send(mutate=nan))
        self.assertFalse(h.core.active)
        self.assertEqual(h.core.stats["watchdog_trips"], 1)

    def test_custom_timeout(self):
        h = Harness(limits=Limits(resume_frames=1, watchdog_ms=30.0))
        h.warm()
        self.assertTrue(h.core.tick(h.now + 0.031))


class TestFakeSdkIsHarmless(unittest.TestCase):
    def test_never_deadman_never_estop(self):
        fake_xrt.configure()
        fake_xrt.init()
        for _ in range(1000):
            fake_xrt.get_time_stamp_ns()
            self.assertLessEqual(max(fake_xrt.get_left_grip(), fake_xrt.get_right_grip()), 0.3)
            self.assertFalse(fake_xrt.get_left_axis_click() or fake_xrt.get_right_axis_click())
            self.assertLessEqual(max(fake_xrt.get_left_trigger(), fake_xrt.get_right_trigger()), 0.5)
        fake_xrt.close()


if __name__ == "__main__":
    unittest.main()
