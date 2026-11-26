"""aimdk.protocol.RobotFrameData: codec, packed quaternions, bridge output, live UDP path."""
import math
import os
import random
import signal
import socket
import subprocess
import sys
import time
import unittest

from robot import a3_proto as pb
from robot.recording import read_capture
from tests.test_bridge_core import Harness
from tests.util import DATA, ROOT, free_udp_port

GENIELINK_SAMPLE = os.path.join(DATA, "genielink_sample.cap")   # vendor traffic, not in git
needs_capture = unittest.skipUnless(os.path.exists(GENIELINK_SAMPLE), "genielink_sample.cap missing")


def robot_frame_harness(**cfg_out):
    def edit(cfg):
        cfg["output"].update(cfg_out)
    h = Harness(cfg_edit=edit)
    h.core.robot_frame = True
    return h


class TestQuaternionPacking(unittest.TestCase):
    def test_known_value(self):
        # identity: w is the largest (index 3), the three others are zero
        self.assertEqual(pb.compress_quat((0, 0, 0, 1)), 0xC0000000)
        self.assertEqual(pb.decompress_quat(0xC0000000), (0.0, 0.0, 0.0, 1.0))

    def test_round_trip_error_is_small(self):
        rnd = random.Random(1)
        worst = 0.0
        for _ in range(2000):
            q = [rnd.gauss(0, 1) for _ in range(4)]
            n = math.sqrt(sum(v * v for v in q))
            q = [v / n for v in q]
            r = pb.decompress_quat(pb.compress_quat(q))
            dot = abs(sum(a * b for a, b in zip(q, r)))
            worst = max(worst, 2 * math.degrees(math.acos(min(1.0, dot))))
        self.assertLess(worst, 0.35)       # degrees

    def test_sign_is_normalised(self):
        q = (0.1, -0.2, 0.3, -0.9)
        self.assertEqual(pb.compress_quat(q), pb.compress_quat([-v for v in q]))

    def test_zero_quaternion_is_refused(self):
        with self.assertRaises(pb.ProtoError):
            pb.compress_quat((0, 0, 0, 0))


class TestCodec(unittest.TestCase):
    def frame(self):
        return pb.RobotFrame(
            timestamp=1795692185822, frame_index=7,
            body=[pb.Joint(code=i, pos=(0.1 * i, -0.5, 0.25), quat=(0.0, 0.0, 0.0, 1.0)) for i in range(24)],
            devices=[pb.DeviceInput(device_type=t, hand_trig=0.5 * t, key_one=t == 1, pos=(1.0, 2.0, 3.0),
                                    quat=(0.0, 0.0, 0.0, 1.0)) for t in (0, 1, 2)])

    def test_round_trip(self):
        for mode in pb.QUAT_MODES:
            g = pb.decode_robot_frame(pb.encode_robot_frame(self.frame(), mode))
            self.assertEqual(g.timestamp, 1795692185822)
            self.assertEqual(g.frame_index, 7)
            self.assertEqual([j.code for j in g.body], list(range(24)))
            self.assertEqual([d.device_type for d in g.devices], [0, 1, 2])
            self.assertTrue(g.devices[1].key_one)
            self.assertAlmostEqual(g.devices[2].hand_trig, 1.0)
            self.assertEqual(g.body[3].quat, (0.0, 0.0, 0.0, 1.0))
            self.assertAlmostEqual(g.body[3].pos[0], 0.3, places=6)

    def test_field_numbers(self):
        raw = pb.encode_robot_frame(self.frame())
        top = [f for f, _, _ in pb._fields(raw)]
        self.assertEqual(top, [1, 2] + [102] * 24 + [103] * 3)
        # zero scalars are written too, like the vendor app does
        self.assertTrue(raw.startswith(b"\x08"))
        first_joint = [v for f, _, v in pb._fields(raw) if f == 102][0]
        self.assertTrue(first_joint.startswith(b"\x08\x00"))

    @needs_capture
    def test_recorded_vendor_traffic_is_reproduced_byte_for_byte(self):
        frames = read_capture(GENIELINK_SAMPLE)
        self.assertGreater(len(frames), 100)
        for _, raw in frames:
            f = pb.decode_robot_frame(raw, keep_packed=True)
            self.assertEqual(len(f.body), 24)
            self.assertEqual([d.device_type for d in f.devices], [0, 1, 2])
            self.assertEqual(pb.encode_robot_frame(f), raw)


class TestBridgeOutput(unittest.TestCase):
    def test_frame_has_body_and_three_devices(self):
        h = robot_frame_harness()
        out = h.send()
        f = pb.decode_robot_frame(out.robot_frame)
        self.assertEqual([j.code for j in f.body], list(range(24)))
        self.assertEqual([d.device_type for d in f.devices], [pb.DEVICE_LEFT, pb.DEVICE_RIGHT, pb.DEVICE_HEAD])
        self.assertAlmostEqual(f.devices[0].hand_trig, 0.4, places=6)
        self.assertAlmostEqual(f.devices[1].hand_trig, 0.7, places=6)
        self.assertEqual(f.timestamp, 1_700_000_000_123)
        self.assertEqual(f.frame_index, 0)

    def test_timestamps_always_rise_and_index_counts(self):
        h = robot_frame_harness()          # the harness clock is frozen: every frame "now"
        frames = [pb.decode_robot_frame(h.send().robot_frame) for _ in range(5)]
        self.assertEqual([f.timestamp for f in frames], [1_700_000_000_123 + i for i in range(5)])
        self.assertEqual([f.frame_index for f in frames], list(range(5)))

    def test_origin_first_head(self):
        fixed = pb.decode_robot_frame(robot_frame_harness().send().robot_frame)
        h = robot_frame_harness(origin="first_head")
        first = pb.decode_robot_frame(h.send().robot_frame)
        head0 = fixed.devices[2].pos
        for a, b in zip(first.devices[2].pos, (0.0, 0.0, 0.0)):
            self.assertAlmostEqual(a, b, places=6)
        for jf, jo in zip(fixed.body, first.body):
            for i in range(3):
                self.assertAlmostEqual(jo.pos[i], jf.pos[i] - head0[i], places=5)

    def test_frame_without_head_is_dropped(self):
        from teleop_common import wire
        h = robot_frame_harness()

        def no_head(f):
            f.flags &= ~wire.FLAG_HEAD
        self.assertIsNone(h.send(mutate=no_head))
        self.assertEqual(h.core.last_reason, "no_head")

    def test_plain_quaternion_option(self):
        raw = robot_frame_harness(robot_frame_quaternion="plain").send().robot_frame
        joint = [v for f, _, v in pb._fields(raw) if f == 102][0]
        pose = [v for f, _, v in pb._fields(joint) if f == 2][0]
        self.assertEqual([f for f, _, _ in pb._fields(pose)], [1, 2])


class TestLivePath(unittest.TestCase):
    def test_robot_input_mode_sends_to_stand_in_listener(self):
        """Whole chain on loopback: fake headset stream -> bridge --robot-input -> stand-in for
        the robot's teleop input.  The real port 25618 is never used."""
        port, fake_robot_port = free_udp_port(), free_udp_port(39300)
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", fake_robot_port))
        rx.settimeout(0.5)
        env = dict(os.environ, PYTHONPATH=ROOT)
        bridge = subprocess.Popen(
            [sys.executable, os.path.join(ROOT, "robot", "teleop_bridge.py"), "--listen-ip", "127.0.0.1",
             "--port", str(port), "--no-discovery", "--robot-input", "--robot-input-port", str(fake_robot_port),
             "--allow-synthetic", "--stats-interval", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        try:
            bridge.stdout.readline()
            subprocess.run([sys.executable, os.path.join(ROOT, "pc", "xrt_sender.py"), "--robot-ip", "127.0.0.1",
                            "--port", str(port), "--fake", "--count", "60", "--rate", "100"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=30, check=True)
            got = []
            try:
                while True:
                    got.append(rx.recv(65535))
            except socket.timeout:
                pass
        finally:
            bridge.send_signal(signal.SIGINT)
            log = bridge.communicate(timeout=10)[0]
            rx.close()
        self.assertGreaterEqual(len(got), 40, log)
        frames = [pb.decode_robot_frame(g) for g in got]
        ts = [f.timestamp for f in frames]
        self.assertEqual(ts, sorted(set(ts)))                      # strictly rising, no duplicates
        idx = [f.frame_index for f in frames]                      # gaps allowed (superseded frames)
        self.assertEqual(idx, sorted(set(idx)))
        self.assertLess(abs(ts[0] / 1000.0 - time.time()), 60)
        self.assertTrue(all(len(f.body) == 24 and len(f.devices) == 3 for f in frames))


if __name__ == "__main__":
    unittest.main()


def _xrt(rx, ry, rz):
    """robot-frame point -> the headset frame used on the wire (default transform)."""
    return (-ry, rz, -rx)


def _body_at(cx, cy, heading_deg, head_z=1.7):
    """24 joints of an upright operator standing at (cx, cy), facing heading_deg (robot frame)."""
    h = math.radians(heading_deg)
    fwd, left = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
    side = {1: 0.1, 2: -0.1, 16: 0.2, 17: -0.2, 22: 0.25, 23: -0.25}      # metres to the left
    front = {22: 0.3, 23: 0.3}                                             # hands held forward
    out = []
    for i in range(24):
        s, f = side.get(i, 0.0), front.get(i, 0.0)
        z = head_z if i == 15 else 1.0
        out.append(_xrt(cx + left[0] * s + fwd[0] * f, cy + left[1] * s + fwd[1] * f, z) + (0.0, 0.0, 0.0, 1.0))
    return out


class TestBodyReference(unittest.TestCase):
    def harness(self):
        return robot_frame_harness(origin="body")

    def send(self, h, body, xyab=False, grip=0.0):
        def mut(f):
            f.body = body
            f.left.key_one = f.left.key_two = f.right.key_one = f.right.key_two = xyab
            f.left.grip = f.right.grip = grip
        out = h.send(mutate=mut)
        self.assertIsNotNone(out, h.core.last_reason)
        return pb.decode_robot_frame(out.robot_frame)

    def walk(self, h, x, y, heading, steps=10, **kw):
        """Move there over several frames, as a person would (no tracking jump)."""
        f = None
        for i in range(1, steps + 1):
            k = i / steps
            f = self.send(h, _body_at(x * k, y * k, heading * k), **kw)
        return f

    def check_centred(self, f):
        """Head joint at the origin, operator facing +x (left shoulder on +y, hands in front)."""
        j = {x.code: x.pos for x in f.body}
        for v in j[15]:
            self.assertAlmostEqual(v, 0.0, places=5)
        self.assertAlmostEqual(j[16][0], 0.0, places=5)
        self.assertAlmostEqual(j[16][1], 0.2, places=5)
        self.assertAlmostEqual(j[22][0], 0.3, places=5)
        self.assertAlmostEqual(j[22][1], 0.25, places=5)
        self.assertAlmostEqual(j[0][2], -0.7, places=5)

    def test_stream_start_sets_the_reference_from_the_body(self):
        f = self.send(self.harness(), _body_at(3.0, -2.0, 40.0))
        self.check_centred(f)

    def test_devices_sit_on_the_body_joints(self):
        h = self.harness()

        def far_away(fr):
            fr.body = _body_at(3.0, -2.0, 40.0)
            fr.head = (50.0, 60.0, 70.0, 0.0, 0.0, 0.0, 1.0)       # devices in some other frame
            fr.left_pose = fr.right_pose = fr.head
        f = pb.decode_robot_frame(h.send(mutate=far_away).robot_frame)
        j = {x.code: x.pos for x in f.body}
        self.assertEqual([d.pos for d in f.devices], [j[22], j[23], j[15]])

    def test_moving_does_not_move_the_reference(self):
        h = self.harness()
        self.send(h, _body_at(0.0, 0.0, 0.0))
        f = self.send(h, _body_at(0.1, 0.0, 0.0))
        self.assertAlmostEqual({x.code: x.pos for x in f.body}[15][0], 0.1, places=5)

    def test_xyab_sets_a_new_reference(self):
        h = self.harness()
        self.send(h, _body_at(0.0, 0.0, 0.0))
        moved = _body_at(0.15, 0.1, 25.0)
        f = self.walk(h, 0.15, 0.1, 25.0)
        self.assertNotAlmostEqual({x.code: x.pos for x in f.body}[16][0], 0.0, places=2)   # turned, old reference
        self.check_centred(self.send(h, moved, xyab=True))
        self.check_centred(self.send(h, moved, xyab=True))            # holding the buttons changes nothing more
        self.check_centred(self.send(h, moved))
        self.assertEqual(h.core.stats["reanchors"], 2)

    def test_xyab_is_ignored_while_the_grips_are_held(self):
        h = self.harness()
        self.send(h, _body_at(0.0, 0.0, 0.0))
        self.walk(h, 0.15, 0.1, 25.0, grip=1.0)
        f = self.send(h, _body_at(0.15, 0.1, 25.0), xyab=True, grip=1.0)
        self.assertNotAlmostEqual({x.code: x.pos for x in f.body}[15][0], 0.0, places=2)
        self.assertEqual(h.core.stats["reanchors"], 1)

    def test_a_big_step_with_xyab_is_not_treated_as_a_tracking_jump(self):
        h = self.harness()
        self.send(h, _body_at(0.0, 0.0, 0.0))
        self.walk(h, 0.2, 0.2, 170.0, steps=40)
        self.check_centred(self.send(h, _body_at(0.2, 0.2, 170.0), xyab=True))
