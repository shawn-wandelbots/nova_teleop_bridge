import os
import struct
import unittest

from robot import a3_proto as pb
from robot.recording import read_capture
from tests.util import SAMPLE_CONTIGUOUS, SAMPLE_SPARSE, needs_sample


class TestEncoding(unittest.TestCase):
    def test_known_bytes_joint(self):
        # Hand-assembled reference: Joint{code=1, pos=(1,2,3), quat=(0,0,0,1)} as body_data (field 4)
        joint = (b"\x08\x01" + b"\x12\x0f" + b"\x0d" + struct.pack("<f", 1) + b"\x15" + struct.pack("<f", 2)
                 + b"\x1d" + struct.pack("<f", 3)
                 + b"\x1a\x14" + b"\x0d" + struct.pack("<f", 0) + b"\x15" + struct.pack("<f", 0)
                 + b"\x1d" + struct.pack("<f", 0) + b"\x25" + struct.pack("<f", 1))
        expect = b"\x08\x05" + b"\x22" + bytes([len(joint)]) + joint
        f = pb.XRFrame(timestamp=5, body=[pb.Joint(1, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))])
        self.assertEqual(pb.encode_xr_frame(f), expect)

    def test_known_bytes_controller(self):
        c = pb.Controller(code=1, axis_click=True, axis_x=0.5, axis_y=-0.5, index_trig=1.0, hand_trig=0.25,
                          key_one=True, key_two=False)
        body = (b"\x08\x01\x10\x01" + b"\x1d" + struct.pack("<f", 0.5) + b"\x25" + struct.pack("<f", -0.5)
                + b"\x2d" + struct.pack("<f", 1.0) + b"\x35" + struct.pack("<f", 0.25) + b"\x38\x01\x40\x00")
        self.assertEqual(pb.encode_vr_buttons(pb.VRButtons(7, [c], pb.EXTERNAL_DEVICE_GENIELINK)),
                         b"\x08\x07" + b"\x12" + bytes([len(body)]) + body + b"\x18\x02")
        self.assertEqual(pb.encode_xr_frame(pb.XRFrame(7, controllers=[c])),
                         b"\x08\x07" + b"\x2a" + bytes([len(body)]) + body)

    def test_roundtrip_full_frame(self):
        f = pb.XRFrame(timestamp=(1 << 40) + 3)
        for i in range(24):
            f.body.append(pb.Joint(i, (i * 0.5, -i * 0.25, 1.0 / (i + 1)), (0.5, -0.5, 0.5, 0.5)))
        f.left_hand.append(pb.Joint(0, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)))
        f.right_hand.append(pb.Joint(0, (-1.0, 2.0, 3.0), (0.0, 0.0, 1.0, 0.0)))
        f.controllers = [pb.Controller(0, False, -1.0, 1.0, 0.0, 0.5, False, True),
                         pb.Controller(1, True, 0.25, -0.75, 1.0, 0.0, True, False)]
        for explicit in (True, False):
            g = pb.decode_xr_frame(pb.encode_xr_frame(f, explicit))
            self.assertEqual(g.timestamp, f.timestamp)
            self.assertEqual(g.controllers, f.controllers)
            self.assertEqual([j.code for j in g.body], list(range(24)))
            for a, b in zip(f.body + f.left_hand + f.right_hand, g.body + g.left_hand + g.right_hand):
                self.assertEqual(struct.pack("<3f", *a.pos), struct.pack("<3f", *b.pos))
                self.assertEqual(struct.pack("<4f", *a.quat), struct.pack("<4f", *b.quat))

    def test_buttons_roundtrip(self):
        b = pb.VRButtons(123456789012, [pb.Controller(0, hand_trig=0.5), pb.Controller(1, key_two=True)],
                         pb.EXTERNAL_DEVICE_YUANJI_APP)
        for explicit in (True, False):
            self.assertEqual(pb.decode_vr_buttons(pb.encode_vr_buttons(b, explicit)), b)

    def test_proto3_canonical_omits_defaults(self):
        c = pb.Controller()
        self.assertEqual(pb.encode_vr_buttons(pb.VRButtons(0, [c], 0), explicit=False), b"\x12\x00")
        self.assertLess(len(pb.encode_xr_frame(pb.XRFrame(0, body=[pb.Joint()]), explicit=False)),
                        len(pb.encode_xr_frame(pb.XRFrame(0, body=[pb.Joint()]), explicit=True)))

    def test_negative_int32_code(self):
        f = pb.XRFrame(body=[pb.Joint(-1)])
        self.assertEqual(pb.decode_xr_frame(pb.encode_xr_frame(f)).body[0].code, -1)

    def test_unknown_fields_are_skipped(self):
        extra = b"\x78\x05" + b"\x82\x01\x02hi"       # field 15 varint, field 16 bytes
        g = pb.decode_xr_frame(pb.encode_xr_frame(pb.XRFrame(9, body=[pb.Joint(3)])) + extra)
        self.assertEqual((g.timestamp, g.body[0].code), (9, 3))

    def test_truncated_input_raises(self):
        data = pb.encode_xr_frame(pb.XRFrame(9, body=[pb.Joint(3)]))
        for cut in (1, 5, len(data) - 1):
            with self.assertRaises(pb.ProtoError):
                pb.decode_xr_frame(data[:cut])
        with self.assertRaises(pb.ProtoError):
            pb.decode_xr_frame(b"\xff" * 12)

    def test_topic_names(self):
        self.assertEqual(pb.ros_topic_name("/remote/vr_data", pb.XR_FRAME_TYPE),
                         "/remote/vr_data/pb_3Aaimdk_2Eprotocol_2EXRFrameData")
        self.assertEqual(pb.ros_topic_name("/remote/vr_button", pb.VR_BUTTON_TYPE),
                         "/remote/vr_button/pb_3Aaimdk_2Eprotocol_2EVRButtonChannel")


class TestGoldenSample(unittest.TestCase):
    """Frames captured from the robot's own PICO recording via pico_mock_player on loopback."""

    def _check(self, path):
        recs = read_capture(path)
        self.assertGreater(len(recs), 0)
        for _, raw in recs:
            f = pb.decode_xr_frame(raw)
            self.assertEqual(len(raw), 1038)
            self.assertEqual(pb.encode_xr_frame(f, explicit=True), raw)     # byte-identical
            self.assertEqual(f.timestamp, 4294967295)
            self.assertEqual([j.code for j in f.body], list(range(24)))
            self.assertEqual((len(f.left_hand), len(f.right_hand), len(f.controllers)), (0, 0, 0))
        return len(recs)

    @needs_sample

    def test_sparse_fixture(self):
        self.assertEqual(self._check(SAMPLE_SPARSE), 31)

    @needs_sample

    def test_contiguous_fixture(self):
        self.assertEqual(self._check(SAMPLE_CONTIGUOUS), 120)

    @needs_sample

    def test_frame0_values(self):
        f = pb.decode_xr_frame(read_capture(SAMPLE_SPARSE)[0][1])
        head, lfoot = f.body[15], f.body[10]
        self.assertAlmostEqual(head.pos[2], -0.039, places=3)
        self.assertAlmostEqual(lfoot.pos[2], -1.500, places=3)
        self.assertLess(f.body[22].pos[0], -0.7)     # left hand at -x
        self.assertGreater(f.body[23].pos[0], 0.6)   # right hand at +x

    @unittest.skipUnless(os.environ.get("NTB_FULL_SAMPLE"), "set NTB_FULL_SAMPLE=<full .cap> to run")
    def test_full_sample(self):
        self.assertEqual(self._check(os.environ["NTB_FULL_SAMPLE"]), 2921)


if __name__ == "__main__":
    unittest.main()
