import contextlib
import io
import math
import sys
import types
import unittest

from pc import fake_xrt, xrt_sender
from pc.xrt_adapter import SdkError, XrtAdapter
from robot import teleop_bridge
from teleop_common import wire
from teleop_common.transform import load_config
from tests.util import CONFIG, FORBIDDEN_PORTS


def stub_sdk(**overrides):
    """A minimal in-memory SDK with fixed values."""
    m = types.SimpleNamespace()
    pose = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    vals = dict(init=lambda: None, close=lambda: None, get_time_stamp_ns=lambda: 123,
                get_headset_pose=lambda: list(pose), get_left_controller_pose=lambda: list(pose),
                get_right_controller_pose=lambda: list(pose),
                get_left_trigger=lambda: 0.25, get_right_trigger=lambda: 0.5,
                get_left_grip=lambda: 0.75, get_right_grip=lambda: 1.0,
                get_left_axis=lambda: [0.5, -0.5], get_right_axis=lambda: [-0.25, 0.25],
                get_left_axis_click=lambda: True, get_right_axis_click=lambda: False,
                get_A_button=lambda: True, get_B_button=lambda: False,
                get_X_button=lambda: False, get_Y_button=lambda: True,
                is_body_data_available=lambda: True,
                get_body_joints_pose=lambda: [list(pose) for _ in range(24)])
    vals.update(overrides)
    for k, v in vals.items():
        setattr(m, k, v)
    return m


class TestAdapter(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(CONFIG)

    def test_field_mapping(self):
        f = XrtAdapter(stub_sdk(), self.cfg).snapshot()
        self.assertEqual(f.t_sdk_ns, 123)
        self.assertEqual(f.flags, wire.FLAG_HEAD | wire.FLAG_LEFT | wire.FLAG_RIGHT)
        self.assertEqual((f.left.trigger, f.left.grip, f.left.axis_x, f.left.axis_y), (0.25, 0.75, 0.5, -0.5))
        self.assertEqual((f.right.trigger, f.right.grip, f.right.axis_x, f.right.axis_y), (0.5, 1.0, -0.25, 0.25))
        # left: key_one = X, key_two = Y ; right: key_one = A, key_two = B
        self.assertEqual((f.left.key_one, f.left.key_two, f.left.axis_click), (False, True, True))
        self.assertEqual((f.right.key_one, f.right.key_two, f.right.axis_click), (True, False, False))
        self.assertEqual(len(f.body), 24)
        self.assertTrue(wire.unpack(wire.pack(f)).flags & wire.FLAG_BODY)

    def test_body_unavailable(self):
        f = XrtAdapter(stub_sdk(is_body_data_available=lambda: False), self.cfg).snapshot()
        self.assertIsNone(f.body)
        f = XrtAdapter(stub_sdk(get_body_joints_pose=lambda: [[0.0] * 7] * 24), self.cfg).snapshot()
        self.assertIsNone(f.body)      # all-zero quaternions = not tracked

    def test_untracked_controller_not_flagged(self):
        f = XrtAdapter(stub_sdk(get_left_controller_pose=lambda: [0.0] * 7), self.cfg).snapshot()
        self.assertEqual(f.flags, wire.FLAG_HEAD | wire.FLAG_RIGHT)

    def test_synthetic_flag(self):
        self.assertTrue(XrtAdapter(stub_sdk(), self.cfg, synthetic=True).snapshot().flags & wire.FLAG_SYNTHETIC)

    def test_missing_function_reported(self):
        sdk = stub_sdk()
        del sdk.get_left_grip
        with self.assertRaises(SdkError):
            XrtAdapter(sdk, self.cfg)

    def test_renamed_function_via_config(self):
        sdk = stub_sdk()
        sdk.read_left_grip = lambda: 0.125
        self.cfg["sdk"]["left_grip"] = "read_left_grip"
        self.assertEqual(XrtAdapter(sdk, self.cfg).snapshot().left.grip, 0.125)

    def test_pose_order(self):
        # SDK that returns qw first: [x, y, z, qw, qx, qy, qz]
        sdk = stub_sdk(get_headset_pose=lambda: [1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0])
        self.cfg["sdk"]["pose_order"] = [0, 1, 2, 4, 5, 6, 3]
        self.assertEqual(XrtAdapter(sdk, self.cfg).snapshot().head, (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))

    def test_bad_pose_length_raises(self):
        with self.assertRaises(SdkError):
            XrtAdapter(stub_sdk(get_headset_pose=lambda: [0.0] * 6), self.cfg).snapshot()

    def test_fake_sdk_through_adapter(self):
        fake_xrt.configure()
        ad = XrtAdapter(fake_xrt, self.cfg, synthetic=True)
        ad.open()
        a, b = ad.snapshot(), ad.snapshot()
        ad.close()
        self.assertEqual(b.t_sdk_ns - a.t_sdk_ns, fake_xrt.TICK_NS)
        self.assertEqual(len(a.body), 24)
        self.assertTrue(sender_finite(a))


def sender_finite(f):
    return xrt_sender.frame_is_finite(f)


class TestSenderHelpers(unittest.TestCase):
    def test_frame_is_finite(self):
        f = XrtAdapter(stub_sdk(), load_config(CONFIG)).snapshot()
        self.assertTrue(xrt_sender.frame_is_finite(f))
        f.right.trigger = math.nan
        self.assertFalse(xrt_sender.frame_is_finite(f))

    def test_forbidden_ports_refused(self):
        # Both programs must refuse these ports during argument parsing, before any socket exists.
        for port in FORBIDDEN_PORTS:
            with contextlib.redirect_stderr(io.StringIO()) as err:
                with self.assertRaises(SystemExit):
                    xrt_sender.main(["--robot-ip", "127.0.0.1", "--port", str(port), "--fake", "--count", "1"])
                with self.assertRaises(SystemExit):
                    teleop_bridge.main(["--port", str(port), "--duration", "0.1"])
            self.assertEqual(err.getvalue().count("belongs to the robot's own teleop stack"), 2)


class TestBridgeCli(unittest.TestCase):
    def test_dry_run_is_default_and_publish_is_explicit(self):
        ap = teleop_bridge.build_parser()
        self.assertFalse(ap.parse_args([]).publish)
        self.assertFalse(ap.parse_args(["--dry-run"]).publish)
        self.assertTrue(ap.parse_args(["--publish"]).publish)
        with self.assertRaises(SystemExit):
            ap.parse_args(["--publish", "--dry-run"])

    def test_importing_bridge_does_not_import_ros(self):
        self.assertNotIn("rclpy", sys.modules)
        self.assertNotIn("ros2_plugin_proto", sys.modules)

    def test_topic_names(self):
        self.assertEqual(teleop_bridge.VR_DATA_TOPIC, "/remote/vr_data/pb_3Aaimdk_2Eprotocol_2EXRFrameData")
        self.assertEqual(teleop_bridge.VR_BUTTON_TOPIC,
                         "/remote/vr_button/pb_3Aaimdk_2Eprotocol_2EVRButtonChannel")


if __name__ == "__main__":
    unittest.main()
