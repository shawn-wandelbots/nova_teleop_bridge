"""Loopback end-to-end: fake SDK -> xrt_sender -> UDP (private port) -> teleop_bridge --dry-run --record.

Runs both programs as real subprocesses on 127.0.0.1. Nothing is published; ROS is never imported.
"""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from pc import fake_xrt
from robot import a3_proto as pb
from robot.recording import KIND_VR_BUTTON, KIND_XR_FRAME, read_capture, read_record
from teleop_common.transform import load_config
from tests.util import CONFIG, ROOT, SAMPLE_CONTIGUOUS, free_udp_port, needs_sample

SENDER = os.path.join(ROOT, "pc", "xrt_sender.py")
BRIDGE = os.path.join(ROOT, "robot", "teleop_bridge.py")


def run_pipeline(tmp, count, sender_extra=(), bridge_extra=(), config=CONFIG, kill_signal=signal.SIGINT):
    port = free_udp_port()
    assert port >= 39000
    rec = os.path.join(tmp, "out.rec")
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    bridge = subprocess.Popen(
        [sys.executable, BRIDGE, "--dry-run", "--listen-ip", "127.0.0.1", "--port", str(port),
         "--sender-ip", "127.0.0.1", "--record", rec, "--config", config, "--stats-interval", "0.5",
         *bridge_extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        line = bridge.stdout.readline()           # "mode=DRY-RUN ..." is printed once the socket is bound
        assert "mode=DRY-RUN" in line, line
        sender = subprocess.run(
            [sys.executable, SENDER, "--robot-ip", "127.0.0.1", "--port", str(port), "--fake",
             "--count", str(count), "--rate", "100", "--config", config, *sender_extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=60)
        time.sleep(0.3)
        bridge.send_signal(kill_signal)
        out, _ = bridge.communicate(timeout=10)
    finally:
        if bridge.poll() is None:
            bridge.kill()
            bridge.communicate()
    return rec, sender, bridge.returncode, line + out


class TestEndToEnd(unittest.TestCase):
    def test_synthetic_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = load_config(CONFIG)
            cfg["output"]["timestamp_mode"] = "sdk_ns"         # lets the test recover the fake's tick index
            cfg_path = os.path.join(tmp, "cfg.json")
            with open(cfg_path, "w") as fh:
                json.dump(cfg, fh)
            rec, sender, rc, log = run_pipeline(tmp, 250, config=cfg_path)
            self.assertEqual(sender.returncode, 0, sender.stdout)
            self.assertIn("sent=250", sender.stdout)
            self.assertEqual(rc, 0, log)
            self.assertIn("exit (SIGINT)", log)
            self.assertIn("would-publish", log)
            self.assertIn("WATCHDOG", log)                     # sender stopped -> bridge went silent
            self.assertNotIn("PUBLISH rx", log)
            records = read_record(rec)
            xr = [r for r in records if r[0] == KIND_XR_FRAME]
            btn = [r for r in records if r[0] == KIND_VR_BUTTON]
            self.assertEqual(len(xr), len(btn))
            # 250 sent, the first 2 are held back by the 3-frame resume rule; allow scheduling slack
            self.assertGreaterEqual(len(xr), 200)
            self.assertLessEqual(len(xr), 248)
            ticks = []
            for (_, _, xraw), (_, _, braw) in zip(xr, btn):
                f = pb.decode_xr_frame(xraw)
                b = pb.decode_vr_buttons(braw)
                tick = (f.timestamp - 1_000_000_000) // fake_xrt.TICK_NS
                ticks.append(tick)
                phase = tick % 200
                self.assertEqual([j.code for j in f.body], list(range(24)))
                self.assertEqual(b.external_device, pb.EXTERNAL_DEVICE_GENIELINK)
                self.assertEqual(f.controllers, b.controllers)
                left, right = b.controllers
                self.assertEqual((left.code, right.code), (0, 1))
                self.assertAlmostEqual(left.index_trig, min(phase, 100) / 200.0, places=5)
                self.assertAlmostEqual(right.index_trig, 0.5 - min(phase, 100) / 200.0, places=5)
                self.assertAlmostEqual(left.hand_trig, 0.1, places=5)
                self.assertAlmostEqual(right.hand_trig, 0.3 if phase >= 100 else 0.2, places=5)
                self.assertAlmostEqual(right.axis_y, -0.25, places=5)
                self.assertEqual(right.key_one, 20 <= phase < 30)       # A
                self.assertEqual(left.key_two, 60 <= phase < 70)        # Y
                self.assertFalse(left.key_one or right.key_two or left.axis_click or right.axis_click)
                # geometry in the robot frame: z up, x forward, y left
                P = [j.pos for j in f.body]
                self.assertAlmostEqual(P[15][2], 1.57, places=5)                 # head height
                self.assertLess(P[10][2], 0.05)                                  # feet near z = 0
                self.assertGreater(P[10][0], P[7][0])                            # toes ahead (+x) of ankles
                self.assertGreater(P[16][1], 0.0)                                # left shoulder at +y
                self.assertLess(P[17][1], 0.0)                                   # right shoulder at -y
                for j in f.body:
                    self.assertAlmostEqual(math.sqrt(sum(v * v for v in j.quat)), 1.0, places=5)
            self.assertEqual(ticks, sorted(set(ticks)))                          # strictly increasing
            self.assertTrue(any(20 <= t % 200 < 30 for t in ticks))              # the A press was seen

    @needs_sample

    def test_replay_of_real_sample_reproduces_it(self):
        """A3 sample -> inverse transform -> fake SDK -> sender -> bridge -> must equal the sample."""
        with tempfile.TemporaryDirectory() as tmp:
            cfg = load_config(CONFIG)
            cfg["output"]["timestamp_mode"] = "sdk_ns"
            cfg_path = os.path.join(tmp, "cfg.json")
            with open(cfg_path, "w") as fh:
                json.dump(cfg, fh)
            rec, sender, rc, log = run_pipeline(tmp, 120, sender_extra=("--fake-replay", SAMPLE_CONTIGUOUS),
                                                config=cfg_path, kill_signal=signal.SIGTERM)
            self.assertEqual(sender.returncode, 0, sender.stdout)
            self.assertEqual(rc, 0, log)
            self.assertIn("exit (SIGTERM)", log)
            self.assertNotIn("'jump'", log)
            golden = [pb.decode_xr_frame(d) for _, d in read_capture(SAMPLE_CONTIGUOUS)]
            xr = [pb.decode_xr_frame(r[2]) for r in read_record(rec) if r[0] == KIND_XR_FRAME]
            self.assertGreaterEqual(len(xr), 100)
            for f in xr:
                tick = (f.timestamp - 1_000_000_000) // fake_xrt.TICK_NS
                g = golden[tick]
                for a, b in zip(g.body, f.body):
                    self.assertEqual(a.code, b.code)
                    self.assertLess(math.dist(a.pos, b.pos), 2e-6)
                    # quaternion equal up to sign (q and -q are the same rotation)
                    self.assertGreater(abs(sum(x * y for x, y in zip(a.quat, b.quat))), 1.0 - 1e-6)

    def test_publish_refuses_synthetic_by_default_in_parser(self):
        from robot import teleop_bridge
        a = teleop_bridge.build_parser().parse_args(["--publish"])
        self.assertFalse(a.allow_synthetic)


if __name__ == "__main__":
    unittest.main()
