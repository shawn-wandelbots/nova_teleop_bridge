"""NTD1 discovery responder: codec, drop rules, rate limit, loopback end-to-end.

Loopback only, private ports >= 39600. Never binds the default data / discovery ports.
"""
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

from robot import discovery as d
from tests.util import CONFIG, ROOT, free_udp_port

BRIDGE = os.path.join(ROOT, "robot", "teleop_bridge.py")
NONCE = bytes(range(1, 9))


def test_port():
    port = free_udp_port(start=39600)
    assert port >= 39600
    return port


def make_responder(mode=d.MODE_DRY_RUN, locked=False, **kw):
    state = {"mode": mode, "locked": locked, "calls": 0}

    def status():
        state["calls"] += 1
        return state["mode"], state["locked"]

    return d.DiscoveryResponder(39612, status, model="A3_TEST", robot_id="Z9Z9", **kw), state


class TestCodec(unittest.TestCase):
    def test_probe_layout(self):
        p = d.encode_probe(NONCE)
        self.assertEqual(len(p), 16)
        self.assertEqual(p, b"NTD1" + bytes([1, 1, 0, 0]) + NONCE)
        self.assertEqual(d.decode_probe(p), NONCE)

    def test_reply_roundtrip(self):
        info = d.build_info("A3_TEST", "Z9Z9", 39500, d.MODE_PUBLISH, True)
        r = d.encode_reply(NONCE, info)
        self.assertLess(len(r), d.MAX_REPLY_SIZE)
        self.assertEqual(r[:6], b"NTD1" + bytes([1, 2]))
        self.assertEqual(struct.unpack_from("<H", r, 6)[0], len(r) - 16)
        self.assertEqual(r[8:16], NONCE)
        self.assertEqual(json.loads(r[16:].decode("utf-8")), info)
        self.assertEqual(d.decode_reply(r, NONCE), info)
        self.assertEqual(info, {"v": 1, "product": "nova_teleop_bridge", "bridge_version": d.BRIDGE_VERSION,
                                "model": "A3_TEST", "robot_id": "Z9Z9", "data_port": 39500,
                                "mode": "publish", "sender_locked": True})

    def test_reply_rejects(self):
        r = d.encode_reply(NONCE, d.build_info("m", "i", 39500, d.MODE_DRY_RUN, False))
        with self.assertRaises(d.ProbeError):
            d.decode_reply(r, b"\0" * 8)                      # foreign nonce
        with self.assertRaises(d.ProbeError):
            d.decode_reply(r[:-1], NONCE)                     # length field mismatch
        with self.assertRaises(d.ProbeError):
            d.decode_reply(b"XTD1" + r[4:], NONCE)
        with self.assertRaises(d.ProbeError):
            d.decode_reply(d.encode_probe(NONCE), NONCE)      # a probe is not a reply

    def test_reply_size_is_bounded(self):
        with self.assertRaises(ValueError):
            d.encode_reply(NONCE, {"x": "y" * 600})
        worst = d.build_info("M" * 200, "I" * 200, 65535, d.MODE_DRY_RUN, False)
        self.assertLess(len(d.encode_reply(NONCE, worst)), d.MAX_REPLY_SIZE)
        self.assertEqual(len(worst["model"]), 48)
        self.assertEqual(len(worst["robot_id"]), 4)
        with self.assertRaises(ValueError):
            d.build_info("m", "i", 39500, "armed", False)

    def test_identity_discloses_only_the_tail(self):
        with tempfile.TemporaryDirectory() as tmp:
            model, name = os.path.join(tmp, "model"), os.path.join(tmp, "name")
            with open(model, "w") as fh:
                fh.write("A3 TEST/01\n")
            with open(name, "w") as fh:
                fh.write("EXAMPLE-0000-ABCD\nsecond line\n")
            self.assertEqual(d.read_robot_identity(model, name), ("A3_TEST_01", "ABCD"))
            self.assertEqual(d.read_robot_identity(os.path.join(tmp, "nope"), os.path.join(tmp, "nope")), ("", ""))


class TestResponder(unittest.TestCase):
    def test_answers_a_valid_probe(self):
        r, state = make_responder()
        reply = r.handle(d.encode_probe(NONCE), "192.168.1.30", 40000, 0.0)
        info = d.decode_reply(reply, NONCE)
        self.assertEqual((info["mode"], info["sender_locked"], info["data_port"]), ("dry-run", False, 39612))
        state["mode"], state["locked"] = d.MODE_PUBLISH, True
        info = d.decode_reply(r.handle(d.encode_probe(NONCE), "192.168.1.30", 40000, 0.1), NONCE)
        self.assertEqual((info["mode"], info["sender_locked"]), ("publish", True))
        self.assertEqual(r.stats["replies"], 2)

    def test_malformed_probes_are_dropped_silently(self):
        r, state = make_responder()
        good = d.encode_probe(NONCE)
        bad = {
            "length": [b"", good[:-1], good + b"\0", good * 2, b"NTD1"],
            "magic": [b"NTB1" + good[4:], b"\0" * 16],
            "version": [good[:4] + bytes([2]) + good[5:], good[:4] + bytes([0]) + good[5:]],
            "type": [good[:5] + bytes([2]) + good[6:], good[:5] + bytes([0]) + good[6:]],
            "reserved": [good[:6] + b"\x01\x00" + good[8:], good[:6] + b"\x00\x80" + good[8:]],
        }
        n = 0
        for reason, datagrams in bad.items():
            for dg in datagrams:
                n += 1
                self.assertIsNone(r.handle(dg, "192.168.1.30", 40000, 0.0), (reason, dg))
            self.assertEqual(r.stats["drop_" + reason], len(datagrams), reason)
        self.assertEqual(r.stats["rx"], n)
        self.assertEqual(r.stats["replies"], 0)
        self.assertEqual(state["calls"], 0)          # the bridge was not even asked for its status
        # a reply fed back in is not answered either (no ping-pong between two bridges)
        reply = d.encode_reply(NONCE, d.build_info("m", "i", 39500, d.MODE_DRY_RUN, False))
        self.assertIsNone(r.handle(reply, "192.168.1.30", 40000, 0.0))

    def test_never_replies_to_reserved_source_ports(self):
        r, _ = make_responder()
        for port in d.FORBIDDEN_PORTS + (0, 53, 123, 1023):
            self.assertIsNone(r.handle(d.encode_probe(NONCE), "192.168.1.30", port, 0.0), port)
        self.assertEqual(r.stats["replies"], 0)
        self.assertIsNotNone(r.handle(d.encode_probe(NONCE), "192.168.1.30", 1024, 0.0))

    def test_rate_limit_per_source(self):
        r, _ = make_responder(limiter=d.RateLimiter(per_source_burst=3, per_source_per_s=2,
                                                    global_burst=100, global_per_s=100))
        p = d.encode_probe(NONCE)
        got = [r.handle(p, "192.168.1.30", 40000, 0.0) is not None for _ in range(10)]
        self.assertEqual(got, [True] * 3 + [False] * 7)
        self.assertIsNotNone(r.handle(p, "192.168.1.31", 40000, 0.0))       # another source is unaffected
        self.assertIsNone(r.handle(p, "192.168.1.30", 40000, 0.25))         # half a token: not yet
        self.assertIsNotNone(r.handle(p, "192.168.1.30", 40000, 0.75))      # one token back
        self.assertIsNone(r.handle(p, "192.168.1.30", 40000, 0.75))
        got = [r.handle(p, "192.168.1.30", 40000, 100.0) is not None for _ in range(10)]
        self.assertEqual(sum(got), 3)                                       # refill is capped at the burst
        self.assertEqual(r.stats["drop_rate"], 16)

    def test_rate_limit_global_and_bounded_table(self):
        lim = d.RateLimiter(per_source_burst=1, per_source_per_s=1, global_burst=5, global_per_s=1, max_sources=4)
        r, _ = make_responder(limiter=lim)
        p = d.encode_probe(NONCE)
        got = [r.handle(p, "10.0.0.%d" % i, 40000, 0.0) is not None for i in range(20)]
        self.assertEqual(sum(got), 5)
        self.assertLessEqual(len(lim._sources), 4)

    def test_default_limits_allow_one_app_scan(self):
        r, _ = make_responder()
        p = d.encode_probe(NONCE)
        # one scan of the app: 3 destinations (subnet broadcast, limited broadcast, unicast), sent twice
        got = [r.handle(p, "192.168.1.30", 40000, t) is not None for t in (0.0, 0.0, 0.0, 0.7, 0.7, 0.7)]
        self.assertTrue(all(got))


class TestServerLoopback(unittest.TestCase):
    def test_one_reply_per_probe(self):
        r, _ = make_responder()
        srv = d.DiscoveryServer(r, "127.0.0.1", test_port())
        cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            cli.bind(("127.0.0.1", 0))
            cli.settimeout(0.3)
            cli.sendto(b"hello", ("127.0.0.1", srv.port))
            cli.sendto(d.encode_probe(NONCE), ("127.0.0.1", srv.port))
            cli.sendto(d.encode_probe(NONCE)[:-1], ("127.0.0.1", srv.port))
            time.sleep(0.05)
            self.assertEqual(srv.service(0.0), 3)
            data, addr = cli.recvfrom(2048)
            self.assertEqual(addr, ("127.0.0.1", srv.port))
            self.assertEqual(d.decode_reply(data, NONCE)["product"], "nova_teleop_bridge")
            with self.assertRaises(socket.timeout):
                cli.recvfrom(2048)                       # exactly one datagram came back
        finally:
            cli.close()
            srv.close()

    def test_forbidden_discovery_port(self):
        r, _ = make_responder()
        with self.assertRaises(ValueError):
            d.DiscoveryServer(r, "127.0.0.1", 25618)


class TestBridgeProcess(unittest.TestCase):
    """The real bridge program (dry-run) answering probes on loopback."""

    def start(self, *extra):
        self.data_port = test_port()
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        self.proc = subprocess.Popen(
            [sys.executable, BRIDGE, "--dry-run", "--listen-ip", "127.0.0.1", "--port", str(self.data_port),
             "--config", CONFIG, "--stats-interval", "0", *extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        self.addCleanup(self.stop)
        head = ""
        while True:                                   # the last start-up line is the discovery status
            line = self.proc.stdout.readline()
            head += line
            if not line or "] discovery: answering" in line or "] discovery: off" in line:
                return head

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.communicate()
        if self.proc.stdout:
            self.proc.stdout.close()

    def probe(self, port, payload, timeout=1.0):
        cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            cli.bind(("127.0.0.1", 0))
            cli.settimeout(timeout)
            cli.sendto(payload, ("127.0.0.1", port))
            try:
                return cli.recvfrom(2048)[0]
            except socket.timeout:
                return None
        finally:
            cli.close()

    def test_bridge_answers_and_state_is_untouched(self):
        dport = free_udp_port(start=39700)
        head = self.start("--discovery-port", str(dport))
        self.assertIn("answering NTD1 probes on UDP %d" % dport, head)
        info = d.decode_reply(self.probe(dport, d.encode_probe(NONCE)), NONCE)
        self.assertEqual(info["product"], "nova_teleop_bridge")
        self.assertEqual(info["data_port"], self.data_port)
        self.assertEqual(info["mode"], "dry-run")
        self.assertIs(info["sender_locked"], False)
        self.assertLessEqual(len(info["robot_id"]), 4)
        self.assertIsNone(self.probe(dport, b"NTD1 not a probe", timeout=0.3))
        # a probe sent to the DATA port is just an invalid NTB1 datagram: no reply, no sender lock
        self.assertIsNone(self.probe(self.data_port, d.encode_probe(NONCE), timeout=0.3))
        info = d.decode_reply(self.probe(dport, d.encode_probe(NONCE)), NONCE)
        self.assertIs(info["sender_locked"], False)
        self.proc.send_signal(signal.SIGINT)
        out, _ = self.proc.communicate(timeout=10)
        self.assertEqual(self.proc.returncode, 0)
        self.assertIn("valid=0", out)
        self.assertIn("'replies': 2", out)

    def test_sender_ip_is_reported_as_locked(self):
        dport = free_udp_port(start=39700)
        self.start("--discovery-port", str(dport), "--sender-ip", "127.0.0.1")
        info = d.decode_reply(self.probe(dport, d.encode_probe(NONCE)), NONCE)
        self.assertIs(info["sender_locked"], True)

    def test_no_discovery(self):
        head = self.start("--no-discovery")
        self.assertIn("discovery: off", head)

    def test_busy_discovery_port_does_not_stop_the_bridge(self):
        blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        blocker.bind(("127.0.0.1", free_udp_port(start=39700)))
        try:
            head = self.start("--discovery-port", str(blocker.getsockname()[1]))
            self.assertIn("discovery disabled", head)
            self.assertIn("mode=DRY-RUN", head)
            self.assertIsNone(self.proc.poll())
        finally:
            blocker.close()

    def test_argument_checks(self):
        from robot import teleop_bridge
        for argv in (["--discovery-port", "25618"], ["--discovery-port", "39650", "--port", "39650"],
                     ["--discovery-port", "80"]):
            with self.assertRaises(SystemExit), open(os.devnull, "w") as null:
                old, sys.stderr = sys.stderr, null
                try:
                    teleop_bridge.main(argv)
                finally:
                    sys.stderr = old
        a = teleop_bridge.build_parser().parse_args([])
        self.assertTrue(a.discovery)
        self.assertEqual(a.discovery_port, 39501)


if __name__ == "__main__":
    unittest.main()
