"""Control page service against a stand-in robot (nothing here talks to the real robot)."""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot import session
from robot import teleop_service as ts
from tests.util import free_udp_port


class FakeRobot(BaseHTTPRequestHandler):
    state = "Manual"
    refuse = False
    calls = []

    def log_message(self, *a):
        pass

    def do_POST(self):
        self.rfile.read(int(self.headers.get("Content-Length") or 0))
        cls = FakeRobot
        cls.calls.append(self.path)
        body = {"header": {"code": "0", "msg": ""}}
        if self.path.endswith("/GetSystemState"):
            body.update(cur_state=cls.state, cur_status="SystemStatus_IN_READY")
        elif self.path.endswith("/StartRemoteMotionStream"):
            if cls.refuse:
                body = {"header": {"code": "1", "msg": "agivslam is not recv heartbeat"}}
            else:
                cls.state = "RemoteMotionStream"
        elif self.path.endswith("/StopRemoteMotionStream"):
            cls.state = "Manual"
        raw = json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def wait_for(fn, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.05)
    return fn()


class TestService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.robot = ThreadingHTTPServer(("127.0.0.1", 0), FakeRobot)
        threading.Thread(target=cls.robot.serve_forever, daemon=True).start()
        url = "http://127.0.0.1:%d" % cls.robot.server_address[1]
        args = ts.build_parser().parse_args([
            "--listen-ip", "127.0.0.1", "--port", "0", "--settings", os.path.join(cls.tmp.name, "s.json"),
            "--run-dir", os.path.join(cls.tmp.name, "run"), "--sm-url", url, "--skillpilot-url", url,
            "--robot-input-port", str(free_udp_port(39400)), "--poll-s", "0.1", "--bridge-arg=--no-discovery",
            "--session-port", "0", "--no-standby", "--max-tries", "3", "--lockout-s", "0.4"])
        cls.args = args
        # Stand-in for the ROS recorder: writes a file until it is interrupted.
        fake = os.path.join(cls.tmp.name, "fake_recorder.py")
        with open(fake, "w") as fh:
            fh.write("import os, signal, sys, time\nos.makedirs(sys.argv[1])\nstop = []\n"
                     "signal.signal(signal.SIGINT, lambda *a: stop.append(1))\n"
                     "fh = open(os.path.join(sys.argv[1], 'bag_0.mcap'), 'wb')\n"
                     "while not stop:\n    fh.write(b'x' * 1000); fh.flush(); time.sleep(0.01)\n")
        args.recorder_cmd = lambda out, topics: [sys.executable, fake, out] + topics
        cls.svc = ts.Service(args)
        ts.Handler.svc = cls.svc
        cls.gate = ts.Gate(cls.svc, args)
        cls.gate_thread = threading.Thread(target=cls.gate.run, daemon=True)
        cls.gate_thread.start()
        cls.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.sid = 1000

    @classmethod
    def tearDownClass(cls):
        cls.svc.shutdown()
        cls.gate.stop()
        cls.gate_thread.join(timeout=5)
        cls.udp.close()
        cls.robot.shutdown()
        cls.tmp.cleanup()

    def setUp(self):
        FakeRobot.state, FakeRobot.refuse = "Manual", False
        self.svc.transition = None
        self.args.idle_close_s = 900.0
        self.connect("LAMP")
        self.cookie = ""
        code, _ = self.post("/api/unlock", {"word": "lamp"})
        self.assertEqual(code, 200)

    # ---- helpers ----
    def say(self, kind, word="LAMP", sid=None):
        self.udp.sendto(session.pack(kind, self.sid if sid is None else sid, word),
                        ("127.0.0.1", self.gate.session_port))

    def connect(self, word):
        """A new headset connection: the page opens (or re-keys) with this word."""
        type(self).sid += 1
        self.say(session.HELLO, word)
        self.assertTrue(wait_for(lambda: self.gate.page_open() and self.svc.word == word))
        self.base = "http://127.0.0.1:%d" % self.gate.web_port

    def get(self, path):
        req = urllib.request.Request(self.base + path, headers={"Cookie": self.cookie})
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.read()

    def post(self, path, body=None, header=True):
        h = {"Cookie": self.cookie}
        if header:
            h["X-Nova-Teleop"] = "1"
        req = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(), method="POST", headers=h)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                c = r.headers.get("Set-Cookie")
                if c:
                    self.cookie = c.split(";")[0]
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def page_refuses_connections(self):
        try:
            urllib.request.urlopen(self.base + "/", timeout=2).read()
            return False
        except OSError:
            return True

    # ---- page access ----
    def test_locked_until_the_word_is_typed(self):
        self.cookie = ""
        self.assertIn(b"Nova Teleop", self.get("/"))                     # lock screen is served
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.get("/api/status")
        self.assertEqual(cm.exception.code, 401)
        self.assertEqual(self.post("/api/teleop/start")[0], 401)
        self.assertEqual(FakeRobot.state, "Manual")
        self.assertEqual(self.post("/api/unlock", {"word": "NOPE"})[0], 403)
        self.assertEqual(self.post("/api/unlock", {"word": " Lamp "})[0], 200)
        self.assertTrue(self.status()["robot"] is not None)

    def test_wrong_words_cause_a_pause(self):
        self.cookie = ""
        for _ in range(3):
            self.assertEqual(self.post("/api/unlock", {"word": "NOPE"})[0], 403)
        code, r = self.post("/api/unlock", {"word": "LAMP"})             # right word, but paused
        self.assertEqual(code, 403)
        self.assertIn("wait", r["error"])
        time.sleep(0.5)
        self.assertEqual(self.post("/api/unlock", {"word": "LAMP"})[0], 200)

    def test_goodbye_closes_the_page(self):
        self.say(session.BYE)
        self.assertTrue(wait_for(lambda: not self.gate.page_open()))
        self.assertTrue(self.page_refuses_connections())
        self.say(session.HELLO)                                          # a late hello of the same connection
        time.sleep(0.7)
        self.assertFalse(self.gate.page_open())

    def test_new_connection_needs_the_new_word(self):
        old = self.cookie
        self.connect("FERN")
        self.cookie = old
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.get("/api/status")
        self.assertEqual(cm.exception.code, 401)
        self.assertEqual(self.post("/api/unlock", {"word": "LAMP"})[0], 403)
        self.assertEqual(self.post("/api/unlock", {"word": "FERN"})[0], 200)

    def test_idle_page_closes_and_use_keeps_it_open(self):
        self.args.idle_close_s = 1.2
        for _ in range(4):                                               # 2 s of "use"
            time.sleep(0.5)
            self.assertEqual(self.post("/api/ping")[0], 200)
        self.assertTrue(self.gate.page_open())
        for _ in range(3):                                               # status polling is not "use"
            time.sleep(0.3)
            try:
                self.get("/api/status")
            except OSError:
                pass
        self.assertTrue(wait_for(lambda: not self.gate.page_open(), timeout=4))
        self.say(session.HELLO)                                          # the same connection does not reopen it
        time.sleep(0.7)
        self.assertFalse(self.gate.page_open())

    def write_stream_status(self, valid_hz):
        with open(self.svc.status_file, "w") as fh:
            json.dump({"t": time.time(), "valid_hz": valid_hz, "mode": "robot-input", "state": "active"}, fh)

    def test_teleoperating_keeps_the_page_open_and_brings_it_back(self):
        self.args.idle_close_s = 1.0
        try:
            end = time.time() + 2.5
            while time.time() < end:                       # headset streams: nobody touches the page
                self.write_stream_status(100.0)
                self.say(session.HELLO)
                time.sleep(0.2)
            self.assertTrue(self.gate.page_open())
            os.remove(self.svc.status_file)                # headset taken off: stream stops
            end = time.time() + 4
            while time.time() < end and self.gate.page_open():
                self.say(session.HELLO)                    # the app is still alive and says hello
                time.sleep(0.2)
            self.assertFalse(self.gate.page_open())
            self.write_stream_status(100.0)                # headset back on: streaming again
            self.say(session.HELLO)
            self.assertTrue(wait_for(self.gate.page_open))
            self.base = "http://127.0.0.1:%d" % self.gate.web_port
            self.assertEqual(self.status()["robot"]["state"], "Manual")   # the old browser still works
        finally:
            try:
                os.remove(self.svc.status_file)
            except OSError:
                pass

    def test_goodbye_leaves_teleoperation_mode(self):
        self.assertEqual(self.post("/api/teleop/start")[0], 200)
        self.assertTrue(wait_for(lambda: self.svc.cache["robot"].get("state") == "RemoteMotionStream"))
        wait_for(lambda: self.svc.transition["done"])
        self.say(session.BYE)
        self.assertTrue(wait_for(lambda: FakeRobot.state == "Manual"))
        self.assertFalse(self.gate.page_open())

    def test_goodbye_can_leave_the_robot_alone(self):
        self.assertEqual(self.post("/api/settings", {"session": {"leave_teleop_on_disconnect": False}})[0], 200)
        try:
            self.post("/api/teleop/start")
            self.assertTrue(wait_for(lambda: self.svc.cache["robot"].get("state") == "RemoteMotionStream"))
            wait_for(lambda: self.svc.transition["done"])
            self.say(session.BYE)
            self.assertTrue(wait_for(lambda: not self.gate.page_open()))
            time.sleep(0.5)
            self.assertEqual(FakeRobot.state, "RemoteMotionStream")
        finally:
            self.svc.save_settings({"session": {"leave_teleop_on_disconnect": True}})

    def test_garbage_datagrams_are_ignored(self):
        self.say(session.BYE)
        self.assertTrue(wait_for(lambda: not self.gate.page_open()))
        good = session.pack(session.HELLO, 77, "LAMP")
        for bad in (b"", good[:-1], good[:-1] + bytes([good[-1] ^ 1]), b"XXXX" + good[4:]):
            self.udp.sendto(bad, ("127.0.0.1", self.gate.session_port))
        with self.assertRaises(ValueError):
            session.pack(session.HELLO, 1, "AB1D")
        time.sleep(0.7)
        self.assertFalse(self.gate.page_open())

    def status(self):
        return json.loads(self.get("/api/status"))

    def test_page_is_served(self):
        self.assertIn(b"Nova Teleop", self.get("/"))
        self.assertIn(b"<svg", self.get("/gestures.svg"))

    def test_status_reports_robot_state(self):
        s = wait_for(lambda: (lambda x: x if x["robot"]["reachable"] else None)(self.status()))
        self.assertEqual(s["robot"]["state"], "Manual")
        self.assertFalse(s["robot"]["teleop"])
        self.assertEqual([b["key"] for b in s["boards"]], ["head", "main", "compute"])
        self.assertFalse(s["bridge"]["running"])

    def test_buttons_need_the_header(self):
        code, _ = self.post("/api/teleop/start", header=False)
        self.assertEqual(code, 403)
        self.assertEqual(FakeRobot.state, "Manual")

    def test_teleop_start_and_stop(self):
        self.assertEqual(self.post("/api/teleop/start")[0], 200)
        s = wait_for(lambda: (lambda x: x if x["robot"]["teleop"] else None)(self.status()))
        self.assertTrue(s["robot"]["teleop"])
        wait_for(lambda: self.status()["transition"]["done"])
        self.assertEqual(self.post("/api/teleop/stop")[0], 200)
        self.assertTrue(wait_for(lambda: self.status()["robot"]["state"] == "Manual"))

    def test_refusal_is_shown(self):
        FakeRobot.refuse = True
        self.post("/api/teleop/start")
        tr = wait_for(lambda: (lambda t: t if t and t["done"] else None)(self.status()["transition"]))
        self.assertIn("agivslam", tr["error"])
        self.assertEqual(FakeRobot.state, "Manual")

    def test_bridge_start_stop_in_test_mode(self):
        port = free_udp_port(39450)
        self.assertEqual(self.post("/api/settings", {"bridge": {"port": port}})[0], 200)
        self.assertEqual(self.post("/api/bridge/start", {"mode": "test"})[0], 200)
        try:
            s = wait_for(lambda: (lambda x: x if x["bridge"].get("link") else None)(self.status()))
            self.assertTrue(s["bridge"]["running"])
            self.assertEqual(s["bridge"]["mode"], "test")
            self.assertEqual(s["bridge"]["link"]["mode"], "dry-run")
            self.assertEqual(self.post("/api/bridge/start", {"mode": "test"})[0], 409)   # already in test mode
        finally:
            self.assertEqual(self.post("/api/bridge/stop")[0], 200)
        self.assertFalse(self.status()["bridge"]["running"])

    def test_settings_are_validated_and_kept(self):
        code, r = self.post("/api/settings", {"data_collection": {"server_url": "ftp://x"}})
        self.assertEqual(code, 409)
        code, r = self.post("/api/settings", {"bridge": {"port": 25618}})
        self.assertEqual(code, 409)
        code, r = self.post("/api/settings", {"data_collection": {
            "server_url": "https://data.example", "upload_format": "raw", "enabled": True, "task": "t",
            "cameras": {"chest": False}, "unknown": 1}})
        self.assertEqual(code, 200)
        d = r["settings"]["data_collection"]
        self.assertEqual((d["server_url"], d["upload_format"], d["task"]), ("https://data.example", "raw", "t"))
        self.assertTrue(d["enabled"])
        self.assertFalse(d["cameras"]["chest"])
        self.assertTrue(d["cameras"]["waist"])
        self.assertNotIn("unknown", d)
        with open(self.svc.settings_path) as fh:
            self.assertEqual(json.load(fh)["data_collection"]["upload_format"], "raw")

    # ---- recording ----
    def recording_settings(self, enabled=True):
        code, _ = self.post("/api/settings", {"data_collection": {
            "enabled": enabled, "server_url": "", "spool_dir": os.path.join(self.tmp.name, "spool"), "min_free_gb": 1}})
        self.assertEqual(code, 200)

    def test_recording_buttons(self):
        self.recording_settings(False)
        self.assertEqual(self.post("/api/record/start")[0], 409)        # switched off in Settings
        self.recording_settings(True)
        code, r = self.post("/api/record/start")
        self.assertEqual(code, 200)
        self.assertTrue(self.status()["data"]["recording"])
        self.assertEqual(self.post("/api/record/start")[0], 409)        # already recording
        time.sleep(0.2)
        code, r2 = self.post("/api/record/stop")
        self.assertEqual((code, r2["episode"]["id"]), (200, r["episode"]))
        d = self.status()["data"]
        self.assertFalse(d["recording"])
        self.assertEqual(d["pending"]["count"] >= 1, True)              # no server set: it stays on the robot
        ep_dir = os.path.join(self.tmp.name, "spool", r["episode"])
        self.assertTrue(os.path.exists(os.path.join(ep_dir, "manifest.json")))
        with open(os.path.join(ep_dir, "episode.json")) as fh:
            self.assertEqual(json.load(fh)["context_start"]["robot"]["state"], "Manual")

    def test_goodbye_ends_the_episode(self):
        self.recording_settings(True)
        self.assertEqual(self.post("/api/record/start")[0], 200)
        self.say(session.BYE)
        self.assertTrue(wait_for(lambda: self.svc.data.status()["last_episode"] is not None
                                 and not self.svc.data.recording()))
        self.assertIn("headset", self.svc.data.status()["last_episode"]["stop_reason"])


if __name__ == "__main__":
    unittest.main()
