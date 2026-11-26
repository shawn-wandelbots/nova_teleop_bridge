"""Control page service against a stand-in robot (nothing here talks to the real robot)."""
import json
import os
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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
            "--robot-input-port", str(free_udp_port(39400)), "--poll-s", "0.1", "--bridge-arg=--no-discovery"])
        cls.svc = ts.Service(args)
        ts.Handler.svc = cls.svc
        cls.httpd = ThreadingHTTPServer(("127.0.0.1", 0), ts.Handler)
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()
        cls.base = "http://127.0.0.1:%d" % cls.httpd.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.svc.stop_bridge()
        cls.httpd.shutdown()
        cls.robot.shutdown()
        cls.tmp.cleanup()

    def setUp(self):
        FakeRobot.state, FakeRobot.refuse = "Manual", False
        self.svc.transition = None

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return r.read()

    def post(self, path, body=None, header=True):
        req = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(), method="POST",
                                     headers={"X-Nova-Teleop": "1"} if header else {})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def status(self):
        return json.loads(self.get("/api/status"))

    def test_page_is_served(self):
        self.assertIn(b"Nova Teleop", self.get("/"))

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
            self.assertEqual(self.post("/api/bridge/start", {"mode": "live"})[0], 409)   # already running
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
            "cameras": {"head": False}, "unknown": 1}})
        self.assertEqual(code, 200)
        d = r["settings"]["data_collection"]
        self.assertEqual((d["server_url"], d["upload_format"], d["task"]), ("https://data.example", "raw", "t"))
        self.assertFalse(d["enabled"])                      # recording is not built: cannot be switched on
        self.assertFalse(d["cameras"]["head"])
        self.assertNotIn("unknown", d)
        with open(self.svc.settings_path) as fh:
            self.assertEqual(json.load(fh)["data_collection"]["upload_format"], "raw")


if __name__ == "__main__":
    unittest.main()
