"""Episode recording, upload, server check and delete-after-receipt.  No robot, no ROS:
the recorder is replaced by a small stand-in program."""
import http.client
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from robot import data_collection as dc  # noqa: E402
from server import ingest_server as ing  # noqa: E402

TOKEN = "test-token-123"

FAKE_RECORDER = r'''
import os, signal, sys, time
out, topics = sys.argv[1], sys.argv[2:]
os.makedirs(out)
stop = []
signal.signal(signal.SIGINT, lambda *a: stop.append(1))
fh = open(os.path.join(out, "bag_0.mcap"), "wb")
n = 0
while not stop:
    fh.write(os.urandom(4096)); fh.flush(); n += 1; time.sleep(0.01)
fh.close()
with open(os.path.join(out, "metadata.yaml"), "w") as m:
    m.write("rosbag2_bagfile_information:\n  topics_with_message_count:\n")
    for t in topics:
        c = 0 if "wrist" in t else n
        m.write("    - topic_metadata:\n        name: %s\n        type: x/msg/Y\n      message_count: %d\n" % (t, c))
'''

CFG = {"ros": {"storage": "mcap"},
       "topics": [{"name": "/cam/chest/stream", "group": "chest", "required": True},
                  {"name": "/cam/wrist_left/stream", "group": "wrist_left", "required": True},
                  {"name": "/robot/joints", "group": "robot", "required": True}]}


def load_json(path):
    with open(path) as fh:
        return json.load(fh)


def open_write(path, text):
    with open(path, "w") as fh:
        fh.write(text)


def wait_for(cond, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="nt_dc_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.fake = os.path.join(self.tmp, "fake_recorder.py")
        with open(self.fake, "w") as fh:
            fh.write(FAKE_RECORDER)
        self.server_root = os.path.join(self.tmp, "server")
        self.srv = ing.make_server(self.server_root, TOKEN, "127.0.0.1", 0, min_free_gb=0)
        self.srv.RequestHandlerClass.log_message = lambda *a, **k: None
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.addCleanup(self.srv.server_close)
        self.addCleanup(self.srv.shutdown)
        self.url = "http://127.0.0.1:%d" % self.srv.server_address[1]
        self.settings = {"enabled": True, "task": "pick up the cup", "operator": "op", "server_url": "",
                         "server_token": TOKEN, "keep_local_copy": False,
                         "spool_dir": os.path.join(self.tmp, "spool"), "max_spool_gb": 10, "min_free_gb": 0.001,
                         "cameras": {"chest": True, "wrist_left": False}}
        self.dcs = []

    def make(self, **kw):
        d = dc.DataCollection(lambda: self.settings, CFG,
                              build_cmd=lambda out, topics: [sys.executable, self.fake, out] + topics,
                              context=lambda: {"robot": {"state": "RemoteMotionStream"}},
                              upload_interval=0.1, watch_interval=0.1, **kw)
        self.addCleanup(d.shutdown)
        return d

    def record(self, d, seconds=0.3):
        ep = d.start()
        time.sleep(seconds)
        return ep, d.stop()

    def server_dir(self, ep):
        return os.path.join(self.server_root, "episodes", ep)


class TestRecording(Base):
    def test_episode_has_recording_description_and_manifest(self):
        d = self.make()
        ep, summary = self.record(d)
        ep_dir = os.path.join(self.settings["spool_dir"], ep)
        info = load_json(os.path.join(ep_dir, "episode.json"))
        self.assertEqual(info["task"], "pick up the cup")
        self.assertEqual(info["context_start"]["robot"]["state"], "RemoteMotionStream")
        self.assertEqual([t["name"] for t in info["topics"]], ["/cam/chest/stream", "/robot/joints"])
        self.assertTrue(all(t["messages"] > 0 for t in info["topics"]))
        self.assertEqual(info["warnings"], [])
        man = load_json(os.path.join(ep_dir, "manifest.json"))
        paths = [f["path"] for f in man["files"]]
        self.assertEqual(paths, sorted(["bag/bag_0.mcap", "bag/metadata.yaml", "episode.json", "record.log"]))
        for f in man["files"]:
            self.assertEqual(dc.sha256_file(os.path.join(ep_dir, f["path"])), f["sha256"])
        self.assertEqual(summary["bytes"], man["total_bytes"])

    def test_silent_required_topic_is_a_warning(self):
        self.settings["cameras"]["wrist_left"] = True
        d = self.make()
        _, summary = self.record(d)
        self.assertEqual(summary["warnings"], ["no messages recorded from /cam/wrist_left/stream"])

    def test_cannot_record_when_switched_off_or_twice(self):
        d = self.make()
        self.settings["enabled"] = False
        with self.assertRaises(RuntimeError):
            d.start()
        self.settings["enabled"] = True
        d.start()
        with self.assertRaises(RuntimeError):
            d.start()
        d.stop()
        with self.assertRaises(RuntimeError):
            d.stop()

    def test_refuses_to_start_without_free_space(self):
        d = self.make()
        self.settings["min_free_gb"] = 10 ** 9
        with self.assertRaisesRegex(RuntimeError, "not enough free space"):
            d.start()
        self.assertEqual(os.listdir(self.settings["spool_dir"]), [])

    def test_stops_by_itself_when_the_spool_limit_is_reached(self):
        d = self.make()
        d.start()
        self.settings["max_spool_gb"] = 1e-6
        self.assertTrue(wait_for(lambda: d.status()["last_episode"]))
        self.assertIn("storage limit", d.status()["last_episode"]["stop_reason"])

    def test_recorder_that_dies_ends_the_episode(self):
        d = dc.DataCollection(lambda: self.settings, CFG, build_cmd=lambda out, topics: [sys.executable, "-c", "pass"],
                              upload_interval=0.1, watch_interval=0.1)
        self.addCleanup(d.shutdown)
        d.start()
        self.assertTrue(wait_for(lambda: d.status()["last_episode"]))
        last = d.status()["last_episode"]
        self.assertIn("stopped by itself", last["stop_reason"])
        self.assertTrue(last["warnings"])

    def test_raw_camera_topics_are_refused_in_the_config(self):
        p = os.path.join(self.tmp, "rec.json")
        for bad in ("/hal/chest_front_d457_camera/depth", "/hal/chest_front_d457_camera/rgb"):
            open_write(p, json.dumps({"ros": {}, "topics": [{"name": bad, "group": "chest"}]}))
            with self.assertRaises(ValueError):
                dc.load_recording_config(p)
        cfg = dc.load_recording_config(os.path.join(ROOT, "config", "recording.json"))
        cmd = dc.ros_recorder_command(cfg)("/tmp/x/bag", ["/a/b"])
        self.assertIn("ros2 bag record -s mcap -o /tmp/x/bag", cmd[2])


class TestHandOver(Base):
    def test_episode_is_deleted_only_after_the_server_confirmed_it(self):
        d = self.make()
        ep, _ = self.record(d)
        ep_dir = os.path.join(self.settings["spool_dir"], ep)
        man = load_json(os.path.join(ep_dir, "manifest.json"))
        time.sleep(0.5)
        self.assertTrue(os.path.isdir(ep_dir), "no server set: the episode must stay")
        self.settings["server_url"] = self.url
        d.retry_now()
        self.assertTrue(wait_for(lambda: not os.path.exists(ep_dir)))
        receipt = load_json(os.path.join(self.server_dir(ep), "receipt.json"))
        self.assertTrue(dc.receipt_matches(man, receipt))
        for f in man["files"]:
            self.assertEqual(ing.sha256_file(os.path.join(self.server_dir(ep), f["path"])), f["sha256"])
        self.assertEqual(d.status()["upload"]["last_ok"], ep)
        self.assertEqual(d.status()["pending"]["count"], 0)

    def test_wrong_token_keeps_the_episode(self):
        self.settings.update(server_url=self.url, server_token="wrong-token")
        d = self.make()
        ep, _ = self.record(d)
        self.assertTrue(wait_for(lambda: d.status()["upload"]["state"] == "waiting"))
        self.assertIn("401", d.status()["upload"]["error"])
        self.assertTrue(os.path.isdir(os.path.join(self.settings["spool_dir"], ep)))
        self.settings["server_token"] = TOKEN
        d.retry_now()
        self.assertTrue(wait_for(lambda: d.status()["pending"]["count"] == 0, 70))

    def test_unreachable_server_keeps_the_episode(self):
        self.settings["server_url"] = "http://127.0.0.1:9"
        d = self.make()
        ep, _ = self.record(d)
        self.assertTrue(wait_for(lambda: d.status()["upload"]["state"] == "waiting"))
        self.assertTrue(os.path.isdir(os.path.join(self.settings["spool_dir"], ep)))

    def test_file_damaged_on_the_server_blocks_the_receipt_and_the_delete(self):
        d = self.make()
        ep, _ = self.record(d)
        ep_dir = os.path.join(self.settings["spool_dir"], ep)
        man = load_json(os.path.join(ep_dir, "manifest.json"))
        s = dict(self.settings, server_url=self.url)
        real = ing.verify_episode

        def damaged(ep_dir_srv, manifest):                 # the disk flips a bit after the upload
            with open(os.path.join(ep_dir_srv, "bag", "bag_0.mcap"), "r+b") as fh:
                fh.write(b"\x00\x01\x02\x03")
            return real(ep_dir_srv, manifest)
        ing.verify_episode = damaged
        try:
            with self.assertRaisesRegex(RuntimeError, "checksum"):
                d.upload_episode(s, ep, ep_dir, man)
        finally:
            ing.verify_episode = real
        self.assertTrue(os.path.isdir(ep_dir))
        self.assertFalse(os.path.exists(os.path.join(self.server_dir(ep), "receipt.json")))
        d.upload_episode(s, ep, ep_dir, man)               # second try repairs the file and succeeds
        self.assertFalse(os.path.exists(ep_dir))

    def test_receipt_must_cover_every_file(self):
        man = {"episode": "e1", "files": [{"path": "a", "size": 1, "sha256": "x"}, {"path": "b", "size": 2, "sha256": "y"}]}
        ok = {"verified": True, "episode": "e1", "files": list(man["files"])}
        self.assertTrue(dc.receipt_matches(man, ok))
        self.assertFalse(dc.receipt_matches(man, dict(ok, verified=False)))
        self.assertFalse(dc.receipt_matches(man, dict(ok, episode="e2")))
        self.assertFalse(dc.receipt_matches(man, dict(ok, files=man["files"][:1])))
        self.assertFalse(dc.receipt_matches(man, dict(ok, files=[man["files"][0], {"path": "b", "size": 2, "sha256": "z"}])))
        self.assertFalse(dc.receipt_matches(man, {}))
        self.assertFalse(dc.receipt_matches({"episode": "e1", "files": []}, {"verified": True, "episode": "e1", "files": []}))

    def test_keep_local_copy_keeps_it_and_does_not_upload_twice(self):
        self.settings.update(server_url=self.url, keep_local_copy=True)
        d = self.make()
        ep, _ = self.record(d)
        ep_dir = os.path.join(self.settings["spool_dir"], ep)
        self.assertTrue(wait_for(lambda: os.path.exists(os.path.join(ep_dir, "receipt.json"))))
        self.assertEqual(d.pending(), [])

    def test_unfinished_episode_is_never_uploaded_or_deleted(self):
        self.settings["server_url"] = self.url
        d = self.make()
        broken = os.path.join(self.settings["spool_dir"], "20260101_000000_dead")
        os.makedirs(os.path.join(broken, "bag"))
        open_write(os.path.join(broken, "bag", "bag_0.mcap"), "half")
        d.retry_now()
        time.sleep(0.5)
        self.assertTrue(os.path.isdir(broken))
        self.assertFalse(os.path.exists(self.server_dir("20260101_000000_dead")))


class TestServer(Base):
    def req(self, method, path, body=None, headers=None, token=TOKEN):
        c = http.client.HTTPConnection("127.0.0.1", self.srv.server_address[1], timeout=5)
        h = {"X-Nova-Token": token}
        h.update(headers or {})
        c.request(method, path, body=body, headers=h)
        r = c.getresponse()
        data = json.loads(r.read().decode() or "{}")
        c.close()
        return r.status, data

    def put(self, path, body, sha=None):
        import hashlib
        return self.req("PUT", path, body, {"X-Sha256": sha or hashlib.sha256(body).hexdigest(),
                                            "Content-Length": str(len(body))})

    def test_needs_the_token(self):
        self.assertEqual(self.req("GET", "/api/health", token="nope")[0], 401)
        self.assertEqual(self.req("GET", "/api/health")[0], 200)

    def test_damaged_upload_is_not_stored(self):
        st, _ = self.put("/api/episodes/e1/files/bag/a.mcap", b"hello", sha="0" * 64)
        self.assertEqual(st, 422)
        self.assertFalse(os.path.exists(os.path.join(self.server_dir("e1"), "bag", "a.mcap")))
        self.assertEqual(os.listdir(os.path.join(self.server_dir("e1"), "bag")), [])

    def test_paths_cannot_leave_the_episode(self):
        for bad in ("/api/episodes/e1/files/../x", "/api/episodes/e1/files/a/../../x", "/api/episodes/e1/files/.hidden",
                    "/api/episodes/e1/files/receipt.json", "/api/episodes/e1/files/a/b/c/d/e",
                    "/api/episodes/..%2Fx/files/a", "/api/episodes/e1/files/x.part"):
            self.assertIn(self.put(bad, b"x")[0], (400, 404), bad)
        self.assertFalse(os.path.exists(os.path.join(self.server_root, "x")))

    def test_commit_fails_when_a_file_is_missing_and_closed_episodes_are_final(self):
        import hashlib
        body = b"data"
        sha = hashlib.sha256(body).hexdigest()
        self.assertEqual(self.put("/api/episodes/e2/files/a.bin", body)[0], 201)
        man = {"episode": "e2", "files": [{"path": "a.bin", "size": 4, "sha256": sha},
                                          {"path": "b.bin", "size": 4, "sha256": sha}]}
        st, js = self.req("POST", "/api/episodes/e2/commit", json.dumps(man).encode())
        self.assertEqual((st, js["verified"]), (409, False))
        man["files"].pop()
        st, js = self.req("POST", "/api/episodes/e2/commit", json.dumps(man).encode())
        self.assertEqual((st, js["verified"]), (200, True))
        self.assertEqual(self.put("/api/episodes/e2/files/a.bin", b"other")[0], 409)
        self.assertEqual(self.req("GET", "/api/health")[1]["episodes_stored"], 1)

    def test_refuses_uploads_when_the_server_disk_is_low(self):
        self.srv.RequestHandlerClass.min_free = 10 ** 18
        self.assertEqual(self.put("/api/episodes/e3/files/a.bin", b"data")[0], 507)


if __name__ == "__main__":
    unittest.main()
