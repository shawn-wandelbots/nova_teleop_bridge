"""Episode recording on the robot, and safe hand-over to a server.

One episode = one directory in the spool:

    <spool>/<episode id>/
        bag/              recording made by the standard ROS 2 recorder (MCAP): every message
                          exactly as it was published, with its type and receive time
        episode.json      who / what / when, message counts, warnings
        record.log        what the recorder printed
        manifest.json     size and SHA-256 of every file above; written LAST, so its presence
                          means "complete, ready to upload"

Hand-over (see server/ingest_server.py for the other side):

    1. each file is sent with its SHA-256; the server refuses a file that does not match
    2. "commit": the server reads every stored file back from its disk, checks it against the
       manifest and answers with a receipt
    3. only when the receipt lists every file with the right size and hash is the episode
       deleted here.  Anything else: the episode stays and the upload is tried again.

Stdlib only.  The recorder itself is an external command (``ros2 bag record``), so nothing
here imports ROS and all of it can be tested on any computer.
"""
from __future__ import annotations

import hashlib
import http.client
import json
import os
import re
import secrets
import shlex
import shutil
import signal
import socket
import subprocess
import threading
import time
import urllib.parse

EPISODE_SCHEMA = "nova_teleop_episode/1"
ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
GB = 1024 ** 3


def log(msg):
    print("[data %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            b = fh.read(chunk)
            if not b:
                return h.hexdigest()
            h.update(b)


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def free_bytes(path):
    p = path
    while not os.path.exists(p):
        p = os.path.dirname(p) or "/"
    return shutil.disk_usage(p).free


def build_manifest(ep_dir, episode_id):
    files = []
    for root, _, names in os.walk(ep_dir):
        for n in sorted(names):
            full = os.path.join(root, n)
            rel = os.path.relpath(full, ep_dir).replace(os.sep, "/")
            if rel == "manifest.json":
                continue
            files.append({"path": rel, "size": os.path.getsize(full), "sha256": sha256_file(full)})
    files.sort(key=lambda f: f["path"])
    return {"episode": episode_id, "files": files, "total_bytes": sum(f["size"] for f in files)}


def receipt_matches(manifest, receipt):
    """True only if the server's receipt confirms every file of the manifest, and nothing else."""
    if not isinstance(receipt, dict) or receipt.get("verified") is not True:
        return False
    if receipt.get("episode") != manifest["episode"]:
        return False
    want = {f["path"]: (f["size"], f["sha256"]) for f in manifest["files"]}
    got = receipt.get("files")
    if not isinstance(got, list):
        return False
    try:
        have = {f["path"]: (f["size"], f["sha256"]) for f in got}
    except (KeyError, TypeError):
        return False
    return bool(want) and have == want


def parse_bag_counts(metadata_yaml_text):
    """Message count per topic from rosbag2's metadata.yaml (no YAML library needed)."""
    counts = {}
    name = None
    for line in metadata_yaml_text.splitlines():
        m = re.match(r"\s*(?:-\s*)?(?:topic_metadata:\s*)?name:\s*(\S+)\s*$", line)
        if m and m.group(1).startswith("/"):
            name = m.group(1).strip("'\"")
            continue
        m = re.match(r"\s*message_count:\s*(\d+)\s*$", line)
        if m and name is not None:
            counts[name] = int(m.group(1))
            name = None
    return counts


def load_recording_config(path):
    with open(path) as fh:
        cfg = json.load(fh)
    for t in cfg["topics"]:
        n = t["name"]
        if not re.match(r"^/[A-Za-z0-9_/]+$", n):
            raise ValueError("bad topic name %r" % n)
        if re.search(r"/(rgb|depth|compressed_depth|image_raw|stream_decode)$", n):
            raise ValueError("raw camera topic %r must not be recorded (see config/recording.json)" % n)
    return cfg


def ros_recorder_command(cfg):
    """Returns build(out_dir, topics) -> argv for ``ros2 bag record`` with the robot's ROS set-up."""
    r = cfg["ros"]

    def build(out_dir, topics):
        lines = ["source %s" % shlex.quote(r["setup"]), "export ROS_DOMAIN_ID=%d" % int(r["domain_id"])]
        for p in r.get("extra_prefixes", []):
            q = shlex.quote(p)
            lines.append('export AMENT_PREFIX_PATH=%s:"$AMENT_PREFIX_PATH" PYTHONPATH=%s/%s:"$PYTHONPATH" '
                         'LD_LIBRARY_PATH=%s/lib:"$LD_LIBRARY_PATH"' % (q, q, r["python_lib"], q))
        rec = "exec nice -n %d ros2 bag record -s %s -o %s --disable-keyboard-controls %s" % (
            int(r.get("nice", 5)), shlex.quote(r.get("storage", "mcap")), shlex.quote(out_dir),
            " ".join(shlex.quote(t) for t in topics))
        return ["bash", "-c", "\n".join(lines + [rec])]
    return build


class DataCollection:
    """Records episodes into the spool and uploads finished ones.  Thread-safe."""

    def __init__(self, get_settings, recording_cfg, build_cmd=None, context=None,
                 upload_interval=5.0, watch_interval=2.0, stop_timeout=20.0):
        self.get_settings = get_settings          # () -> the "data_collection" settings dict
        self.cfg = recording_cfg
        self.build_cmd = build_cmd or ros_recorder_command(recording_cfg)
        self.context = context or (lambda: {})    # () -> dict stored in episode.json
        self.upload_interval, self.watch_interval, self.stop_timeout = upload_interval, watch_interval, stop_timeout
        self.lock = threading.RLock()
        self.rec = None                           # running episode: dict
        self.last_episode = None                  # summary of the last finished one
        self.upload = {"state": "idle", "episode": "", "error": "", "last_ok": "", "last_ok_t": 0,
                       "sent_bytes": 0, "total_bytes": 0}
        self.closing = False
        self._wake = threading.Event()
        threading.Thread(target=self._upload_loop, daemon=True).start()
        threading.Thread(target=self._watch_loop, daemon=True).start()

    # ---- settings helpers -------------------------------------------------------------------
    def spool(self):
        return os.path.abspath(os.path.expanduser(self.get_settings()["spool_dir"]))

    def selected_topics(self):
        cams = self.get_settings().get("cameras", {})
        return [t for t in self.cfg["topics"] if t["group"] == "robot" or cams.get(t["group"])]

    # ---- recording --------------------------------------------------------------------------
    def recording(self):
        with self.lock:
            return self.rec is not None

    def start(self):
        s = self.get_settings()
        with self.lock:
            if not s["enabled"]:
                raise RuntimeError("data collection is switched off in Settings")
            if self.rec is not None:
                raise RuntimeError("an episode is already being recorded")
            spool = self.spool()
            os.makedirs(spool, exist_ok=True)
            free = free_bytes(spool)
            if free < s["min_free_gb"] * GB:
                raise RuntimeError("not enough free space on the robot: %.1f GB free, %.1f GB needed"
                                   % (free / GB, s["min_free_gb"]))
            if dir_bytes(spool) >= s["max_spool_gb"] * GB:
                raise RuntimeError("temporary storage is full: waiting for uploads to finish")
            topics = self.selected_topics()
            ep_id = time.strftime("%Y%m%d_%H%M%S") + "_" + secrets.token_hex(2)
            ep_dir = os.path.join(spool, ep_id)
            os.makedirs(ep_dir)
            logfh = open(os.path.join(ep_dir, "record.log"), "ab")
            try:
                proc = subprocess.Popen(self.build_cmd(os.path.join(ep_dir, "bag"), [t["name"] for t in topics]),
                                        stdout=logfh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                        start_new_session=True)
            except OSError as e:
                logfh.close()
                shutil.rmtree(ep_dir, ignore_errors=True)
                raise RuntimeError("could not start the recorder: %s" % e)
            logfh.close()
            self.rec = {"id": ep_id, "dir": ep_dir, "proc": proc, "t0": time.time(), "topics": topics,
                        "task": s["task"], "operator": s["operator"], "context_start": self.context()}
        log("episode %s: recording started" % ep_id)
        return ep_id

    def stop(self, reason="stopped by operator"):
        with self.lock:
            rec, self.rec = self.rec, None
        if rec is None:
            raise RuntimeError("no episode is being recorded")
        proc = rec["proc"]
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)      # the recorder closes its file on Ctrl-C
            except OSError:
                pass
            try:
                proc.wait(timeout=self.stop_timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except OSError:
                    pass
                proc.wait()
                reason += "; recorder did not stop in time and was killed (file may be incomplete)"
        t1 = time.time()
        counts = {}
        try:
            with open(os.path.join(rec["dir"], "bag", "metadata.yaml")) as fh:
                counts = parse_bag_counts(fh.read())
        except OSError:
            pass
        warnings = []
        if not counts:
            warnings.append("the recorder left no summary: the recording may be empty or damaged")
        for t in rec["topics"]:
            if t.get("required") and not counts.get(t["name"]):
                warnings.append("no messages recorded from %s" % t["name"])
        info = {
            "schema": EPISODE_SCHEMA, "episode": rec["id"], "task": rec["task"], "operator": rec["operator"],
            "start_unix": rec["t0"], "end_unix": t1, "duration_s": round(t1 - rec["t0"], 3),
            "start_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(rec["t0"])),
            "stop_reason": reason, "recorder_exit_code": proc.returncode, "host": socket.gethostname(),
            "storage": {"path": "bag", "format": "rosbag2 " + self.cfg["ros"].get("storage", "mcap"),
                        "note": "messages are stored unchanged as published (CDR). The log time of "
                                "each message is the head computer's clock at reception; camera "
                                "messages also carry the camera's own timestamp."},
            "topics": [dict(t, messages=counts.get(t["name"], 0)) for t in rec["topics"]],
            "warnings": warnings,
            "context_start": rec["context_start"], "context_end": self.context(),
        }
        with open(os.path.join(rec["dir"], "episode.json"), "w") as fh:
            json.dump(info, fh, indent=2)
        manifest = build_manifest(rec["dir"], rec["id"])
        tmp = os.path.join(rec["dir"], "manifest.json.tmp")
        with open(tmp, "w") as fh:
            json.dump(manifest, fh, indent=2)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, os.path.join(rec["dir"], "manifest.json"))
        summary = {"id": rec["id"], "duration_s": info["duration_s"], "bytes": manifest["total_bytes"],
                   "warnings": warnings, "stop_reason": reason}
        with self.lock:
            self.last_episode = summary
        log("episode %s: stopped after %.1f s, %.1f MB, %d warning(s)"
            % (rec["id"], info["duration_s"], manifest["total_bytes"] / 1e6, len(warnings)))
        self._wake.set()
        return summary

    def stop_if_recording(self, reason):
        try:
            return self.stop(reason)
        except RuntimeError:
            return None

    def shutdown(self):
        self.closing = True
        self.stop_if_recording("service stopped")
        self._wake.set()

    def _watch_loop(self):
        """Stops a running episode when space runs out or the recorder dies."""
        while not self.closing:
            time.sleep(self.watch_interval)
            with self.lock:
                rec = self.rec
            if rec is None:
                continue
            try:
                s = self.get_settings()
                spool = self.spool()
                if rec["proc"].poll() is not None:
                    self.stop_if_recording("the recorder stopped by itself (exit code %s)" % rec["proc"].returncode)
                elif free_bytes(spool) < min(1.0, s["min_free_gb"]) * GB:
                    self.stop_if_recording("stopped: the robot's disk is almost full")
                elif dir_bytes(spool) >= s["max_spool_gb"] * GB:
                    self.stop_if_recording("stopped: temporary storage limit reached")
            except Exception as e:                      # never let the guard thread die
                log("watch: %s" % e)

    # ---- spool ------------------------------------------------------------------------------
    def pending(self):
        """Finished episodes waiting for upload, oldest first: [(id, dir, manifest)]."""
        out = []
        spool = self.spool()
        try:
            names = sorted(os.listdir(spool))
        except OSError:
            return out
        with self.lock:
            active = self.rec["id"] if self.rec else None
        for n in names:
            d = os.path.join(spool, n)
            if n == active or not ID_RE.match(n) or not os.path.isdir(d):
                continue
            if os.path.exists(os.path.join(d, "receipt.json")):
                continue                                # uploaded, kept on purpose
            try:
                with open(os.path.join(d, "manifest.json")) as fh:
                    out.append((n, d, json.load(fh)))
            except (OSError, ValueError):
                continue                                # unfinished (e.g. power cut): left alone
        return out

    # ---- upload -----------------------------------------------------------------------------
    def _upload_loop(self):
        backoff = self.upload_interval
        while not self.closing:
            self._wake.wait(backoff)
            self._wake.clear()
            if self.closing:
                return
            try:
                s = self.get_settings()
                if not s["server_url"]:
                    self._set_upload(state="idle", error="")
                    continue
                todo = self.pending()
                if not todo:
                    self._set_upload(state="idle", error="", episode="")
                    backoff = self.upload_interval
                    continue
                ep_id, ep_dir, manifest = todo[0]
                self._set_upload(state="uploading", episode=ep_id, error="", sent_bytes=0,
                                 total_bytes=manifest["total_bytes"])
                self.upload_episode(s, ep_id, ep_dir, manifest)
                self._set_upload(state="idle", last_ok=ep_id, last_ok_t=time.time(), episode="")
                backoff = 0.1 if len(todo) > 1 else self.upload_interval
            except Exception as e:
                self._set_upload(state="waiting", error=str(e)[:300])
                log("upload: %s" % e)
                backoff = min(60.0, max(self.upload_interval, backoff * 2))

    def _set_upload(self, **kw):
        with self.lock:
            self.upload.update(kw)

    @staticmethod
    def _conn(url, timeout=30):
        u = urllib.parse.urlsplit(url)
        cls = http.client.HTTPSConnection if u.scheme == "https" else http.client.HTTPConnection
        return cls(u.hostname, u.port or (443 if u.scheme == "https" else 80), timeout=timeout), u.path.rstrip("/")

    def _request(self, s, method, path, body=None, headers=None, timeout=30):
        conn, base = self._conn(s["server_url"], timeout)
        h = {"X-Nova-Token": s.get("server_token", "")}
        h.update(headers or {})
        try:
            conn.request(method, base + path, body=body, headers=h)
            r = conn.getresponse()
            data = r.read()
        finally:
            conn.close()
        try:
            js = json.loads(data.decode() or "{}")
        except ValueError:
            js = {}
        if r.status >= 300:
            raise RuntimeError("server answered %d: %s" % (r.status, js.get("error") or data[:200].decode(errors="replace")))
        return js

    def upload_episode(self, s, ep_id, ep_dir, manifest):
        """Raises on any doubt.  Deletes the local episode only after a matching receipt."""
        base = "/api/episodes/" + ep_id
        have = {f["path"]: f["sha256"] for f in self._request(s, "GET", base).get("files", [])}
        sent = 0
        for f in manifest["files"]:
            full = os.path.join(ep_dir, *f["path"].split("/"))
            if os.path.getsize(full) != f["size"]:
                raise RuntimeError("%s changed size after the episode was closed" % f["path"])
            if have.get(f["path"]) != f["sha256"]:
                with open(full, "rb") as fh:
                    self._request(s, "PUT", base + "/files/" + urllib.parse.quote(f["path"]), body=fh,
                                  headers={"Content-Length": str(f["size"]), "X-Sha256": f["sha256"],
                                           "Content-Type": "application/octet-stream"}, timeout=120)
            sent += f["size"]
            self._set_upload(sent_bytes=sent)
        receipt = self._request(s, "POST", base + "/commit", body=json.dumps(manifest).encode(),
                                headers={"Content-Type": "application/json"}, timeout=600)
        if not receipt_matches(manifest, receipt):
            raise RuntimeError("the server's receipt for %s does not match what was sent; nothing deleted" % ep_id)
        if s["keep_local_copy"]:
            with open(os.path.join(ep_dir, "receipt.json"), "w") as fh:
                json.dump(receipt, fh, indent=2)
            log("episode %s: stored and checked on the server; local copy kept" % ep_id)
        else:
            shutil.rmtree(ep_dir)
            log("episode %s: stored and checked on the server; local copy deleted" % ep_id)

    def retry_now(self):
        self._wake.set()

    # ---- status for the page ----------------------------------------------------------------
    def status(self):
        s = self.get_settings()
        spool = self.spool()
        with self.lock:
            rec = self.rec
            out = {"enabled": bool(s["enabled"]), "recording": rec is not None,
                   "last_episode": self.last_episode, "upload": dict(self.upload)}
            if rec:
                out["episode"] = {"id": rec["id"], "seconds": round(time.time() - rec["t0"], 1),
                                  "bytes": dir_bytes(rec["dir"])}
        pend = self.pending()
        out["pending"] = {"count": len(pend), "bytes": sum(m["total_bytes"] for _, _, m in pend)}
        out["free_gb"] = round(free_bytes(spool) / GB, 1)
        out["spool_gb"] = round(dir_bytes(spool) / GB, 2) if os.path.isdir(spool) else 0.0
        out["topics"] = [{"name": t["name"], "group": t["group"]} for t in self.selected_topics()]
        return out
