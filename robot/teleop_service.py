#!/usr/bin/env python3
"""Nova Teleop control page, hosted on the robot.

    python3 robot/teleop_service.py                 # then open http://<robot>:8090/

What it does:
  * shows the robot's state, its three boards, the robot's teleop input and the headset link
  * switches the robot into and out of its teleoperation state (the same two calls the
    vendor phone app makes)
  * starts and stops the teleop bridge (robot/teleop_bridge.py), in test mode or live
  * keeps the settings, including the ones for data collection that is not built yet

Nothing here moves the robot by itself: every action is a button press on the page.
Stdlib only.  There is no login: run it only on a network you trust.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(ROOT, "web")
BRIDGE = os.path.join(ROOT, "robot", "teleop_bridge.py")
TELEOP_STATE = "RemoteMotionStream"

BOARDS = (("head", "Head board", "10.42.10.10"), ("main", "Main controller", "10.42.10.12"),
          ("compute", "Compute board", "10.42.10.11"))

DEFAULT_SETTINGS = {
    "bridge": {"sender_ip": "", "port": 39500},
    "data_collection": {
        "enabled": False,
        "task": "",
        "operator": "",
        "server_url": "",
        "upload_format": "lerobot",          # lerobot | raw
        "keep_local_copy": False,
        "spool_dir": "~/nova_teleop_data/spool",
        "max_spool_gb": 20,
        "cameras": {"head": True, "wrist_left": False, "wrist_right": False},
    },
}
UPLOAD_FORMATS = ("lerobot", "raw")


def log(msg):
    print("[service %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def _same_kind(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool)
    if isinstance(a, (int, float)):
        return isinstance(b, (int, float))
    return isinstance(a, str) and isinstance(b, str)


def merge_settings(base, new):
    """Copy of ``base``, taking values from ``new`` only for known keys of the same kind."""
    out = {}
    for k, v in base.items():
        n = new.get(k) if isinstance(new, dict) else None
        if isinstance(v, dict):
            out[k] = merge_settings(v, n)
        else:
            out[k] = n if n is not None and _same_kind(v, n) else v
    return out


def validate_settings(s):
    d = s["data_collection"]
    if d["upload_format"] not in UPLOAD_FORMATS:
        raise ValueError("upload format must be one of %s" % (UPLOAD_FORMATS,))
    if d["server_url"] and not d["server_url"].startswith(("http://", "https://")):
        raise ValueError("server address must start with http:// or https://")
    if not 1 <= d["max_spool_gb"] <= 2000:
        raise ValueError("temporary storage limit must be 1..2000 GB")
    port = s["bridge"]["port"]
    if not 1024 <= port <= 65535 or port in (25618, 25619, 15000, 51000):
        raise ValueError("bridge port must be 1024..65535 and not one of the robot's own teleop ports")
    ip = s["bridge"]["sender_ip"]
    if ip:
        try:
            socket.inet_aton(ip)
        except OSError:
            raise ValueError("headset address is not a valid IPv4 address")
    # Recording is not built yet: the switch stays off however the page is used.
    d["enabled"] = False
    return s


class Service:
    def __init__(self, args):
        self.a = args
        self.lock = threading.Lock()
        self.settings_path = os.path.expanduser(args.settings)
        self.settings = self._load_settings()
        self.status_file = os.path.join(args.run_dir, "bridge_status.json")
        self.bridge_log = os.path.join(args.run_dir, "bridge.log")
        os.makedirs(args.run_dir, exist_ok=True)
        self.bridge = None            # subprocess.Popen
        self.bridge_mode = None
        self.transition = None        # {"action", "since", "error"}
        self.last_error = ""
        self.cache = {"robot": {"reachable": False}, "boards": {}, "input": {}}
        threading.Thread(target=self._poll_loop, daemon=True).start()

    # ---- settings -----------------------------------------------------------------------
    def _load_settings(self):
        try:
            with open(self.settings_path) as fh:
                return validate_settings(merge_settings(DEFAULT_SETTINGS, json.load(fh)))
        except (OSError, ValueError):
            return merge_settings(DEFAULT_SETTINGS, {})

    def save_settings(self, new):
        s = validate_settings(merge_settings(self.settings, new))
        os.makedirs(os.path.dirname(self.settings_path), exist_ok=True)
        tmp = self.settings_path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(s, fh, indent=2)
        os.replace(tmp, self.settings_path)
        with self.lock:
            self.settings = s
        return s

    # ---- robot calls ----------------------------------------------------------------------
    def _rpc(self, base, method, timeout):
        req = urllib.request.Request(base.rstrip("/") + "/rpc/" + method, data=b"{}",
                                     headers={"content-type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode() or "{}")

    def _poll_loop(self):
        while True:
            robot = {"reachable": False, "state": "", "status": ""}
            try:
                r = self._rpc(self.a.sm_url, "aimdk.protocol.SystemService/GetSystemState", 2.0)
                robot = {"reachable": True, "state": r.get("cur_state", ""),
                         "status": r.get("cur_status", "").replace("SystemStatus_", "")}
            except (OSError, ValueError):
                pass
            boards = {}
            for key, _, ip in BOARDS:
                boards[key] = self._tcp_open(ip, 22)
            inp = {"running": self._process_running("hal_remote"),
                   "controller_running": self._process_running("teleop_avatar")}
            with self.lock:
                self.cache = {"robot": robot, "boards": boards, "input": inp}
            time.sleep(self.a.poll_s)

    @staticmethod
    def _tcp_open(ip, port):
        try:
            with socket.create_connection((ip, port), timeout=0.6):
                return True
        except OSError:
            return False

    @staticmethod
    def _process_running(name):
        try:
            return subprocess.run(["pgrep", "-f", "/opt/agibot/bin/" + name], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, timeout=2).returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    def teleop(self, action):
        """action: 'start' or 'stop'. The robot takes about 25 s to answer; runs in the background."""
        method = {"start": "StartRemoteMotionStream", "stop": "StopRemoteMotionStream"}[action]
        with self.lock:
            if self.transition and not self.transition.get("done"):
                raise RuntimeError("the robot is already changing state")
            self.transition = {"action": action, "since": time.time(), "done": False, "error": ""}

        def run():
            err = ""
            try:
                r = self._rpc(self.a.skillpilot_url, "aimdk.protocol.SkillPilotService/" + method, 60.0)
                h = r.get("header", {}) if isinstance(r, dict) else {}
                code = str(h.get("code", r.get("code", "0")))
                if code not in ("0", ""):
                    err = h.get("msg") or r.get("msg") or ("robot answered with code " + code)
            except urllib.error.HTTPError as e:
                err = "robot refused: %s" % (e.read().decode(errors="replace")[:300] or e.reason)
            except (OSError, ValueError) as e:
                err = "no answer from the robot: %s" % e
            with self.lock:
                self.transition.update(done=True, error=err, finished=time.time())
            log("teleop %s -> %s" % (action, err or "ok"))

        log("teleop %s requested" % action)
        threading.Thread(target=run, daemon=True).start()

    # ---- bridge process ---------------------------------------------------------------------
    def bridge_running(self):
        return self.bridge is not None and self.bridge.poll() is None

    def start_bridge(self, mode):
        if mode not in ("test", "live"):
            raise ValueError("mode must be 'test' or 'live'")
        with self.lock:
            if self.bridge_running():
                raise RuntimeError("the headset link is already running; stop it first")
            b = self.settings["bridge"]
            cmd = [sys.executable, BRIDGE, "--port", str(b["port"]), "--status-file", self.status_file]
            if b["sender_ip"]:
                cmd += ["--sender-ip", b["sender_ip"]]
            if mode == "live":
                cmd += ["--robot-input", "--robot-input-host", self.a.robot_input_host,
                        "--robot-input-port", str(self.a.robot_input_port)]
            cmd += self.a.bridge_arg
            logfh = open(self.bridge_log, "ab")
            self.bridge = subprocess.Popen(cmd, stdout=logfh, stderr=subprocess.STDOUT, cwd=ROOT)
            logfh.close()
            self.bridge_mode = mode
        log("bridge started (%s), pid %d" % (mode, self.bridge.pid))

    def stop_bridge(self):
        with self.lock:
            p = self.bridge
        if p is None or p.poll() is not None:
            return
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
        log("bridge stopped")

    def _bridge_status(self):
        running = self.bridge_running()
        out = {"running": running, "mode": self.bridge_mode if running else None, "managed": True}
        try:
            with open(self.status_file) as fh:
                snap = json.load(fh)
            if time.time() - snap.get("t", 0) < 5:
                out.update(link=snap)
                if not running:       # a bridge started by hand with the same status file
                    out.update(running=True, managed=False,
                               mode="live" if snap.get("mode") != "dry-run" else "test")
        except (OSError, ValueError):
            pass
        try:
            with open(self.bridge_log, "rb") as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - 4000))
                out["log"] = fh.read().decode(errors="replace").splitlines()[-12:]
        except OSError:
            out["log"] = []
        return out

    def status(self):
        with self.lock:
            snap = dict(self.cache)
            tr = dict(self.transition) if self.transition else None
            settings = self.settings
        if tr and tr.get("done") and time.time() - tr.get("finished", 0) > 30 and not tr.get("error"):
            tr = None
        snap["boards"] = [{"key": k, "name": n, "up": bool(snap["boards"].get(k))} for k, n, _ in BOARDS]
        snap["robot"]["teleop"] = snap["robot"].get("state") == TELEOP_STATE
        snap["transition"] = tr
        snap["bridge"] = self._bridge_status()
        snap["settings"] = settings
        snap["time"] = time.time()
        return snap


class Handler(BaseHTTPRequestHandler):
    server_version = "NovaTeleop/1"
    svc: Service = None

    def log_message(self, fmt, *args):      # quiet: the poller would fill the log
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            return self._send(200, self.svc.status())
        if path in ("/", "/index.html"):
            try:
                with open(os.path.join(WEB_DIR, "index.html"), "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            except OSError:
                return self._send(500, {"error": "web/index.html is missing"})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        # A plain cross-site form cannot set this header, so other web pages cannot press buttons here.
        if self.headers.get("X-Nova-Teleop") != "1":
            return self._send(403, {"error": "missing X-Nova-Teleop header"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > 65536:
                return self._send(413, {"error": "too large"})
            body = json.loads(self.rfile.read(n).decode() or "{}") if n else {}
            if not isinstance(body, dict):
                raise ValueError("body must be a JSON object")
        except ValueError as e:
            return self._send(400, {"error": str(e)})
        path = self.path.split("?", 1)[0]
        try:
            if path == "/api/teleop/start":
                self.svc.teleop("start")
            elif path == "/api/teleop/stop":
                self.svc.stop_bridge()          # stop feeding the robot before it leaves teleop
                self.svc.teleop("stop")
            elif path == "/api/bridge/start":
                self.svc.start_bridge(body.get("mode", "test"))
            elif path == "/api/bridge/stop":
                self.svc.stop_bridge()
            elif path == "/api/settings":
                return self._send(200, {"ok": True, "settings": self.svc.save_settings(body)})
            else:
                return self._send(404, {"error": "not found"})
        except (ValueError, RuntimeError) as e:
            return self._send(409, {"error": str(e)})
        except OSError as e:
            return self._send(500, {"error": str(e)})
        self._send(200, {"ok": True})


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen-ip", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--settings", default="~/.config/nova_teleop/settings.json")
    ap.add_argument("--run-dir", default=os.path.expanduser("~/.local/state/nova_teleop"),
                    help="bridge log and status snapshot live here")
    ap.add_argument("--sm-url", default="http://10.42.10.12:51011", help="robot state service")
    ap.add_argument("--skillpilot-url", default="http://10.42.10.12:52893", help="service that switches teleop on/off")
    ap.add_argument("--robot-input-host", default="127.0.0.1")
    ap.add_argument("--robot-input-port", type=int, default=25618)
    ap.add_argument("--poll-s", type=float, default=2.0)
    ap.add_argument("--bridge-arg", action="append", default=[], help="extra argument for the bridge (repeatable)")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    svc = Service(a)
    Handler.svc = svc
    httpd = ThreadingHTTPServer((a.listen_ip, a.port), Handler)
    httpd.daemon_threads = True

    def stop(*_):
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    log("control page on http://%s:%d/" % (a.listen_ip, a.port))
    try:
        httpd.serve_forever()
    finally:
        svc.stop_bridge()       # never leave a bridge feeding the robot without its control page
        httpd.server_close()
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
