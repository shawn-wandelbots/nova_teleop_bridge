#!/usr/bin/env python3
"""Nova Teleop control page, hosted on the robot.

    python3 robot/teleop_service.py

The page is NOT reachable all the time.  It opens when the headset app connects (the app
sends a small "hello" with a random four-letter word, see robot/session.py) and it closes
when the app says goodbye or after 15 minutes without anyone using the page.  The browser
has to type the word shown in the headset before it gets anything but the lock screen.

While it runs, the service also keeps the teleop bridge in test mode, so the headset app can
find the robot and its stream can be checked.  Test mode sends nothing to the robot.

What the page does:
  * shows the robot's state, its three boards, the robot's teleop input and the headset link
  * switches the robot into and out of its teleoperation state (the same two calls the
    vendor phone app makes)
  * starts and stops the teleop bridge (robot/teleop_bridge.py), in test mode or live
  * keeps the settings, including the ones for data collection that is not built yet

Nothing here moves the robot by itself: every action is a button press on the page.
Stdlib only.  The four-letter word keeps casual visitors out; it is not strong security,
and traffic is not encrypted: run it only on a network you trust.
"""
from __future__ import annotations

import argparse
import hmac
import json
import os
import secrets
import select
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
sys.path.insert(0, ROOT)
from robot import session as sess  # noqa: E402

WEB_DIR = os.path.join(ROOT, "web")
BRIDGE = os.path.join(ROOT, "robot", "teleop_bridge.py")
TELEOP_STATE = "RemoteMotionStream"
COOKIE = "nova_teleop"

BOARDS = (("head", "Head board", "10.42.10.10"), ("main", "Main controller", "10.42.10.12"),
          ("compute", "Compute board", "10.42.10.11"))

DEFAULT_SETTINGS = {
    "bridge": {"sender_ip": "", "port": 39500},
    # What the robot does when the operator closes the connection in the headset.
    "session": {"leave_teleop_on_disconnect": True},
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
        self.closing = False
        self.bridge = None            # subprocess.Popen
        self.bridge_mode = None
        self.transition = None        # {"action", "since", "error"}
        self.last_error = ""
        self.cache = {"robot": {"reachable": False}, "boards": {}, "input": {}}
        # ---- page access (see Gate) ----
        self.word = None              # four letters from the headset; None = page closed
        self.token = None             # cookie value of the unlocked browser(s)
        self.last_interaction = 0.0
        self.bad_tries = 0
        self.locked_until = 0.0
        threading.Thread(target=self._poll_loop, daemon=True).start()

    # ---- page access ------------------------------------------------------------------------
    def open_page(self, word):
        with self.lock:
            self.word, self.token = word, secrets.token_urlsafe(24)
            self.last_interaction = time.time()
            self.bad_tries, self.locked_until = 0, 0.0

    def close_page(self):
        with self.lock:
            self.word = self.token = None

    def touch(self):
        self.last_interaction = time.time()

    def stream_active(self):
        """True while valid headset frames are arriving (the operator is teleoperating)."""
        try:
            with open(self.status_file) as fh:
                snap = json.load(fh)
            return time.time() - snap.get("t", 0) < 3.0 and snap.get("valid_hz", 0) > 0
        except (OSError, ValueError):
            return False

    def headset_left(self):
        """The operator closed the connection in the headset: stop feeding the robot and,
        if set so, take the robot out of teleoperation mode."""
        if self.bridge_running() and self.bridge_mode == "live":
            self.stop_bridge()
        with self.lock:
            leave = self.settings["session"]["leave_teleop_on_disconnect"]
            in_teleop = self.cache["robot"].get("state") == TELEOP_STATE
        if leave and in_teleop:
            try:
                self.teleop("stop")
            except RuntimeError as e:
                log("could not leave teleoperation mode: %s" % e)

    def idle_seconds(self):
        return time.time() - self.last_interaction

    def unlock(self, word):
        """Returns the cookie token, or raises PermissionError with a message for the page."""
        now = time.time()
        with self.lock:
            if self.word is None:
                raise PermissionError("the page is closed")
            if now < self.locked_until:
                raise PermissionError("too many wrong tries: wait %d s" % (self.locked_until - now + 1))
            ok = hmac.compare_digest(str(word).strip().upper().encode(), self.word.encode())
            if not ok:
                self.bad_tries += 1
                if self.bad_tries >= self.a.max_tries:
                    self.bad_tries, self.locked_until = 0, now + self.a.lockout_s
                raise PermissionError("that is not the word shown in the headset")
            self.bad_tries = 0
            self.last_interaction = now
            return self.token

    def authorised(self, cookie_header):
        token = self.token
        if not token or not cookie_header:
            return False
        for part in cookie_header.split(";"):
            k, _, v = part.strip().partition("=")
            if k == COOKIE and hmac.compare_digest(v.encode(), token.encode()):
                return True
        return False

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
        if self.bridge_running():
            if self.bridge_mode == mode:
                raise RuntimeError("the headset link is already in %s mode" % mode)
            self._kill_bridge()
        with self.lock:
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
        """Leave live mode. With standby on, the bridge goes back to test mode (sends nothing)."""
        self._kill_bridge()
        if self.a.standby and not self.closing:
            try:
                self.start_bridge("test")
            except (RuntimeError, OSError) as e:
                log("could not restart the bridge in test mode: %s" % e)

    def shutdown(self):
        self.closing = True
        self._kill_bridge()

    def _kill_bridge(self):
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
        snap["closes_in"] = max(0, int(self.a.idle_close_s - self.idle_seconds()))
        snap["time"] = time.time()
        return snap


class Handler(BaseHTTPRequestHandler):
    server_version = "NovaTeleop/1"
    svc: Service = None

    def log_message(self, fmt, *args):      # quiet: the poller would fill the log
        pass

    def _send(self, code, body, ctype="application/json", cookie=None):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        if cookie:
            self.send_header("Set-Cookie", "%s=%s; Path=/; HttpOnly; SameSite=Strict" % (COOKIE, cookie))
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            if not self.svc.authorised(self.headers.get("Cookie")):
                return self._send(401, {"error": "locked"})
            return self._send(200, self.svc.status())
        if path == "/gestures.svg":
            try:
                with open(os.path.join(WEB_DIR, "gestures.svg"), "rb") as fh:
                    return self._send(200, fh.read(), "image/svg+xml")
            except OSError:
                return self._send(404, {"error": "not found"})
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
        if path == "/api/unlock":
            try:
                return self._send(200, {"ok": True}, cookie=self.svc.unlock(body.get("word", "")))
            except PermissionError as e:
                return self._send(403, {"error": str(e)})
        if not self.svc.authorised(self.headers.get("Cookie")):
            return self._send(401, {"error": "locked"})
        self.svc.touch()                    # any button press or page activity counts as use
        try:
            if path == "/api/ping":
                pass
            elif path == "/api/teleop/start":
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
    ap.add_argument("--session-port", type=int, default=sess.DEFAULT_PORT,
                    help="UDP port for the headset's hello / goodbye (default %(default)s)")
    ap.add_argument("--idle-close-s", type=float, default=900.0, help="close the page after this long without use")
    ap.add_argument("--max-tries", type=int, default=5, help="wrong words before a pause")
    ap.add_argument("--lockout-s", type=float, default=30.0)
    ap.add_argument("--no-standby", dest="standby", action="store_false",
                    help="do not keep the bridge running in test mode")
    return ap


class Gate:
    """Opens the web page when a headset says hello and closes it again.

    hello (new session)  -> page opens with that word; browsers unlocked before are locked out
    goodbye              -> page closes; live link stops; robot leaves teleoperation mode (setting)
    idle_close_s unused  -> page closes.  Teleoperating counts as use: while valid headset
                            frames arrive the page never idles out.  A page closed for idleness
                            comes back, with the same word, as soon as that headset streams again.
    While a headset keeps saying hello, hellos from other addresses are ignored.
    """
    HELLO_FRESH_S = 5.0

    def __init__(self, svc, args):
        self.svc, self.a = svc, args
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind((args.listen_ip, args.session_port))
        self.udp.setblocking(False)
        self.session_port = self.udp.getsockname()[1]
        self.httpd = None
        self.web_port = None
        self.peer = None              # (ip, session id)
        self.ended = None             # the session whose page was closed: its late hellos do not reopen it
        self.resumable = False        # ended by idleness (not by goodbye): may come back
        self.ended_hello = 0.0
        self.last_hello = 0.0
        self._stop = False

    def page_open(self):
        return self.httpd is not None

    def _open(self, word, why):
        if self.httpd is None:
            self.httpd = ThreadingHTTPServer((self.a.listen_ip, self.a.port), Handler)
            self.httpd.daemon_threads = True
            self.web_port = self.httpd.server_address[1]
            threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.svc.open_page(word)
        log("page OPEN on port %d (%s)" % (self.web_port, why))

    def _close(self, why, resumable=False):
        if not resumable:
            self.svc.close_page()         # forget the word: a new connection is needed
        self.resumable = resumable
        self.ended, self.peer = self.peer or self.ended, None
        if self.httpd is not None:
            h, self.httpd = self.httpd, None
            h.shutdown()
            h.server_close()
            log("page CLOSED (%s)" % why)

    def _on_datagram(self, data, ip, now):
        try:
            kind, sid, word = sess.unpack(data)
        except ValueError:
            return
        only = self.svc.settings["bridge"]["sender_ip"]
        if only and ip != only:
            return
        if kind == sess.HELLO:
            if self.peer == (ip, sid):
                self.last_hello = now
                return
            if self.ended == (ip, sid):
                self.ended_hello = now              # closed for this connection; see step() for the way back
                return
            if self.peer and self.peer[0] != ip and now - self.last_hello < self.HELLO_FRESH_S:
                return                              # another headset is connected right now
            self.peer, self.last_hello = (ip, sid), now
            self.resumable = False
            self._open(word, "headset %s connected" % ip)
        elif kind == sess.BYE and self.peer == (ip, sid):
            self._close("headset closed the connection")
            self.svc.headset_left()
        elif kind == sess.BYE and (self.peer is None or self.ended == (ip, sid)):
            if self.resumable and self.ended == (ip, sid):
                self.svc.close_page()
                self.svc.headset_left()
            self.ended, self.resumable = (ip, sid), False

    def step(self, timeout=0.5):
        try:
            readable, _, _ = select.select([self.udp], [], [], timeout)
        except InterruptedError:
            return
        now = time.monotonic()
        if readable:
            while True:
                try:
                    data, addr = self.udp.recvfrom(256)
                except (BlockingIOError, InterruptedError):
                    break
                self._on_datagram(data, addr[0], now)
        active = self.svc.stream_active()
        if self.httpd is not None:
            if active:
                self.svc.touch()                    # teleoperating counts as using the page
            elif self.svc.idle_seconds() > self.a.idle_close_s:
                self._close("not used for %d min" % (self.a.idle_close_s // 60), resumable=True)
        elif self.resumable and active and self.ended and now - self.ended_hello < self.HELLO_FRESH_S:
            # the same headset is streaming again: give the page back, same word, same browsers
            self.peer, self.ended, self.resumable, self.last_hello = self.ended, None, False, now
            self.httpd = ThreadingHTTPServer((self.a.listen_ip, self.a.port), Handler)
            self.httpd.daemon_threads = True
            self.web_port = self.httpd.server_address[1]
            threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
            self.svc.touch()
            log("page OPEN again on port %d (headset is streaming again)" % self.web_port)

    def run(self):
        while not self._stop:
            self.step()
        self._close("service stopping")
        self.udp.close()

    def stop(self):
        self._stop = True


def main(argv=None):
    a = build_parser().parse_args(argv)
    if a.session_port in (25618, 25619, 15000, 51000):
        raise SystemExit("that session port belongs to the robot's own teleop stack")
    svc = Service(a)
    Handler.svc = svc
    gate = Gate(svc, a)
    signal.signal(signal.SIGINT, lambda *_: gate.stop())
    signal.signal(signal.SIGTERM, lambda *_: gate.stop())
    log("waiting for a headset on UDP %d; the page (port %d) opens when one connects" % (gate.session_port, a.port))
    if a.standby:
        try:
            svc.start_bridge("test")
        except (RuntimeError, OSError) as e:
            log("could not start the bridge in test mode: %s" % e)
    try:
        gate.run()
    finally:
        svc.shutdown()          # never leave a bridge feeding the robot without its service
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
