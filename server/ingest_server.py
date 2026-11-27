#!/usr/bin/env python3
"""Receives recorded episodes from the robot and proves that they are stored intact.

    python3 server/ingest_server.py --root ~/nova_teleop_data --token <secret>

The robot deletes its copy of an episode only after this server has answered "commit" with a
receipt, and the server writes a receipt only after reading every file back from its own disk
and comparing size and SHA-256 with the robot's list.

    GET  /api/health                              free space, number of episodes
    GET  /api/episodes/<id>                       files already stored (lets an upload resume)
    PUT  /api/episodes/<id>/files/<path>          one file; header X-Sha256 must match the body
    POST /api/episodes/<id>/commit                body = manifest; answers with the receipt
    POST /api/episodes/<id>/discard               the operator threw the episode away: it is
                                                  moved to <root>/discarded/, not erased

Every request needs the header ``X-Nova-Token``.  Stdlib only.  Traffic is not encrypted:
use it on a network you trust.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
import re
import shutil
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
PART_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9._-]{0,127}$")
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
GB = 1024 ** 3


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            b = fh.read(chunk)
            if not b:
                return h.hexdigest()
            h.update(b)


def safe_rel(rel):
    """'bag/x.mcap' -> ['bag', 'x.mcap'], or None if it could leave the episode directory."""
    parts = rel.split("/")
    if not 1 <= len(parts) <= 4 or any(not PART_RE.match(p) or p.endswith(".part") for p in parts):
        return None
    if parts[-1] == "receipt.json":
        return None
    return parts


def verify_episode(ep_dir, manifest):
    """Reads every file from disk.  Returns (receipt files, None) or (None, reason)."""
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        return None, "manifest lists no files"
    out, seen = [], set()
    for f in files:
        try:
            rel, size, sha = f["path"], int(f["size"]), f["sha256"]
        except (KeyError, TypeError, ValueError):
            return None, "manifest entry is malformed"
        parts = safe_rel(rel) if isinstance(rel, str) else None
        if parts is None or rel in seen or not isinstance(sha, str) or not SHA_RE.match(sha):
            return None, "manifest entry %r is not acceptable" % (rel,)
        seen.add(rel)
        full = os.path.join(ep_dir, *parts)
        if not os.path.isfile(full):
            return None, "%s is missing on the server" % rel
        if os.path.getsize(full) != size:
            return None, "%s has the wrong size on the server" % rel
        if sha256_file(full) != sha:
            return None, "%s does not match its checksum on the server" % rel
        out.append({"path": rel, "size": size, "sha256": sha})
    return out, None


class Handler(BaseHTTPRequestHandler):
    server_version = "NovaIngest/1"
    root = ""
    token = ""
    min_free = 0

    def log_message(self, fmt, *args):
        print("[ingest %s] %s" % (time.strftime("%H:%M:%S"), fmt % args), flush=True)

    def _send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _auth(self):
        got = self.headers.get("X-Nova-Token") or ""
        if not self.token or not hmac.compare_digest(got.encode(), self.token.encode()):
            self.close_connection = True          # the body, if any, is not read
            self._send(401, {"error": "wrong or missing token"})
            return False
        return True

    def _episode(self, path):
        """'/api/episodes/<id>[/rest]' -> (id, rest) or (None, None)."""
        if not path.startswith("/api/episodes/"):
            return None, None
        ep, _, rest = path[len("/api/episodes/"):].partition("/")
        return (ep, rest) if ID_RE.match(ep) else (None, None)

    def do_GET(self):
        if not self._auth():
            return
        path = self.path.split("?", 1)[0]
        eps = os.path.join(self.root, "episodes")
        if path == "/api/health":
            n = len([d for d in os.listdir(eps) if os.path.exists(os.path.join(eps, d, "receipt.json"))])
            return self._send(200, {"ok": True, "free_gb": round(shutil.disk_usage(self.root).free / GB, 1),
                                    "episodes_stored": n})
        ep, rest = self._episode(path)
        if ep is None or rest:
            return self._send(404, {"error": "not found"})
        ep_dir = os.path.join(eps, ep)
        files = []
        for root, _, names in os.walk(ep_dir):
            for n in names:
                if n.endswith(".part") or n == "receipt.json":
                    continue
                full = os.path.join(root, n)
                files.append({"path": os.path.relpath(full, ep_dir).replace(os.sep, "/"),
                              "size": os.path.getsize(full), "sha256": sha256_file(full)})
        self._send(200, {"episode": ep, "files": files,
                         "committed": os.path.exists(os.path.join(ep_dir, "receipt.json"))})

    def do_PUT(self):
        if not self._auth():
            return
        self.close_connection = True              # simplest safe handling of refused bodies
        ep, rest = self._episode(self.path.split("?", 1)[0])
        parts = safe_rel(rest[len("files/"):]) if ep and rest.startswith("files/") else None
        want = (self.headers.get("X-Sha256") or "").lower()
        try:
            length = int(self.headers.get("Content-Length"))
        except (TypeError, ValueError):
            return self._send(411, {"error": "Content-Length is required"})
        if parts is None or not SHA_RE.match(want) or length < 0:
            return self._send(400, {"error": "bad path or checksum header"})
        if shutil.disk_usage(self.root).free - length < self.min_free:
            return self._send(507, {"error": "the server is short of disk space"})
        ep_dir = os.path.join(self.root, "episodes", ep)
        if os.path.exists(os.path.join(ep_dir, "receipt.json")):
            return self._send(409, {"error": "this episode is already stored and closed"})
        full = os.path.join(ep_dir, *parts)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        tmp = full + ".part"
        h, left = hashlib.sha256(), length
        try:
            with open(tmp, "wb") as fh:
                while left:
                    b = self.rfile.read(min(left, 1 << 20))
                    if not b:
                        break
                    fh.write(b)
                    h.update(b)
                    left -= len(b)
                fh.flush()
                os.fsync(fh.fileno())
            if left or h.hexdigest() != want:
                os.remove(tmp)
                return self._send(422, {"error": "the file arrived incomplete or damaged; not stored"})
            os.replace(tmp, full)
        except OSError as e:
            try:
                os.remove(tmp)
            except OSError:
                pass
            return self._send(500, {"error": "could not store the file: %s" % e})
        self._send(201, {"ok": True, "path": "/".join(parts), "size": length, "sha256": want})

    def do_POST(self):
        if not self._auth():
            return
        ep, rest = self._episode(self.path.split("?", 1)[0])
        if ep is not None and rest == "discard":
            self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 65536))
            src = os.path.join(self.root, "episodes", ep)
            if not os.path.isdir(src):
                return self._send(404, {"error": "the server has no episode %s" % ep})
            os.makedirs(os.path.join(self.root, "discarded"), exist_ok=True)
            dst, n = os.path.join(self.root, "discarded", ep), 1
            while os.path.exists(dst):
                n += 1
                dst = os.path.join(self.root, "discarded", "%s_%d" % (ep, n))
            try:
                os.replace(src, dst)
            except OSError as e:
                return self._send(500, {"error": "could not move the episode: %s" % e})
            return self._send(200, {"ok": True, "episode": ep, "discarded": True})
        if ep is None or rest != "commit":
            self.close_connection = True
            return self._send(404, {"error": "not found"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if not 0 < n <= 8 << 20:
                raise ValueError("manifest size")
            manifest = json.loads(self.rfile.read(n).decode())
            if not isinstance(manifest, dict) or manifest.get("episode") != ep:
                raise ValueError("manifest is for another episode")
        except (ValueError, UnicodeDecodeError) as e:
            self.close_connection = True
            return self._send(400, {"error": "bad manifest: %s" % e})
        ep_dir = os.path.join(self.root, "episodes", ep)
        files, why = verify_episode(ep_dir, manifest)
        if files is None:
            return self._send(409, {"error": why, "verified": False})
        receipt = {"verified": True, "episode": ep, "files": files,
                   "total_bytes": sum(f["size"] for f in files),
                   "verified_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        try:
            with open(os.path.join(ep_dir, "manifest.json"), "w") as fh:
                json.dump(manifest, fh, indent=2)
            tmp = os.path.join(ep_dir, "receipt.json.part")
            with open(tmp, "w") as fh:
                json.dump(receipt, fh, indent=2)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, os.path.join(ep_dir, "receipt.json"))
            dfd = os.open(ep_dir, os.O_RDONLY)
            try:
                os.fsync(dfd)
            finally:
                os.close(dfd)
        except OSError as e:
            return self._send(500, {"error": "could not write the receipt: %s" % e, "verified": False})
        self._send(200, receipt)


def make_server(root, token, listen_ip="0.0.0.0", port=8095, min_free_gb=5.0):
    root = os.path.abspath(os.path.expanduser(root))
    os.makedirs(os.path.join(root, "episodes"), exist_ok=True)
    h = type("BoundHandler", (Handler,), {"root": root, "token": token, "min_free": int(min_free_gb * GB)})
    return ThreadingHTTPServer((listen_ip, port), h)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="~/nova_teleop_data", help="episodes are stored in <root>/episodes/")
    ap.add_argument("--token", default=os.environ.get("NOVA_INGEST_TOKEN", ""),
                    help="shared secret (or set NOVA_INGEST_TOKEN)")
    ap.add_argument("--listen-ip", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8095)
    ap.add_argument("--min-free-gb", type=float, default=5.0, help="refuse uploads below this much free space")
    a = ap.parse_args(argv)
    if len(a.token) < 8:
        sys.exit("a token of at least 8 characters is required (--token or NOVA_INGEST_TOKEN)")
    srv = make_server(a.root, a.token, a.listen_ip, a.port, a.min_free_gb)
    print("[ingest] storing episodes in %s, listening on %s:%d"
          % (os.path.abspath(os.path.expanduser(a.root)), a.listen_ip, a.port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
