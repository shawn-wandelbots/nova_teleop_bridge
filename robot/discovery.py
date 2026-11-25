"""NTD1 discovery responder: lets the headset app find this bridge on the local network.

Protocol (docs/DISCOVERY.md): a 16-byte probe is answered with one small datagram that echoes
the probe's nonce and carries a JSON description of the bridge.  The responder is read-only:

  * only a well-formed probe is answered (wrong length / magic / version / type / reserved bits
    -> silent drop); nothing else is ever sent from the discovery socket
  * exactly one reply per probe, always shorter than MAX_REPLY_SIZE (512) bytes
  * replies are rate-limited per source address and globally
  * probes from privileged source ports or from the robot's own teleop ports are dropped, so a
    forged source can never make the bridge send to those ports
  * a probe never changes bridge state; the status is obtained through a read-only callback

Standard library only.  No ROS, no robot services.
"""
from __future__ import annotations

import json
import re
import socket
import struct
from collections import Counter, OrderedDict
from typing import Callable, Dict, Optional, Tuple

MAGIC = b"NTD1"
VERSION = 1
TYPE_PROBE = 1
TYPE_REPLY = 2
NONCE_SIZE = 8
HEADER = struct.Struct("<4sBBH8s")      # magic, version, type, u16 (probe: reserved 0; reply: JSON length), nonce
PROBE_SIZE = HEADER.size                # 16
MAX_REPLY_SIZE = 512                    # a reply is always strictly smaller than this
MAX_JSON_SIZE = MAX_REPLY_SIZE - HEADER.size - 16   # 480
DEFAULT_DISCOVERY_PORT = 39501          # NTB1 data port + 1
PRODUCT = "nova_teleop_bridge"
BRIDGE_VERSION = "0.2.0"
MODE_DRY_RUN = "dry-run"
MODE_PUBLISH = "publish"
# The robot's own live teleop ports. Never bound, never a reply destination.
FORBIDDEN_PORTS = (25618, 25619, 15000, 51000)
MIN_SOURCE_PORT = 1024

MODEL_FILE = "/agibot/info/model"
NAME_FILE = "/agibot/info/name"
ROBOT_ID_CHARS = 4                      # only the tail of the name file is ever disclosed


class ProbeError(ValueError):
    """A datagram that is not a valid NTD1 probe / reply. ``reason`` is a short counter name."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def encode_probe(nonce: bytes) -> bytes:
    if len(nonce) != NONCE_SIZE:
        raise ValueError("nonce must be %d bytes" % NONCE_SIZE)
    return HEADER.pack(MAGIC, VERSION, TYPE_PROBE, 0, nonce)


def decode_probe(data: bytes) -> bytes:
    """Returns the nonce of a well-formed probe; raises ProbeError otherwise."""
    if len(data) != PROBE_SIZE:
        raise ProbeError("length")
    magic, version, kind, reserved, nonce = HEADER.unpack(data)
    if magic != MAGIC:
        raise ProbeError("magic")
    if version != VERSION:
        raise ProbeError("version")
    if kind != TYPE_PROBE:
        raise ProbeError("type")
    if reserved != 0:
        raise ProbeError("reserved")
    return nonce


def encode_reply(nonce: bytes, info: Dict[str, object]) -> bytes:
    if len(nonce) != NONCE_SIZE:
        raise ValueError("nonce must be %d bytes" % NONCE_SIZE)
    body = json.dumps(info, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    if len(body) > MAX_JSON_SIZE:
        raise ValueError("discovery reply too large (%d bytes of JSON)" % len(body))
    return HEADER.pack(MAGIC, VERSION, TYPE_REPLY, len(body), nonce) + body


def decode_reply(data: bytes, nonce: Optional[bytes] = None) -> Dict[str, object]:
    """Client side (tests, tools): validates a reply and returns its JSON object."""
    if len(data) < HEADER.size or len(data) >= MAX_REPLY_SIZE:
        raise ProbeError("length")
    magic, version, kind, json_len, got_nonce = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ProbeError("magic")
    if version != VERSION:
        raise ProbeError("version")
    if kind != TYPE_REPLY:
        raise ProbeError("type")
    if json_len != len(data) - HEADER.size or json_len > MAX_JSON_SIZE:
        raise ProbeError("length")
    if nonce is not None and got_nonce != nonce:
        raise ProbeError("nonce")
    try:
        info = json.loads(data[HEADER.size:].decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise ProbeError("json") from None
    if not isinstance(info, dict):
        raise ProbeError("json")
    return info


def _read_first_line(path: str, limit: int = 256) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            return fh.read(limit).strip().splitlines()[0].strip()
    except (OSError, IndexError):
        return ""


def read_robot_identity(model_file: str = MODEL_FILE, name_file: str = NAME_FILE) -> Tuple[str, str]:
    """(model, short id). The id is only the last ROBOT_ID_CHARS characters of the name file,
    never the whole serial-like name. Missing files give empty strings."""
    model = re.sub(r"[^A-Za-z0-9_.\-]", "_", _read_first_line(model_file))[:48]
    name = re.sub(r"[^A-Za-z0-9]", "", _read_first_line(name_file))
    return model, name[-ROBOT_ID_CHARS:]


def build_info(model: str, robot_id: str, data_port: int, mode: str, sender_locked: bool) -> Dict[str, object]:
    if mode not in (MODE_DRY_RUN, MODE_PUBLISH):
        raise ValueError("mode must be %r or %r" % (MODE_DRY_RUN, MODE_PUBLISH))
    return {
        "v": VERSION,
        "product": PRODUCT,
        "bridge_version": BRIDGE_VERSION,
        "model": str(model)[:48],
        "robot_id": str(robot_id)[-ROBOT_ID_CHARS:],
        "data_port": int(data_port),
        "mode": mode,
        "sender_locked": bool(sender_locked),
    }


class RateLimiter:
    """Token buckets: one per source address plus a global one. Bounded memory."""

    def __init__(self, per_source_burst: float = 8.0, per_source_per_s: float = 4.0,
                 global_burst: float = 40.0, global_per_s: float = 20.0, max_sources: int = 256):
        self.per_source_burst, self.per_source_per_s = float(per_source_burst), float(per_source_per_s)
        self.global_burst, self.global_per_s = float(global_burst), float(global_per_s)
        self.max_sources = int(max_sources)
        self._sources: "OrderedDict[str, list]" = OrderedDict()   # ip -> [tokens, t_last]
        self._global = [self.global_burst, None]

    @staticmethod
    def _refill(bucket: list, now: float, burst: float, per_s: float) -> None:
        if bucket[1] is not None and now > bucket[1]:
            bucket[0] = min(burst, bucket[0] + (now - bucket[1]) * per_s)
        bucket[1] = now

    def allow(self, src_ip: str, now: float) -> bool:
        bucket = self._sources.get(src_ip)
        if bucket is None:
            bucket = [self.per_source_burst, now]
            self._sources[src_ip] = bucket
            while len(self._sources) > self.max_sources:
                self._sources.popitem(last=False)
        else:
            self._sources.move_to_end(src_ip)
            self._refill(bucket, now, self.per_source_burst, self.per_source_per_s)
        self._refill(self._global, now, self.global_burst, self.global_per_s)
        if bucket[0] < 1.0 or self._global[0] < 1.0:
            return False
        bucket[0] -= 1.0
        self._global[0] -= 1.0
        return True


class DiscoveryResponder:
    """Pure logic: datagram in -> reply bytes or None. ``status`` is a read-only callback
    returning (mode, sender_locked); it is the only contact with the bridge."""

    def __init__(self, data_port: int, status: Callable[[], Tuple[str, bool]], model: str = "",
                 robot_id: str = "", limiter: Optional[RateLimiter] = None):
        self.data_port = int(data_port)
        self.model, self.robot_id = model, robot_id
        self._status = status
        self._limiter = limiter or RateLimiter()
        self.stats = Counter()

    def handle(self, data: bytes, src_ip: str, src_port: int, now: float) -> Optional[bytes]:
        self.stats["rx"] += 1
        try:
            nonce = decode_probe(data)
        except ProbeError as e:
            self.stats["drop_" + e.reason] += 1
            return None
        if src_port < MIN_SOURCE_PORT or src_port in FORBIDDEN_PORTS:
            self.stats["drop_source_port"] += 1
            return None
        if not self._limiter.allow(src_ip, now):
            self.stats["drop_rate"] += 1
            return None
        mode, locked = self._status()
        reply = encode_reply(nonce, build_info(self.model, self.robot_id, self.data_port, mode, locked))
        assert len(reply) < MAX_REPLY_SIZE
        self.stats["replies"] += 1
        return reply


class DiscoveryServer:
    """The responder plus its UDP socket. ``service`` is called by the bridge's main loop when
    the socket is readable; it answers at most ``max_datagrams`` probes per call."""

    def __init__(self, responder: DiscoveryResponder, listen_ip: str, port: int):
        if port in FORBIDDEN_PORTS:
            raise ValueError("discovery port %d belongs to the robot's own teleop stack" % port)
        self.responder = responder
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((listen_ip, port))
            self.sock.setblocking(False)
        except OSError:
            self.sock.close()
            raise
        self.port = self.sock.getsockname()[1]

    def fileno(self) -> int:
        return self.sock.fileno()

    def service(self, now: float, max_datagrams: int = 32) -> int:
        handled = 0
        while handled < max_datagrams:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            handled += 1
            reply = self.responder.handle(data, addr[0], addr[1], now)
            if reply is not None:
                try:
                    self.sock.sendto(reply, addr)
                except OSError:
                    self.responder.stats["send_errors"] += 1
        return handled

    def close(self) -> None:
        self.sock.close()
