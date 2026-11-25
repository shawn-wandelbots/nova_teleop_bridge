"""Tiny record-file formats (stdlib only).

1. Bridge record (``teleop_bridge.py --record``):
       file header  : 8 bytes  b"NTBREC1\\n"
       per record   : <B kind> <Q t_unix_ns> <I length> <payload>
       kind 1 = encoded aimdk.protocol.XRFrameData, kind 2 = aimdk.protocol.VRButtonChannel
2. Raw capture (``tools/capture_udp.py``):
       file header  : 8 bytes  b"NTBCAP1\\n"
       per record   : <Q t_monotonic_ns since first datagram> <I length> <datagram>
"""
import struct

REC_MAGIC = b"NTBREC1\n"
CAP_MAGIC = b"NTBCAP1\n"
KIND_XR_FRAME = 1
KIND_VR_BUTTON = 2
_REC_HDR = struct.Struct("<BQI")
_CAP_HDR = struct.Struct("<QI")


class RecordWriter:
    def __init__(self, path):
        self._fh = open(path, "wb")
        self._fh.write(REC_MAGIC)

    def write(self, kind, t_ns, payload):
        self._fh.write(_REC_HDR.pack(kind, t_ns, len(payload)))
        self._fh.write(payload)

    def close(self):
        if self._fh:
            self._fh.flush()
            self._fh.close()
            self._fh = None


def read_record(path):
    """Return a list of (kind, t_ns, payload)."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:8] != REC_MAGIC:
        raise ValueError("not a bridge record file")
    out, pos = [], 8
    while pos + _REC_HDR.size <= len(data):
        kind, t_ns, ln = _REC_HDR.unpack_from(data, pos)
        pos += _REC_HDR.size
        if pos + ln > len(data):
            break  # truncated tail (e.g. killed mid-write)
        out.append((kind, t_ns, data[pos:pos + ln]))
        pos += ln
    return out


class CaptureWriter:
    def __init__(self, path):
        self._fh = open(path, "wb")
        self._fh.write(CAP_MAGIC)

    def write(self, t_ns, payload):
        self._fh.write(_CAP_HDR.pack(t_ns, len(payload)))
        self._fh.write(payload)

    def close(self):
        self._fh.close()


def read_capture(path):
    """Return a list of (t_ns, datagram)."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:8] != CAP_MAGIC:
        raise ValueError("not a capture file")
    out, pos = [], 8
    while pos + _CAP_HDR.size <= len(data):
        t_ns, ln = _CAP_HDR.unpack_from(data, pos)
        pos += _CAP_HDR.size
        if pos + ln > len(data):
            break
        out.append((t_ns, data[pos:pos + ln]))
        pos += ln
    return out
