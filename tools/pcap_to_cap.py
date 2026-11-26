#!/usr/bin/env python3
"""Turn a tcpdump capture of headset -> robot teleop traffic into this repo's .cap format.

    python3 tools/pcap_to_cap.py capture.pcap tests/data/genielink_sample.cap --port 25618

Only UDP payloads to --port are kept.  Classic pcap (not pcapng), Ethernet / Linux cooked.
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.recording import CAP_MAGIC  # noqa: E402

_LINK_HEADER = {1: 14, 113: 16, 276: 20}


def read_pcap(path, port):
    """Yield (t_seconds, udp_payload)."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:4] != b"\xd4\xc3\xb2\xa1":
        raise SystemExit("not a little-endian classic pcap file")
    link = struct.unpack_from("<I", data, 20)[0]
    if link not in _LINK_HEADER:
        raise SystemExit("unsupported link type %d" % link)
    pos = 24
    while pos + 16 <= len(data):
        sec, usec, caplen, _ = struct.unpack_from("<IIII", data, pos)
        pkt = data[pos + 16:pos + 16 + caplen]
        pos += 16 + caplen
        ip = _LINK_HEADER[link]
        if len(pkt) < ip + 28 or pkt[ip] >> 4 != 4 or pkt[ip + 9] != 17:
            continue
        udp = ip + (pkt[ip] & 15) * 4
        if struct.unpack_from(">H", pkt, udp + 2)[0] != port:
            continue
        yield sec + usec / 1e6, pkt[udp + 8:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcap")
    ap.add_argument("out")
    ap.add_argument("--port", type=int, default=25618)
    ap.add_argument("--every", type=int, default=1, help="keep every Nth datagram")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--count", type=int, default=0, help="0 = all")
    a = ap.parse_args()
    n = 0
    t0 = None
    with open(a.out, "wb") as out:
        out.write(CAP_MAGIC)
        for i, (t, payload) in enumerate(read_pcap(a.pcap, a.port)):
            if i < a.start or (i - a.start) % a.every:
                continue
            t0 = t if t0 is None else t0
            out.write(struct.pack("<QI", int((t - t0) * 1e9), len(payload)))
            out.write(payload)
            n += 1
            if a.count and n >= a.count:
                break
    print("wrote %d datagrams to %s" % (n, a.out))


if __name__ == "__main__":
    main()
