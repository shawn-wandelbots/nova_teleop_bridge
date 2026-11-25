import os
import socket
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "tests", "data")
SAMPLE_SPARSE = os.path.join(DATA, "pico_sample_frames.cap")        # every 100th frame of the PICO sample
SAMPLE_CONTIGUOUS = os.path.join(DATA, "pico_sample_contiguous.cap")  # frames 880..999 (fastest motion)
CONFIG = os.path.join(ROOT, "config", "transform.json")
# The fixtures are excerpts of a vendor recording and are not kept in git (see README).
HAVE_SAMPLE = os.path.exists(SAMPLE_SPARSE) and os.path.exists(SAMPLE_CONTIGUOUS)
needs_sample = unittest.skipUnless(HAVE_SAMPLE, "sample fixtures missing: see README, Test fixtures")
FORBIDDEN_PORTS = (25618, 25619, 15000, 51000)


def free_udp_port(start=39200, end=39999):
    """A free loopback UDP port in the private test range (>= 39000)."""
    for port in range(start, end):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("127.0.0.1", port))
            return port
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError("no free test port")
