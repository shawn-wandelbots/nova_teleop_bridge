import math
import struct
import unittest
import zlib

from teleop_common import wire


def make_frame(body=True, seq=1):
    f = wire.Frame(session=0xDEADBEEF, seq=seq, t_send_ns=1700000000123456789, t_sdk_ns=42,
                   flags=wire.FLAG_HEAD | wire.FLAG_LEFT | wire.FLAG_RIGHT)
    f.head = (0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0)
    f.left_pose = (-0.5, 1.0, -0.25, 0.0, 0.0, 0.0, 1.0)
    f.right_pose = (0.5, 1.0, -0.25, 0.0, 0.0, 0.0, 1.0)
    f.left = wire.ControllerState(0.25, -0.5, 0.75, 1.0, False, True, False)
    f.right = wire.ControllerState(-1.0, 1.0, 0.0, 0.5, True, False, True)
    if body:
        f.body = [(i * 0.125, 1.0, -i * 0.25, 0.0, 0.0, 0.0, 1.0) for i in range(24)]
    return f


def refresh_crc(buf: bytearray) -> bytes:
    buf[-4:] = struct.pack("<I", zlib.crc32(bytes(buf[:-4])) & 0xFFFFFFFF)
    return bytes(buf)


class TestWire(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(wire.HEADER_SIZE, 32)
        self.assertEqual(len(wire.pack(make_frame(True))), 832)
        self.assertEqual(len(wire.pack(make_frame(False))), 160)
        self.assertEqual((wire.SIZE_WITH_BODY, wire.SIZE_NO_BODY), (832, 160))

    def test_roundtrip(self):
        for body in (True, False):
            f = make_frame(body)
            g = wire.unpack(wire.pack(f))
            f.flags |= wire.FLAG_BODY if body else 0
            self.assertEqual(g, f)        # all values chosen to be exact in float32

    def test_header_layout(self):
        d = wire.pack(make_frame(True, seq=77))
        self.assertEqual(d[:4], b"NTB1")
        self.assertEqual(d[4], 1)
        self.assertEqual(d[6], 24)
        self.assertEqual(struct.unpack_from("<IIQQ", d, 8), (0xDEADBEEF, 77, 1700000000123456789, 42))

    def test_reject_short(self):
        d = wire.pack(make_frame())
        for n in (0, 3, 31):
            with self.assertRaises(wire.WireError) as cm:
                wire.unpack(d[:n])
            self.assertEqual(cm.exception.reason, "short")
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(d[:-1])
        self.assertEqual(cm.exception.reason, "bad_length")
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(d + b"\x00")
        self.assertEqual(cm.exception.reason, "bad_length")

    def test_reject_magic_version(self):
        d = bytearray(wire.pack(make_frame()))
        bad = bytearray(d)
        bad[0:4] = b"XXXX"
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(refresh_crc(bad))
        self.assertEqual(cm.exception.reason, "bad_magic")
        bad = bytearray(d)
        bad[4] = 2
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(refresh_crc(bad))
        self.assertEqual(cm.exception.reason, "bad_version")

    def test_reject_crc_and_flags(self):
        d = bytearray(wire.pack(make_frame()))
        bad = bytearray(d)
        bad[100] ^= 0x01
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(bytes(bad))
        self.assertEqual(cm.exception.reason, "bad_crc")
        bad = bytearray(d)
        bad[5] |= 0x40               # unknown flag bit
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(refresh_crc(bad))
        self.assertEqual(cm.exception.reason, "bad_flags")
        bad = bytearray(d)
        bad[6] = 23                  # body count mismatch
        with self.assertRaises(wire.WireError) as cm:
            wire.unpack(refresh_crc(bad))
        self.assertEqual(cm.exception.reason, "bad_body_count")

    def test_raw_pico_protobuf_is_rejected(self):
        with self.assertRaises(wire.WireError):
            wire.unpack(b"\x08\xff\xff\xff\xff\x0f" + b"\x22\x29" * 500)

    def test_nan_survives_transport(self):
        f = make_frame()
        f.body[3] = (math.nan, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.assertTrue(math.isnan(wire.unpack(wire.pack(f)).body[3][0]))   # rejection is the bridge's job

    def test_pack_validates_body_shape(self):
        f = make_frame()
        f.body = f.body[:23]
        with self.assertRaises(ValueError):
            wire.pack(f)


if __name__ == "__main__":
    unittest.main()
