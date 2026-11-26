"""Headset -> robot session datagram ("NTC1"): opens and closes the control page.

One fixed 20-byte UDP datagram, little-endian:

    4s  magic    b"NTC1"
    B   version  1
    B   kind     1 = hello (sent about once a second while connected), 2 = goodbye
    H   reserved 0
    I   session  random number chosen by the headset for this connection
    4s  word     four capital letters A-Z shown to the operator in the headset
    I   crc32    of the 16 bytes before it

The robot never answers.  Stdlib only.
"""
import struct
import zlib

MAGIC = b"NTC1"
VERSION = 1
HELLO, BYE = 1, 2
DEFAULT_PORT = 39502
_S = struct.Struct("<4sBBHI4sI")
SIZE = _S.size


def pack(kind: int, session: int, word: str) -> bytes:
    w = word.upper().encode("ascii")
    if len(w) != 4 or not w.isalpha():
        raise ValueError("word must be four letters")
    body = _S.pack(MAGIC, VERSION, kind, 0, session & 0xFFFFFFFF, w, 0)[:-4]
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def unpack(data: bytes):
    """-> (kind, session, word). Raises ValueError for anything that is not a valid datagram."""
    if len(data) != SIZE:
        raise ValueError("bad length")
    magic, version, kind, reserved, session, word, crc = _S.unpack(data)
    if magic != MAGIC or version != VERSION or reserved != 0 or kind not in (HELLO, BYE):
        raise ValueError("bad header")
    if crc != zlib.crc32(data[:-4]) & 0xFFFFFFFF:
        raise ValueError("bad crc")
    if not (word.isalpha() and word.isupper() and all(65 <= c <= 90 for c in word)):
        raise ValueError("bad word")
    return kind, session, word.decode("ascii")
