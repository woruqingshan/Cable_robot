"""Bounded RGB8 PNG validation using only the standard library.

Stage 1 accepts non-interlaced, 8-bit RGB PNGs. Decode bounds are checked before
inflation; CRCs, complete IDAT stream, scanline lengths and filter tags are checked.
"""
import struct
import zlib
from .errors import VLAError
from .profiles import MAX_IMAGE_BYTES, MAX_IMAGE_PIXELS

SIGNATURE = b"\x89PNG\r\n\x1a\n"


def chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", zlib.crc32(kind + data))


def encode_rgb(width: int, height: int, rgb: bytes) -> bytes:
    if not 0 < width * height <= MAX_IMAGE_PIXELS or len(rgb) != width * height * 3:
        raise VLAError("INVALID_IMAGE", "RGB dimensions/buffer mismatch")
    rows = b"".join(b"\0" + rgb[i:i + width * 3] for i in range(0, len(rgb), width * 3))
    data = SIGNATURE + chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b"")
    if len(data) > MAX_IMAGE_BYTES:
        raise VLAError("PAYLOAD_TOO_LARGE", "encoded PNG exceeds wire ceiling")
    return data


def validate_png(data: bytes, width: int, height: int) -> None:
    try:
        if len(data) > MAX_IMAGE_BYTES or data[:8] != SIGNATURE:
            raise ValueError("PNG signature/size")
        if width <= 0 or height <= 0 or width * height > MAX_IMAGE_PIXELS:
            raise ValueError("pixel limit")
        pos, index, ended, idat_closed = 8, 0, False, False
        compressed = bytearray()
        while pos < len(data):
            if pos + 12 > len(data):
                raise ValueError("truncated chunk")
            size = struct.unpack("!I", data[pos:pos + 4])[0]
            kind = data[pos + 4:pos + 8]
            end = pos + 12 + size
            if end > len(data):
                raise ValueError("chunk size")
            payload = data[pos + 8:pos + 8 + size]
            if zlib.crc32(kind + payload) != struct.unpack("!I", data[end - 4:end])[0]:
                raise ValueError("chunk CRC")
            if index == 0:
                if kind != b"IHDR" or payload != struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0):
                    raise ValueError("expected declared non-interlaced RGB8 IHDR")
            elif kind == b"IHDR":
                raise ValueError("duplicate IHDR")
            if kind == b"IDAT":
                if idat_closed:
                    raise ValueError("non-contiguous IDAT")
                compressed.extend(payload)
            elif compressed:
                idat_closed = True
            if kind == b"IEND":
                if payload or end != len(data):
                    raise ValueError("invalid IEND/trailing data")
                ended = True
            elif kind not in (b"IHDR", b"IDAT") and not kind[0] & 32:
                raise ValueError("unsupported critical chunk")
            pos, index = end, index + 1
        if not ended or not compressed:
            raise ValueError("missing IDAT/IEND")
        expected = height * (width * 3 + 1)
        inflater = zlib.decompressobj()
        raw = inflater.decompress(bytes(compressed), expected + 1)
        if len(raw) != expected or not inflater.eof or inflater.unconsumed_tail or inflater.unused_data:
            raise ValueError("invalid/oversized decompressed stream")
        if any(raw[i] > 4 for i in range(0, expected, width * 3 + 1)):
            raise ValueError("invalid PNG filter")
    except (ValueError, struct.error, zlib.error, IndexError) as exc:
        raise VLAError("INVALID_IMAGE", str(exc)) from exc
