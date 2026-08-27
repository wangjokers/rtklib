#!/usr/bin/env python3
import importlib.util
import io
import struct
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("extract_krec_fixture.py")
SPEC = importlib.util.spec_from_file_location("extract_krec_fixture", MODULE_PATH)
extractor = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(extractor)


def make_outer(items):
    extra = b"source"
    payload = b"".join(bytes([dtype]) + struct.pack("<I", len(data)) + data for dtype, data in items)
    header = bytearray(24)
    header[:4] = extractor.MAGIC
    struct.pack_into("<I", header, 19, len(payload))
    header[23] = len(extra)
    return bytes(header) + extra + payload


def make_record(value, offset=1, timestamp=2):
    return f"topic\t0\t{offset}\t{timestamp}\t{len(value)}\n".encode() + value


class KrecFixtureTests(unittest.TestCase):
    def test_record_boundary_preserves_binary_newlines(self):
        first = b"abc\n\x00def"
        second = b"xyz"
        data = make_record(first, 10, 20) + make_record(second, 11, 21)
        records = list(extractor.iter_krec_records(io.BytesIO(data)))
        self.assertEqual([record[4] for record in records], [first, second])

    def test_outer_packet_iterates_all_items(self):
        packet = make_outer([(131, b"one"), (134, b"two"), (135, b"three")])
        self.assertEqual(
            list(extractor.parse_outer_items(packet, 1)),
            [(131, b"one"), (134, b"two"), (135, b"three")],
        )

    def test_outer_length_mismatch_is_rejected(self):
        packet = bytearray(make_outer([(131, b"one")]))
        struct.pack_into("<I", packet, 19, 999)
        with self.assertRaises(extractor.FixtureError):
            list(extractor.parse_outer_items(bytes(packet), 1))

    def test_truncated_record_value_is_rejected(self):
        capture = io.BytesIO(b"topic\t0\t1\t2\t10\nshort")
        with self.assertRaises(extractor.FixtureError):
            list(extractor.iter_krec_records(capture))

    def test_rtklib_timetag_layout(self):
        stream = io.BytesIO()
        extractor.write_rtklib_timetag(stream, 1000, [(0, 10), (25, 30)])
        data = stream.getvalue()
        self.assertEqual(len(data), 64 + 4 + 8 + 2 * 8)
        self.assertEqual(struct.unpack_from("<I", data, 60)[0], 0)
        self.assertEqual(
            struct.unpack_from("<I", data, 64)[0], extractor.GPS_EPOCH_UNIX + 1
        )
        self.assertEqual(struct.unpack_from("<II", data, 76), (0, 10))
        self.assertEqual(struct.unpack_from("<II", data, 84), (25, 30))

    def test_known_count_gate_rejects_synthetic_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "tiny.krec"
            output = Path(directory) / "out"
            capture.write_bytes(make_record(make_outer([(200, b"x")])))
            with self.assertRaises(extractor.FixtureError):
                extractor.extract_fixture(capture, output, True)
            self.assertTrue((output / "stats.failed.json").is_file())
            self.assertFalse((output / "obs.rtcm3").exists())


if __name__ == "__main__":
    unittest.main()
