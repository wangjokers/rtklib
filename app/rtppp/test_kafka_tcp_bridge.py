#!/usr/bin/env python3
import argparse
import io
import struct
import unittest
from types import SimpleNamespace

from app.rtppp import kafka_tcp_bridge as bridge


def make_outer(dtype: int, data: bytes) -> bytes:
    extra = b"source"
    payload = bytes([dtype]) + struct.pack("<I", len(data)) + data
    header = bytearray(24)
    header[:4] = b"\xEE\xEE\xEE\xEE"
    struct.pack_into("<I", header, 19, len(payload))
    header[23] = len(extra)
    return bytes(header) + extra + payload


def make_record(value: bytes, offset: int) -> bytes:
    metadata = f"topic\t0\t{offset}\t{1000 + offset}\t{len(value)}\n"
    return metadata.encode() + value


class KafkaTcpBridgeTests(unittest.TestCase):
    def test_kcat_group_is_latest_binary_and_non_committing(self):
        args = argparse.Namespace(
            kcat="kcat",
            broker="broker:9092",
            group_id="dedicated-group",
            topic="topic",
            offset_reset="latest",
        )
        command = bridge.build_kcat_command(args)
        self.assertEqual(
            bridge.KCAT_RECORD_FORMAT, r"%t\t%p\t%o\t%T\t%S\n%s"
        )
        self.assertIn("dedicated-group", command)
        self.assertIn("auto.offset.reset=latest", command)
        self.assertIn("enable.auto.commit=false", command)
        self.assertIn("enable.auto.offset.store=false", command)

    def test_duplicate_and_backward_offsets_are_not_forwarded(self):
        stats = bridge.new_stats()
        offsets: dict[int, int] = {}
        timestamps: dict[int, int] = {}
        self.assertTrue(
            bridge.audit_record_order(stats, 0, 10, 100, offsets, timestamps)
        )
        self.assertTrue(
            bridge.audit_record_order(stats, 0, 12, 102, offsets, timestamps)
        )
        self.assertFalse(
            bridge.audit_record_order(stats, 0, 12, 102, offsets, timestamps)
        )
        self.assertFalse(
            bridge.audit_record_order(stats, 0, 11, 101, offsets, timestamps)
        )
        self.assertTrue(
            bridge.audit_record_order(stats, 0, 13, 103, offsets, timestamps)
        )
        self.assertEqual(stats["offset_gaps"], 1)
        self.assertEqual(stats["offset_duplicates"], 1)
        self.assertEqual(stats["offset_backwards"], 1)
        self.assertEqual(stats["timestamp_backwards"], 1)

    def test_binary_record_reader_uses_payload_size_not_newlines(self):
        first = make_outer(200, b"binary\nvalue\x00tail")
        second = make_outer(200, b"next\nrecord")
        process = SimpleNamespace(
            stdout=io.BytesIO(make_record(first, 1) + make_record(second, 2))
        )
        stats = bridge.new_stats()
        bridge.forward_records(process, {}, "topic", 0, stats)
        self.assertEqual(stats["records"], 2)
        self.assertEqual(stats["dtype_counts"][200], 2)
        self.assertEqual(stats["record_boundary_errors"], 0)


if __name__ == "__main__":
    unittest.main()
