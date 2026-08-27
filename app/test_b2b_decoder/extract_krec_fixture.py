#!/usr/bin/env python3
"""Extract deterministic RTCM3/Sino fixtures from a binary kcat capture.

The capture format is one UTF-8 metadata line followed by exactly value_size
bytes of Kafka Value. Kafka timestamps are audit metadata only; GNSS time stays
inside the RTCM3 and Sino receiver frames.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
import struct
import sys
import tempfile
import zlib
from pathlib import Path
from typing import BinaryIO, Dict, Iterator, List, Tuple


MAGIC = b"\xEE\xEE\xEE\xEE"
SINO_SYNC = b"\xAA\x44\x12"
SINO_HEADER_LENGTH = 28
KNOWN_COUNTS = {
    "records": 36053,
    "dtype_131": 4834,
    "message_72": 580,
    "message_1697": 2192,
    "record_boundary_errors": 0,
}
GPS_EPOCH_UNIX = 315964800
TIMETAG_HEADER_LENGTH = 64


class FixtureError(RuntimeError):
    """A deterministic capture or inner-frame contract was violated."""


def _read_exact(stream: BinaryIO, size: int) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise FixtureError(
            f"record value truncated: expected {size} bytes, got {len(data)}"
        )
    return data


def iter_krec_records(stream: BinaryIO) -> Iterator[Tuple[str, int, int, int, bytes]]:
    """Yield topic, partition, offset, timestamp and exact Kafka Value."""
    record_index = 0
    while True:
        line = stream.readline()
        if line == b"":
            return
        record_index += 1
        if not line.endswith(b"\n"):
            raise FixtureError(f"record {record_index}: unterminated metadata line")
        fields = line[:-1].split(b"\t")
        if len(fields) != 5:
            raise FixtureError(
                f"record {record_index}: expected 5 metadata fields, got {len(fields)}"
            )
        try:
            topic = fields[0].decode("utf-8", "strict")
            partition = int(fields[1])
            offset = int(fields[2])
            timestamp = int(fields[3])
            value_size = int(fields[4])
        except (UnicodeDecodeError, ValueError) as exc:
            raise FixtureError(f"record {record_index}: invalid metadata: {exc}") from exc
        if value_size < 0:
            raise FixtureError(f"record {record_index}: negative value size")
        yield topic, partition, offset, timestamp, _read_exact(stream, value_size)


def parse_outer_items(value: bytes, record_index: int) -> Iterator[Tuple[int, bytes]]:
    """Validate one EEEEEEEE Value and yield every data item in wire order."""
    if len(value) < 24:
        raise FixtureError(f"record {record_index}: outer packet shorter than 24 bytes")
    if value[:4] != MAGIC:
        raise FixtureError(f"record {record_index}: invalid EEEEEEEE magic")
    payload_length = struct.unpack_from("<I", value, 19)[0]
    extra_length = value[23]
    item_start = 24 + extra_length
    expected_length = item_start + payload_length
    if expected_length != len(value):
        raise FixtureError(
            f"record {record_index}: PL/EL closure error: "
            f"24+{extra_length}+{payload_length}={expected_length}, value={len(value)}"
        )
    position = item_start
    item_index = 0
    while position < expected_length:
        item_index += 1
        if expected_length - position < 5:
            raise FixtureError(
                f"record {record_index} item {item_index}: truncated item header"
            )
        dtype = value[position]
        data_length = struct.unpack_from("<I", value, position + 1)[0]
        position += 5
        end = position + data_length
        if end > expected_length:
            raise FixtureError(
                f"record {record_index} item {item_index}: data length crosses PL boundary"
            )
        yield dtype, value[position:end]
        position = end
    if position != expected_length:
        raise FixtureError(f"record {record_index}: unconsumed outer packet bytes")


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1864CFB) if crc & 0x800000 else crc << 1
            crc &= 0xFFFFFF
    return crc


def validate_rtcm_frames(data: bytes, record_index: int) -> List[int]:
    """Return RTCM message numbers after strict no-resync frame validation."""
    messages: List[int] = []
    position = 0
    while position < len(data):
        if len(data) - position < 6 or data[position] != 0xD3:
            raise FixtureError(f"record {record_index}: invalid RTCM3 frame boundary")
        if data[position + 1] & 0xFC:
            raise FixtureError(f"record {record_index}: non-zero RTCM3 reserved bits")
        payload_length = ((data[position + 1] & 0x03) << 8) | data[position + 2]
        frame_length = 3 + payload_length + 3
        end = position + frame_length
        if end > len(data):
            raise FixtureError(f"record {record_index}: truncated RTCM3 frame")
        frame = data[position:end]
        stored_crc = int.from_bytes(frame[-3:], "big")
        if crc24q(frame[:-3]) != stored_crc:
            raise FixtureError(f"record {record_index}: RTCM3 CRC24Q mismatch")
        if payload_length < 2:
            raise FixtureError(f"record {record_index}: RTCM3 payload too short")
        messages.append((frame[3] << 4) | (frame[4] >> 4))
        position = end
    if not messages:
        raise FixtureError(f"record {record_index}: dtype 131 contains no RTCM3 frame")
    return messages


def rtk_crc32(data: bytes) -> int:
    return (zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF


def iter_sino_frames(data: bytes, record_index: int) -> Iterator[Tuple[int, bytes, int]]:
    """Yield message id, complete frame and header GNSS time in milliseconds."""
    position = 0
    while position < len(data):
        if len(data) - position < SINO_HEADER_LENGTH + 4:
            raise FixtureError(f"record {record_index}: truncated Sino frame")
        if data[position : position + 3] != SINO_SYNC:
            raise FixtureError(f"record {record_index}: invalid Sino frame boundary")
        header_length = data[position + 3]
        if header_length != SINO_HEADER_LENGTH:
            raise FixtureError(
                f"record {record_index}: unsupported Sino header length {header_length}"
            )
        payload_length = int.from_bytes(data[position + 8 : position + 10], "little")
        frame_length = header_length + payload_length + 4
        end = position + frame_length
        if end > len(data):
            raise FixtureError(f"record {record_index}: truncated Sino payload")
        frame = data[position:end]
        stored_crc = int.from_bytes(frame[-4:], "little")
        if rtk_crc32(frame[:-4]) != stored_crc:
            raise FixtureError(f"record {record_index}: Sino CRC32 mismatch")
        message = int.from_bytes(frame[4:6], "little")
        week = int.from_bytes(frame[14:16], "little")
        tow_ms = int.from_bytes(frame[16:20], "little")
        yield message, frame, week * 604800000 + tow_ms
        position = end


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def _counter_json(counter: collections.Counter) -> Dict[str, int]:
    return {str(key): counter[key] for key in sorted(counter)}


def write_rtklib_timetag(
    stream: BinaryIO, start_gnss_ms: int, entries: List[Tuple[int, int]]
) -> None:
    """Write the native RTKLIB 4-byte-position time-tag sidecar format."""
    header = bytearray(TIMETAG_HEADER_LENGTH)
    label = b"TIMETAG RTKLIB deterministic krec fixture"
    header[: len(label)] = label
    struct.pack_into("<I", header, TIMETAG_HEADER_LENGTH - 4, 0)
    whole_seconds, milliseconds = divmod(start_gnss_ms, 1000)
    stream.write(header)
    stream.write(struct.pack("<I", GPS_EPOCH_UNIX + whole_seconds))
    stream.write(struct.pack("<d", milliseconds / 1000.0))
    for tick, position in entries:
        if not (0 <= tick <= 0xFFFFFFFF and 0 <= position <= 0xFFFFFFFF):
            raise FixtureError("RTKLIB time-tag tick or file position exceeds u32")
        stream.write(struct.pack("<II", tick, position))


def extract_fixture(input_path: Path, output_dir: Path, enforce_known: bool) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    stats_path = output_dir / "stats.json"
    failed_stats_path = output_dir / "stats.failed.json"
    stats = {
        "schema": 1,
        "input": {
            "path": str(input_path.resolve()),
            "bytes": input_path.stat().st_size,
            "sha256": _sha256(input_path),
        },
        "records": 0,
        "record_boundary_errors": 0,
        "outer_errors": 0,
        "partitions": collections.Counter(),
        "dtype_counts": collections.Counter(),
        "records_with_multiple_items": 0,
        "offset_gaps": 0,
        "offset_duplicates": 0,
        "offset_backwards": 0,
        "kafka_timestamp_backwards": 0,
        "rtcm_frames": 0,
        "rtcm_message_counts": collections.Counter(),
        "sino_frames_scanned": 0,
        "sino_message_counts": collections.Counter(),
        "message_72": 0,
        "message_1697": 0,
        "message_1697_type_counts": collections.Counter(),
        "sino_selected_time_backwards": 0,
    }
    last_offset: Dict[int, int] = {}
    last_timestamp: Dict[int, int] = {}
    last_sino_time = None
    first_kafka_timestamp = None
    first_selected_sino_gnss_time = None
    obs_position = sino_position = 0
    obs_tag_entries: List[Tuple[int, int]] = []
    sino_tag_entries: List[Tuple[int, int]] = []
    temp_paths: List[Path] = []
    obs_temp = sino_temp = None

    try:
        with tempfile.NamedTemporaryFile("wb", dir=output_dir, delete=False) as obs_stream:
            obs_temp = Path(obs_stream.name)
            temp_paths.append(obs_temp)
            with tempfile.NamedTemporaryFile("wb", dir=output_dir, delete=False) as sino_stream:
                sino_temp = Path(sino_stream.name)
                temp_paths.append(sino_temp)
                with input_path.open("rb") as capture:
                    for record in iter_krec_records(capture):
                        topic, partition, offset, timestamp, value = record
                        stats["records"] += 1
                        record_index = stats["records"]
                        if first_kafka_timestamp is None:
                            first_kafka_timestamp = timestamp
                        replay_tick = timestamp - first_kafka_timestamp
                        if replay_tick < 0:
                            raise FixtureError(
                                f"record {record_index}: timestamp precedes capture start"
                            )
                        stats["partitions"][partition] += 1
                        if partition in last_offset:
                            delta = offset - last_offset[partition]
                            if delta == 0:
                                stats["offset_duplicates"] += 1
                            elif delta < 0:
                                stats["offset_backwards"] += 1
                            elif delta > 1:
                                stats["offset_gaps"] += delta - 1
                        if partition in last_timestamp and timestamp < last_timestamp[partition]:
                            stats["kafka_timestamp_backwards"] += 1
                        last_offset[partition] = offset
                        last_timestamp[partition] = timestamp

                        items = list(parse_outer_items(value, record_index))
                        if len(items) > 1:
                            stats["records_with_multiple_items"] += 1
                        for dtype, item in items:
                            stats["dtype_counts"][dtype] += 1
                            if dtype == 131:
                                messages = validate_rtcm_frames(item, record_index)
                                stats["rtcm_frames"] += len(messages)
                                stats["rtcm_message_counts"].update(messages)
                                obs_stream.write(item)
                                obs_position += len(item)
                                obs_tag_entries.append((replay_tick, obs_position))
                            elif dtype in (134, 135):
                                for message, frame, gnss_time in iter_sino_frames(
                                    item, record_index
                                ):
                                    stats["sino_frames_scanned"] += 1
                                    stats["sino_message_counts"][message] += 1
                                    selected = (dtype == 135 and message == 72) or (
                                        dtype == 134 and message == 1697
                                    )
                                    if not selected:
                                        continue
                                    if first_selected_sino_gnss_time is None:
                                        first_selected_sino_gnss_time = gnss_time
                                    if last_sino_time is not None and gnss_time < last_sino_time:
                                        stats["sino_selected_time_backwards"] += 1
                                    last_sino_time = gnss_time
                                    sino_stream.write(frame)
                                    sino_position += len(frame)
                                    sino_tag_entries.append((replay_tick, sino_position))
                                    if message == 72:
                                        stats["message_72"] += 1
                                    else:
                                        stats["message_1697"] += 1
                                        payload_bit = SINO_HEADER_LENGTH * 8
                                        message_type = (int.from_bytes(frame, "big") >> (
                                            len(frame) * 8 - (payload_bit + 50)
                                        )) & 0x3F
                                        stats["message_1697_type_counts"][message_type] += 1

        observed = {
            "records": stats["records"],
            "dtype_131": stats["dtype_counts"][131],
            "message_72": stats["message_72"],
            "message_1697": stats["message_1697"],
            "record_boundary_errors": stats["record_boundary_errors"],
        }
        if enforce_known and observed != KNOWN_COUNTS:
            raise FixtureError(f"known 10-minute counts mismatch: {observed}")
        order_errors = (
            stats["offset_gaps"]
            + stats["offset_duplicates"]
            + stats["offset_backwards"]
            + stats["kafka_timestamp_backwards"]
            + stats["sino_selected_time_backwards"]
        )
        if order_errors:
            raise FixtureError(f"capture order/time audit failed with {order_errors} errors")
        if first_selected_sino_gnss_time is None:
            raise FixtureError("no selected Sino frame available for GNSS replay epoch")

        with tempfile.NamedTemporaryFile("w+b", dir=output_dir, delete=False) as stream:
            obs_tag_temp = Path(stream.name)
            temp_paths.append(obs_tag_temp)
            write_rtklib_timetag(
                stream, first_selected_sino_gnss_time, obs_tag_entries
            )
        with tempfile.NamedTemporaryFile("w+b", dir=output_dir, delete=False) as stream:
            sino_tag_temp = Path(stream.name)
            temp_paths.append(sino_tag_temp)
            write_rtklib_timetag(
                stream, first_selected_sino_gnss_time, sino_tag_entries
            )

        obs_path = output_dir / "obs.rtcm3"
        sino_path = output_dir / "sino.bin"
        obs_tag_path = output_dir / "obs.rtcm3.tag"
        sino_tag_path = output_dir / "sino.bin.tag"
        os.replace(obs_temp, obs_path)
        temp_paths.remove(obs_temp)
        os.replace(sino_temp, sino_path)
        temp_paths.remove(sino_temp)
        os.replace(obs_tag_temp, obs_tag_path)
        temp_paths.remove(obs_tag_temp)
        os.replace(sino_tag_temp, sino_tag_path)
        temp_paths.remove(sino_tag_temp)
        stats["replay_timing"] = {
            "source": "Kafka timestamp relative intervals only",
            "absolute_epoch_source": "first selected Sino GNSS week/TOW",
            "start_gnss_ms": first_selected_sino_gnss_time,
            "duration_ms": max(last_timestamp.values()) - first_kafka_timestamp,
            "obs_tag_entries": len(obs_tag_entries),
            "sino_tag_entries": len(sino_tag_entries),
        }
        stats["outputs"] = {
            "obs.rtcm3": {
                "bytes": obs_path.stat().st_size,
                "sha256": _sha256(obs_path),
            },
            "sino.bin": {
                "bytes": sino_path.stat().st_size,
                "sha256": _sha256(sino_path),
            },
            "obs.rtcm3.tag": {
                "bytes": obs_tag_path.stat().st_size,
                "sha256": _sha256(obs_tag_path),
            },
            "sino.bin.tag": {
                "bytes": sino_tag_path.stat().st_size,
                "sha256": _sha256(sino_tag_path),
            },
        }
        stats["status"] = "PASS"
        serializable = dict(stats)
        for name in (
            "partitions",
            "dtype_counts",
            "rtcm_message_counts",
            "sino_message_counts",
            "message_1697_type_counts",
        ):
            serializable[name] = _counter_json(stats[name])
        with stats_path.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(serializable, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return serializable
    except (FixtureError, OSError) as exc:
        stats["status"] = "FAIL"
        stats["error"] = str(exc)
        stats["record_boundary_errors"] += int(
            isinstance(exc, FixtureError) and "record" in str(exc).lower()
        )
        serializable = dict(stats)
        for name in (
            "partitions",
            "dtype_counts",
            "rtcm_message_counts",
            "sino_message_counts",
            "message_1697_type_counts",
        ):
            serializable[name] = _counter_json(stats[name])
        with failed_stats_path.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(serializable, stream, indent=2, sort_keys=True)
            stream.write("\n")
        raise
    finally:
        for path in temp_paths:
            try:
                path.unlink()
            except FileNotFoundError:
                pass


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="read-only .krec path")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--expect-known-10min",
        action="store_true",
        help="require the frozen 2026-08-26 record/message counts",
    )
    return parser.parse_args(argv)


def main(argv: List[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        stats = extract_fixture(args.input, args.output_dir, args.expect_known_10min)
    except (FixtureError, OSError) as exc:
        print(f"extract_krec_fixture: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": stats["status"], "outputs": stats["outputs"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
