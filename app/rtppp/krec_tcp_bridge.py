#!/usr/bin/env python3
"""Replay a captured KREC stream into the frozen rtppp localhost TCP inputs.

The KREC and EEEEEEEE parsing contract is shared with the Phase 1 fixture
extractor.  This bridge only schedules the already validated binary payloads;
it does not decode or alter RTCM3 or Sino receiver messages.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterator


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from app.test_b2b_decoder.extract_krec_fixture import (  # noqa: E402
    FixtureError,
    iter_krec_records,
    iter_sino_frames,
    parse_outer_items,
    validate_rtcm_frames,
)


OBS_CHANNEL = "obs"
SINO_CHANNEL = "sino"


@dataclass(frozen=True)
class ReplayEvent:
    tick_ms: int
    channel: str
    data: bytes


@dataclass
class ReplayPlan:
    events: list[ReplayEvent]
    obs_data: bytes
    sino_data: bytes
    stats: dict[str, object]


@dataclass(frozen=True)
class RoutedValue:
    """Validated outputs selected from exactly one Kafka Record Value."""

    item_count: int
    dtype_counts: collections.Counter[int]
    rtcm_frames: int
    message_72: int
    message_1697: int
    outputs: tuple[tuple[str, bytes], ...]


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def _checked_records(stream: BinaryIO) -> Iterator[tuple[str, int, int, int, bytes]]:
    """Keep the record-reading boundary visible at the bridge call site."""
    yield from iter_krec_records(stream)


def route_record_value(value: bytes, record_index: int) -> RoutedValue:
    """Validate and route one complete EEEEEEEE Kafka Value atomically."""
    items = list(parse_outer_items(value, record_index))
    dtype_counts: collections.Counter[int] = collections.Counter()
    outputs: list[tuple[str, bytes]] = []
    rtcm_frames = message_72 = message_1697 = 0

    for dtype, item in items:
        dtype_counts[dtype] += 1
        if dtype == 131:
            rtcm_frames += len(validate_rtcm_frames(item, record_index))
            outputs.append((OBS_CHANNEL, item))
        elif dtype in (134, 135):
            for message, frame, _gnss_time in iter_sino_frames(item, record_index):
                selected = (dtype == 135 and message == 72) or (
                    dtype == 134 and message == 1697
                )
                if not selected:
                    continue
                outputs.append((SINO_CHANNEL, frame))
                if message == 72:
                    message_72 += 1
                else:
                    message_1697 += 1
    return RoutedValue(
        len(items),
        dtype_counts,
        rtcm_frames,
        message_72,
        message_1697,
        tuple(outputs),
    )


def classify_record_error(exc: FixtureError) -> str:
    """Map a strict parser failure to one stable bridge counter."""
    text = str(exc)
    if "RTCM3" in text:
        return "rtcm_crc_errors"
    if "Sino" in text:
        return "sino_crc_errors"
    if "item" in text and ("data length" in text or "item header" in text):
        return "dtype_length_errors"
    return "outer_errors"


def build_replay_plan(input_path: Path) -> ReplayPlan:
    """Strictly validate one capture and retain selected bytes in record order."""
    events: list[ReplayEvent] = []
    obs_chunks: list[bytes] = []
    sino_chunks: list[bytes] = []
    dtype_counts: collections.Counter[int] = collections.Counter()
    partitions: collections.Counter[int] = collections.Counter()
    last_offset: dict[int, int] = {}
    last_timestamp: dict[int, int] = {}
    first_timestamp: int | None = None
    stats: dict[str, object] = {
        "records": 0,
        "record_boundary_errors": 0,
        "outer_errors": 0,
        "dtype_length_errors": 0,
        "offset_gaps": 0,
        "offset_duplicates": 0,
        "offset_backwards": 0,
        "kafka_timestamp_backwards": 0,
        "records_with_multiple_items": 0,
        "rtcm_frames": 0,
        "rtcm_crc_errors": 0,
        "sino_crc_errors": 0,
        "message_72": 0,
        "message_1697": 0,
    }

    try:
        with input_path.open("rb") as capture:
            for topic, partition, offset, timestamp, value in _checked_records(capture):
                del topic  # Topic text is metadata; routing is by the binary dtype.
                stats["records"] = int(stats["records"]) + 1
                record_index = int(stats["records"])
                partitions[partition] += 1
                if first_timestamp is None:
                    first_timestamp = timestamp
                tick_ms = timestamp - first_timestamp
                if tick_ms < 0:
                    raise FixtureError(
                        f"record {record_index}: timestamp precedes capture start"
                    )

                if partition in last_offset:
                    delta = offset - last_offset[partition]
                    if delta == 0:
                        stats["offset_duplicates"] = int(stats["offset_duplicates"]) + 1
                    elif delta < 0:
                        stats["offset_backwards"] = int(stats["offset_backwards"]) + 1
                    elif delta > 1:
                        stats["offset_gaps"] = int(stats["offset_gaps"]) + delta - 1
                if (
                    partition in last_timestamp
                    and timestamp < last_timestamp[partition]
                ):
                    stats["kafka_timestamp_backwards"] = int(
                        stats["kafka_timestamp_backwards"]
                    ) + 1
                last_offset[partition] = offset
                last_timestamp[partition] = timestamp

                try:
                    routed = route_record_value(value, record_index)
                except FixtureError as exc:
                    key = classify_record_error(exc)
                    stats[key] = int(stats[key]) + 1
                    raise
                if routed.item_count > 1:
                    stats["records_with_multiple_items"] = int(
                        stats["records_with_multiple_items"]
                    ) + 1
                dtype_counts.update(routed.dtype_counts)
                stats["rtcm_frames"] = int(stats["rtcm_frames"]) + routed.rtcm_frames
                stats["message_72"] = int(stats["message_72"]) + routed.message_72
                stats["message_1697"] = (
                    int(stats["message_1697"]) + routed.message_1697
                )
                for channel, chunk in routed.outputs:
                    if channel == OBS_CHANNEL:
                        obs_chunks.append(chunk)
                    else:
                        sino_chunks.append(chunk)
                    events.append(ReplayEvent(tick_ms, channel, chunk))
    except FixtureError as exc:
        if "record value" in str(exc) or "metadata" in str(exc):
            stats["record_boundary_errors"] = int(
                stats["record_boundary_errors"]
            ) + 1
        raise

    order_errors = sum(
        int(stats[name])
        for name in (
            "offset_gaps",
            "offset_duplicates",
            "offset_backwards",
            "kafka_timestamp_backwards",
        )
    )
    if order_errors:
        raise FixtureError(f"capture record order audit failed with {order_errors} errors")
    if not events or not obs_chunks or not sino_chunks:
        raise FixtureError("capture does not contain both selected output streams")

    obs_data = b"".join(obs_chunks)
    sino_data = b"".join(sino_chunks)
    stats["partitions"] = dict(sorted(partitions.items()))
    stats["dtype_counts"] = dict(sorted(dtype_counts.items()))
    stats["events"] = len(events)
    stats["duration_ms"] = events[-1].tick_ms
    stats["obs_bytes"] = len(obs_data)
    stats["sino_bytes"] = len(sino_data)
    stats["obs_sha256"] = sha256_bytes(obs_data)
    stats["sino_sha256"] = sha256_bytes(sino_data)
    return ReplayPlan(events, obs_data, sino_data, stats)


def verify_fixture(name: str, actual: bytes, expected_path: Path | None) -> None:
    if expected_path is None:
        return
    expected = expected_path.read_bytes()
    if actual != expected:
        raise FixtureError(
            f"{name} differs from frozen fixture: "
            f"actual={len(actual)}/{sha256_bytes(actual)} "
            f"expected={len(expected)}/{sha256_bytes(expected)}"
        )


def open_listener(host: str, port: int) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    return listener


def replay_channel(
    connection: socket.socket,
    events: list[ReplayEvent],
    start: float,
    speed: float,
    sent: dict[str, int],
    errors: list[BaseException],
) -> None:
    channel = events[0].channel
    try:
        for event in events:
            deadline = start + event.tick_ms / (speed * 1000.0)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.05))
            connection.sendall(event.data)
            sent[channel] += len(event.data)
        connection.shutdown(socket.SHUT_WR)
    except BaseException as exc:  # Surface sender failures in the main thread.
        errors.append(exc)
    finally:
        connection.close()


def replay(
    plan: ReplayPlan,
    host: str,
    obs_port: int,
    sino_port: int,
    speed: float,
) -> tuple[int, int]:
    obs_listener = open_listener(host, obs_port)
    try:
        sino_listener = open_listener(host, sino_port)
    except BaseException:
        obs_listener.close()
        raise
    print(
        f"bridge: listening obs={host}:{obs_port} sino={host}:{sino_port} "
        f"speed={speed:g}",
        flush=True,
    )
    try:
        obs_connection, _ = obs_listener.accept()
        sino_connection, _ = sino_listener.accept()
    finally:
        obs_listener.close()
        sino_listener.close()
    print("bridge: both rtppp clients connected", flush=True)

    channel_events = {
        channel: [event for event in plan.events if event.channel == channel]
        for channel in (OBS_CHANNEL, SINO_CHANNEL)
    }
    sent = {OBS_CHANNEL: 0, SINO_CHANNEL: 0}
    errors: list[BaseException] = []
    start = time.monotonic() + 0.05
    threads = [
        threading.Thread(
            target=replay_channel,
            args=(
                connection,
                channel_events[channel],
                start,
                speed,
                sent,
                errors,
            ),
            name=f"{channel}-bridge",
        )
        for channel, connection in (
            (OBS_CHANNEL, obs_connection),
            (SINO_CHANNEL, sino_connection),
        )
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    if errors:
        raise errors[0]
    return sent[OBS_CHANNEL], sent[SINO_CHANNEL]


def print_plan(plan: ReplayPlan) -> None:
    stats = plan.stats
    print(
        "bridge: records={records} events={events} duration_ms={duration_ms} "
        "offset_gaps={offset_gaps} offset_duplicates={offset_duplicates} "
        "offset_backwards={offset_backwards} "
        "timestamp_backwards={kafka_timestamp_backwards}".format(**stats)
    )
    print(
        "bridge: record_boundary_errors={record_boundary_errors} "
        "eeee_errors={outer_errors} dtype_length_errors={dtype_length_errors} "
        "rtcm_crc_errors={rtcm_crc_errors} sino_crc_errors={sino_crc_errors}".format(
            **stats
        )
    )
    print(
        "bridge: dtype131={dtype131} rtcm_frames={rtcm_frames} "
        "message72={message_72} message1697={message_1697}".format(
            dtype131=stats["dtype_counts"].get(131, 0), **stats
        )
    )
    print(
        f"bridge: obs_bytes={stats['obs_bytes']} "
        f"obs_sha256={stats['obs_sha256']}"
    )
    print(
        f"bridge: sino_bytes={stats['sino_bytes']} "
        f"sino_sha256={stats['sino_sha256']}"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--obs-port", default=23001, type=int)
    parser.add_argument("--sino-port", default=23002, type=int)
    parser.add_argument("--speed", default=10.0, type=float)
    parser.add_argument("--expect-obs", type=Path)
    parser.add_argument("--expect-sino", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if not 1 <= args.obs_port <= 65535 or not 1 <= args.sino_port <= 65535:
        parser.error("TCP ports must be in 1..65535")
    if args.obs_port == args.sino_port:
        parser.error("observation and Sino ports must differ")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = build_replay_plan(args.input)
        verify_fixture("observation output", plan.obs_data, args.expect_obs)
        verify_fixture("Sino output", plan.sino_data, args.expect_sino)
        print_plan(plan)
        if args.expect_obs or args.expect_sino:
            print("bridge: frozen_fixture_match=1")
        if args.validate_only:
            print("bridge: validation_only=1 completed=1")
            return 0
        obs_sent, sino_sent = replay(
            plan, args.host, args.obs_port, args.sino_port, args.speed
        )
        print(
            f"bridge: completed=1 obs_sent={obs_sent}/{len(plan.obs_data)} "
            f"sino_sent={sino_sent}/{len(plan.sino_data)}"
        )
        return 0
    except (FixtureError, OSError) as exc:
        print(f"bridge: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
