#!/usr/bin/env python3
"""Consume live Kafka Values with kcat and feed the frozen rtppp TCP inputs.

kcat writes one metadata line containing the payload size followed immediately
by that many raw Value bytes.  The binary stdout is therefore consumed with
the same record reader used by the validated KREC path; it is never split on
payload newlines or decoded as text.
"""

from __future__ import annotations

import argparse
import collections
import shlex
import shutil
import socket
import subprocess
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from app.rtppp.krec_tcp_bridge import (  # noqa: E402
    OBS_CHANNEL,
    SINO_CHANNEL,
    classify_record_error,
    open_listener,
    route_record_value,
)
from app.test_b2b_decoder.extract_krec_fixture import (  # noqa: E402
    FixtureError,
    iter_krec_records,
)


KCAT_RECORD_FORMAT = r"%t\t%p\t%o\t%T\t%S\n%s"


def new_stats() -> dict[str, object]:
    return {
        "records": 0,
        "offset_gaps": 0,
        "offset_duplicates": 0,
        "offset_backwards": 0,
        "timestamp_backwards": 0,
        "record_boundary_errors": 0,
        "outer_errors": 0,
        "dtype_length_errors": 0,
        "rtcm_crc_errors": 0,
        "sino_crc_errors": 0,
        "rtcm_frames": 0,
        "message_72": 0,
        "message_1697": 0,
        "obs_sent": 0,
        "sino_sent": 0,
        "partitions": collections.Counter(),
        "dtype_counts": collections.Counter(),
    }


def build_kcat_command(args: argparse.Namespace) -> list[str]:
    """Build a group consumer that starts live and never commits offsets."""
    return [
        args.kcat,
        "-u",
        "-b",
        args.broker,
        "-G",
        args.group_id,
        args.topic,
        "-X",
        "security.protocol=PLAINTEXT",
        "-X",
        f"auto.offset.reset={args.offset_reset}",
        "-X",
        "enable.auto.commit=false",
        "-X",
        "enable.auto.offset.store=false",
        "-X",
        "socket.keepalive.enable=true",
        "-f",
        KCAT_RECORD_FORMAT,
    ]


def audit_record_order(
    stats: dict[str, object],
    partition: int,
    offset: int,
    timestamp: int,
    last_offsets: dict[int, int],
    last_timestamps: dict[int, int],
) -> bool:
    forward = True
    stats["partitions"][partition] += 1
    if partition in last_offsets:
        delta = offset - last_offsets[partition]
        if delta == 0:
            stats["offset_duplicates"] = int(stats["offset_duplicates"]) + 1
            forward = False
        elif delta < 0:
            stats["offset_backwards"] = int(stats["offset_backwards"]) + 1
            forward = False
        elif delta > 1:
            stats["offset_gaps"] = int(stats["offset_gaps"]) + delta - 1
    if partition in last_timestamps and timestamp < last_timestamps[partition]:
        stats["timestamp_backwards"] = int(stats["timestamp_backwards"]) + 1
    if forward:
        last_offsets[partition] = offset
    last_timestamps[partition] = max(
        timestamp, last_timestamps.get(partition, timestamp)
    )
    return forward


def print_stats(stats: dict[str, object], prefix: str = "kafka-bridge") -> None:
    dtype_counts = stats["dtype_counts"]
    print(
        f"{prefix}: records={stats['records']} "
        f"offset_gaps={stats['offset_gaps']} "
        f"offset_duplicates={stats['offset_duplicates']} "
        f"offset_backwards={stats['offset_backwards']} "
        f"timestamp_backwards={stats['timestamp_backwards']}",
        flush=True,
    )
    print(
        f"{prefix}: record_boundary_errors={stats['record_boundary_errors']} "
        f"eeee_errors={stats['outer_errors']} "
        f"dtype_length_errors={stats['dtype_length_errors']} "
        f"rtcm_crc_errors={stats['rtcm_crc_errors']} "
        f"sino_crc_errors={stats['sino_crc_errors']}",
        flush=True,
    )
    print(
        f"{prefix}: dtype131={dtype_counts.get(131, 0)} "
        f"rtcm_frames={stats['rtcm_frames']} "
        f"message72={stats['message_72']} "
        f"message1697={stats['message_1697']} "
        f"obs_sent={stats['obs_sent']} sino_sent={stats['sino_sent']}",
        flush=True,
    )


def accept_rtppp(
    host: str, obs_port: int, sino_port: int
) -> dict[str, socket.socket]:
    obs_listener = open_listener(host, obs_port)
    try:
        sino_listener = open_listener(host, sino_port)
    except BaseException:
        obs_listener.close()
        raise
    print(
        f"kafka-bridge: listening obs={host}:{obs_port} "
        f"sino={host}:{sino_port}",
        flush=True,
    )
    try:
        obs_connection, _ = obs_listener.accept()
        sino_connection, _ = sino_listener.accept()
    finally:
        obs_listener.close()
        sino_listener.close()
    print("kafka-bridge: both rtppp clients connected", flush=True)
    return {OBS_CHANNEL: obs_connection, SINO_CHANNEL: sino_connection}


def close_outputs(connections: dict[str, socket.socket]) -> None:
    for connection in connections.values():
        try:
            connection.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        connection.close()


def stop_consumer(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def forward_records(
    process: subprocess.Popen[bytes],
    connections: dict[str, socket.socket],
    topic_expected: str,
    stats_every: int,
    stats: dict[str, object],
) -> None:
    if process.stdout is None:
        raise RuntimeError("kcat stdout pipe is unavailable")
    records = iter(iter_krec_records(process.stdout))
    last_offsets: dict[int, int] = {}
    last_timestamps: dict[int, int] = {}

    while True:
        try:
            topic, partition, offset, timestamp, value = next(records)
        except StopIteration:
            return
        except FixtureError:
            stats["record_boundary_errors"] = int(
                stats["record_boundary_errors"]
            ) + 1
            raise

        stats["records"] = int(stats["records"]) + 1
        record_index = int(stats["records"])
        if topic != topic_expected:
            raise FixtureError(
                f"record {record_index}: unexpected topic {topic!r}"
            )
        forward = audit_record_order(
            stats,
            partition,
            offset,
            timestamp,
            last_offsets,
            last_timestamps,
        )
        if not forward:
            continue
        try:
            routed = route_record_value(value, record_index)
        except FixtureError as exc:
            key = classify_record_error(exc)
            stats[key] = int(stats[key]) + 1
            raise

        stats["dtype_counts"].update(routed.dtype_counts)
        stats["rtcm_frames"] = int(stats["rtcm_frames"]) + routed.rtcm_frames
        stats["message_72"] = int(stats["message_72"]) + routed.message_72
        stats["message_1697"] = (
            int(stats["message_1697"]) + routed.message_1697
        )
        for channel, chunk in routed.outputs:
            connections[channel].sendall(chunk)
            key = "obs_sent" if channel == OBS_CHANNEL else "sino_sent"
            stats[key] = int(stats[key]) + len(chunk)
        if stats_every and record_index % stats_every == 0:
            print_stats(stats)


def run_live(args: argparse.Namespace) -> int:
    executable = shutil.which(args.kcat)
    if executable is None:
        raise RuntimeError(f"kcat executable not found: {args.kcat}")
    command = build_kcat_command(args)
    command[0] = executable
    connections = accept_rtppp(args.host, args.obs_port, args.sino_port)
    stats = new_stats()
    process: subprocess.Popen[bytes] | None = None
    interrupted = False
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            bufsize=0,
        )
        print(
            f"kafka-bridge: consumer_started=1 broker={args.broker} "
            f"topic={args.topic} group_id={args.group_id} "
            f"offset_reset={args.offset_reset} commits=disabled",
            flush=True,
        )
        try:
            forward_records(
                process,
                connections,
                args.topic,
                args.stats_every,
                stats,
            )
        except KeyboardInterrupt:
            interrupted = True
        if not interrupted:
            return_code = process.wait(timeout=5.0)
            raise RuntimeError(f"kcat consumer exited unexpectedly: {return_code}")
    finally:
        if process is not None:
            stop_consumer(process)
        close_outputs(connections)
        print_stats(stats)
        print(
            f"kafka-bridge: stopped=1 interrupted={int(interrupted)}",
            flush=True,
        )
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--group-id", default="rtppp-pi-sk068")
    parser.add_argument(
        "--offset-reset",
        choices=("latest", "earliest"),
        default="latest",
        help="initial position only when this dedicated group has no committed offset",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--obs-port", default=23001, type=int)
    parser.add_argument("--sino-port", default=23002, type=int)
    parser.add_argument("--stats-every", default=1000, type=int)
    parser.add_argument("--kcat", default="kcat")
    parser.add_argument(
        "--print-command",
        action="store_true",
        help="print the kcat argv without opening TCP or Kafka",
    )
    args = parser.parse_args(argv)
    if not args.broker.strip() or not args.topic.strip() or not args.group_id.strip():
        parser.error("broker, topic and group-id must be non-empty")
    if not 1 <= args.obs_port <= 65535 or not 1 <= args.sino_port <= 65535:
        parser.error("TCP ports must be in 1..65535")
    if args.obs_port == args.sino_port:
        parser.error("observation and Sino ports must differ")
    if args.stats_every < 0:
        parser.error("--stats-every must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.print_command:
        print(shlex.join(build_kcat_command(args)))
        return 0
    try:
        return run_live(args)
    except KeyboardInterrupt:
        print("kafka-bridge: stopped=1 interrupted=1", flush=True)
        return 0
    except (FixtureError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"kafka-bridge: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
