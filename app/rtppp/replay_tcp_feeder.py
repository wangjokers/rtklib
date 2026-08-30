#!/usr/bin/env python3
"""Replay two RTKLIB time-tagged files through localhost TCP servers."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path


TAG_HEADER_SIZE = 64
TAG_TIME_SIZE = 12
TAG_RECORD_SIZE = 8


@dataclass(frozen=True)
class TaggedInput:
    name: str
    data: bytes
    tick_f: int
    records: tuple[tuple[int, int], ...]
    sha256: str
    start_gpst: str


def load_tagged_input(path: Path) -> TaggedInput:
    data = path.read_bytes()
    tag_path = Path(f"{path}.tag")
    tag = tag_path.read_bytes()
    first_record = TAG_HEADER_SIZE + TAG_TIME_SIZE
    if len(tag) < first_record or (len(tag) - first_record) % TAG_RECORD_SIZE:
        raise ValueError(f"invalid RTKLIB tag size: {tag_path}")
    if not tag.startswith(b"TIMETAG"):
        raise ValueError(f"invalid RTKLIB tag header: {tag_path}")
    tick_f = struct.unpack_from("<I", tag, TAG_HEADER_SIZE - 4)[0]
    start_seconds = struct.unpack_from("<I", tag, TAG_HEADER_SIZE)[0]
    start_fraction = struct.unpack_from("<d", tag, TAG_HEADER_SIZE + 4)[0]
    records = tuple(
        struct.unpack_from("<II", tag, offset)
        for offset in range(first_record, len(tag), TAG_RECORD_SIZE)
    )
    if not records:
        raise ValueError(f"empty RTKLIB tag: {tag_path}")
    previous_tick = previous_position = 0
    for tick, position in records:
        if tick < previous_tick:
            raise ValueError(f"backwards tag tick in {tag_path}")
        if position < previous_position or position > len(data):
            raise ValueError(f"invalid tag file position in {tag_path}")
        previous_tick, previous_position = tick, position
    if records[-1][1] != len(data):
        raise ValueError(f"final tag position does not equal file size: {tag_path}")
    return TaggedInput(
        path.name,
        data,
        tick_f,
        records,
        hashlib.sha256(data).hexdigest().upper(),
        (
            datetime.datetime.fromtimestamp(
                start_seconds, datetime.timezone.utc
            )
            + datetime.timedelta(seconds=start_fraction)
        ).strftime("%Y %m %d %H %M %S.%f"),
    )


def open_listener(host: str, port: int) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    return listener


def replay_stream(
    connection: socket.socket,
    source: TaggedInput,
    master_tick_f: int,
    start: float,
    speed: float,
    result: dict[str, int],
    errors: list[BaseException],
) -> None:
    position = 0
    try:
        for tick, next_position in source.records:
            due_ms = tick + source.tick_f - master_tick_f
            deadline = start + max(0, due_ms) / (speed * 1000.0)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.05))
            chunk = source.data[position:next_position]
            if chunk:
                connection.sendall(chunk)
            position = next_position
        result[source.name] = position
        connection.shutdown(socket.SHUT_WR)
    except BaseException as exc:  # Report worker failures in the main thread.
        errors.append(exc)
    finally:
        connection.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay unmodified fixture bytes over two time-tagged TCP streams."
    )
    parser.add_argument("--obs", required=True, type=Path)
    parser.add_argument("--sino", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--obs-port", default=23001, type=int)
    parser.add_argument("--sino-port", default=23002, type=int)
    parser.add_argument("--speed", default=1.0, type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be positive")
    obs = load_tagged_input(args.obs)
    sino = load_tagged_input(args.sino)
    if obs.start_gpst != sino.start_gpst:
        raise SystemExit("obs and sino tag files have different GPST starts")
    obs_listener = open_listener(args.host, args.obs_port)
    sino_listener = open_listener(args.host, args.sino_port)
    print(
        f"feeder: listening obs={args.host}:{args.obs_port} "
        f"sino={args.host}:{args.sino_port} speed={args.speed:g} "
        f"start_gpst=\"{obs.start_gpst}\"",
        flush=True,
    )
    try:
        obs_connection, _ = obs_listener.accept()
        sino_connection, _ = sino_listener.accept()
    finally:
        obs_listener.close()
        sino_listener.close()
    print("feeder: both clients connected", flush=True)

    start = time.monotonic() + 0.05
    result: dict[str, int] = {}
    errors: list[BaseException] = []
    threads = [
        threading.Thread(
            target=replay_stream,
            args=(obs_connection, obs, obs.tick_f, start, args.speed, result, errors),
            name="obs-feeder",
        ),
        threading.Thread(
            target=replay_stream,
            args=(sino_connection, sino, obs.tick_f, start, args.speed, result, errors),
            name="sino-feeder",
        ),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    if errors:
        raise errors[0]
    print(
        f"feeder: completed obs_bytes={result[obs.name]}/{len(obs.data)} "
        f"sino_bytes={result[sino.name]}/{len(sino.data)}",
        flush=True,
    )
    print(f"feeder: obs_sha256={obs.sha256}", flush=True)
    print(f"feeder: sino_sha256={sino.sha256}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
