#!/usr/bin/env python3
"""PPP-B2b 实验的只读预检与结果汇总工具。

本程序绝不修改 RTKLIB 输入。PowerShell 总控脚本会为每个生成文件提供
新建实验目录内的路径。本程序只依赖 Python 标准库。
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import mmap
import os
from pathlib import Path
import re
import statistics
import sys
import zlib


SCHEMA_VERSION = "1.0"
GPS_EPOCH = dt.datetime(1980, 1, 6)
Q_NAMES = {
    1: "fix",
    2: "float",
    3: "sbas",
    4: "dgps",
    5: "single",
    6: "ppp",
}


class ChineseArgumentParser(argparse.ArgumentParser):
    """将 argparse 的固定帮助标题本地化，不改变参数名和调用接口。"""

    @staticmethod
    def _localize_help(text: str) -> str:
        replacements = {
            "usage: ": "用法：",
            "positional arguments:": "位置参数：",
            "optional arguments:": "选项：",
            "options:": "选项：",
            "show this help message and exit": "显示本帮助信息并退出",
        }
        for source, translated in replacements.items():
            text = text.replace(source, translated)
        return text

    def format_help(self) -> str:
        return self._localize_help(super().format_help())

    def format_usage(self) -> str:
        return self._localize_help(super().format_usage())


def iso_time(value: dt.datetime | None) -> str | None:
    return value.isoformat(timespec="milliseconds") if value else None


def parse_time(value: str | None) -> dt.datetime | None:
    if not value:
        return None
    text = value.strip().replace("T", " ")
    for fmt in (
        "%Y/%m/%d %H:%M:%S.%f",
        "%Y/%m/%d %H:%M:%S",
        "%Y-%m-%d %H:%M:%S.%f",
        "%Y-%m-%d %H:%M:%S",
    ):
        try:
            return dt.datetime.strptime(text, fmt)
        except ValueError:
            pass
    raise ValueError(f"不支持的时间格式：{value!r}")


def epoch_from_fields(fields: list[str]) -> dt.datetime | None:
    try:
        year, month, day, hour, minute = (int(fields[i]) for i in range(5))
        seconds = float(fields[5])
        base = dt.datetime(year, month, day, hour, minute)
        return base + dt.timedelta(seconds=seconds)
    except (ValueError, IndexError, OverflowError):
        return None


def gps_week_time(week: int, tow_seconds: float) -> dt.datetime | None:
    if week <= 0 or not math.isfinite(tow_seconds):
        return None
    try:
        return GPS_EPOCH + dt.timedelta(weeks=week, seconds=tow_seconds)
    except OverflowError:
        return None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def file_record(role: str, path: Path, generated: bool = False) -> dict:
    stat = path.stat()
    return {
        "role": role,
        "path": str(path.resolve()),
        "size_bytes": stat.st_size,
        "sha256": sha256_file(path),
        "generated": generated,
    }


def inspect_config_references(path: Path) -> list[dict]:
    """Return non-empty file-* inputs from an RTKLIB configuration."""
    output_only = {"file-solstatfile", "file-tracefile", "file-tempdir"}
    references = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(r"^\s*(file-[^=\s]+)\s*=\s*(.*?)\s*(?:#.*)?$", line)
        if not match or match.group(1) in output_only:
            continue
        key, value = match.group(1), os.path.expandvars(match.group(2).strip())
        if not value:
            continue
        candidate = Path(value)
        candidates = [candidate] if candidate.is_absolute() else [Path.cwd() / candidate, path.parent / candidate]
        resolved = next((item.resolve() for item in candidates if item.is_file()), candidates[0].resolve())
        references.append({"key": key, "path": str(resolved), "exists": resolved.is_file()})
    return references


def _update_range(current: tuple[dt.datetime | None, dt.datetime | None],
                  value: dt.datetime | None) -> tuple[dt.datetime | None, dt.datetime | None]:
    first, last = current
    if value is None:
        return first, last
    return value if first is None or value < first else first, value if last is None or value > last else last


def inspect_rinex_observation(path: Path, start: dt.datetime | None,
                              end: dt.datetime | None) -> dict:
    version = None
    signals: dict[str, list[str]] = {}
    expected_counts: dict[str, int] = {}
    current_system = None
    approximate_xyz = None
    header_first = None
    header_last = None
    interval = None
    in_header = True
    epoch_range: tuple[dt.datetime | None, dt.datetime | None] = (None, None)
    epoch_count = 0
    window_epoch_count = 0

    with path.open("r", encoding="ascii", errors="replace", newline="") as stream:
        for line_no, line in enumerate(stream, 1):
            if line_no == 1:
                try:
                    version = float(line[:9].strip())
                except ValueError:
                    version = None
            if in_header:
                label = line[60:].strip() if len(line) >= 60 else ""
                if label == "SYS / # / OBS TYPES":
                    if line and not line[0].isspace():
                        current_system = line[0]
                        signals.setdefault(current_system, [])
                        try:
                            expected_counts[current_system] = int(line[3:6])
                        except ValueError:
                            pass
                    if current_system:
                        signals[current_system].extend(line[7:60].split())
                elif label == "APPROX POSITION XYZ":
                    try:
                        approximate_xyz = [float(value) for value in line[:60].split()[:3]]
                    except ValueError:
                        approximate_xyz = None
                elif label in ("TIME OF FIRST OBS", "TIME OF LAST OBS"):
                    value = epoch_from_fields(line[:43].split())
                    if label == "TIME OF FIRST OBS":
                        header_first = value
                    else:
                        header_last = value
                elif label == "INTERVAL":
                    try:
                        interval = float(line[:10])
                    except ValueError:
                        interval = None
                if "END OF HEADER" in line:
                    in_header = False
                continue

            value = None
            if line.startswith(">"):
                value = epoch_from_fields(line[1:].split()[:6])
            elif version is not None and version < 3.0:
                match = re.match(
                    r"^\s*(\d{2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+([0-9.]+)",
                    line,
                )
                if match:
                    fields = list(match.groups())
                    year = int(fields[0])
                    fields[0] = str(2000 + year if year < 80 else 1900 + year)
                    value = epoch_from_fields(fields)
            if value is None:
                continue
            epoch_range = _update_range(epoch_range, value)
            epoch_count += 1
            if (start is None or value >= start) and (end is None or value <= end):
                window_epoch_count += 1

    for system, count in expected_counts.items():
        signals[system] = signals.get(system, [])[:count]
    first = epoch_range[0] or header_first
    last = epoch_range[1] or header_last
    return {
        "path": str(path.resolve()),
        "version": version,
        "time_start_gpst": iso_time(first),
        "time_end_gpst": iso_time(last),
        "header_time_start_gpst": iso_time(header_first),
        "header_time_end_gpst": iso_time(header_last),
        "interval_seconds": interval,
        "epoch_count": epoch_count,
        "window_epoch_count": window_epoch_count,
        "signals": signals,
        "approx_position_xyz_m": approximate_xyz,
    }


def inspect_rinex_navigation(path: Path) -> dict:
    version = None
    in_header = True
    pending_eph = False
    nav_types: dict[str, set[str]] = {}
    record_count = 0
    epoch_range: tuple[dt.datetime | None, dt.datetime | None] = (None, None)

    with path.open("r", encoding="ascii", errors="replace", newline="") as stream:
        for line_no, line in enumerate(stream, 1):
            if line_no == 1:
                try:
                    version = float(line[:9].strip())
                except ValueError:
                    version = None
            if in_header:
                if "END OF HEADER" in line:
                    in_header = False
                continue

            if line.startswith(">"):
                tokens = line.split()
                pending_eph = len(tokens) >= 4 and tokens[1] == "EPH"
                if pending_eph:
                    satellite = tokens[2]
                    subtype = tokens[3]
                    nav_types.setdefault(satellite[0], set()).add(subtype)
                continue

            value = None
            if pending_eph:
                match = re.match(
                    r"^([A-Z]\d{2})\s+(\d{4})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2}(?:\.\d*)?)",
                    line,
                )
                if match:
                    _, *fields = match.groups()
                    value = epoch_from_fields(fields)
                    record_count += 1
                pending_eph = False
            elif version is not None and version < 4.0:
                match = re.match(r"^([A-Z]\d{2})\s+(\d{4})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+([0-9.]+)", line)
                if match:
                    satellite, *fields = match.groups()
                    nav_types.setdefault(satellite[0], set()).add("LEGACY")
                    value = epoch_from_fields(fields)
                    record_count += 1
            epoch_range = _update_range(epoch_range, value)

    return {
        "path": str(path.resolve()),
        "version": version,
        "time_start_gpst": iso_time(epoch_range[0]),
        "time_end_gpst": iso_time(epoch_range[1]),
        "record_count": record_count,
        "ephemeris_types": {key: sorted(values) for key, values in sorted(nav_types.items())},
    }


def _rtk_crc32(data: bytes | mmap.mmap) -> int:
    return (zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF


def _get_msb_bits(data: mmap.mmap, bit_offset: int, bit_length: int) -> int:
    value = 0
    for bit in range(bit_offset, bit_offset + bit_length):
        byte = data[bit // 8]
        value = (value << 1) | ((byte >> (7 - bit % 8)) & 1)
    return value


def _scan_sino(mapping: mmap.mmap) -> dict:
    sync = b"\xAA\x44\x12"
    pos = 0
    structural = 0
    crc_errors = 0
    message_1697 = 0
    counts = {"MASK": 0, "ORBIT_URAI": 0, "DIFF_CODE_BIAS": 0, "CLOCK": 0}
    type_names = {1: "MASK", 2: "ORBIT_URAI", 3: "DIFF_CODE_BIAS", 4: "CLOCK"}
    epoch_range: tuple[dt.datetime | None, dt.datetime | None] = (None, None)

    while True:
        found = mapping.find(sync, pos)
        if found < 0:
            break
        if found + 10 > len(mapping):
            break
        if mapping[found + 3] != 28:
            pos = found + 1
            continue
        payload_length = int.from_bytes(mapping[found + 8:found + 10], "little")
        frame_length = 28 + payload_length + 4
        if payload_length < 0 or frame_length > 65540 or found + frame_length > len(mapping):
            pos = found + 1
            continue
        structural += 1
        body_end = found + frame_length - 4
        stored_crc = int.from_bytes(mapping[body_end:body_end + 4], "little")
        valid_crc = _rtk_crc32(mapping[found:body_end]) == stored_crc
        if not valid_crc:
            crc_errors += 1
        message = int.from_bytes(mapping[found + 4:found + 6], "little")
        week = int.from_bytes(mapping[found + 14:found + 16], "little")
        tow = int.from_bytes(mapping[found + 16:found + 20], "little") * 0.001
        epoch_range = _update_range(epoch_range, gps_week_time(week, tow))
        if valid_crc and message == 1697 and payload_length >= 7:
            message_1697 += 1
            message_type = _get_msb_bits(mapping, (found + 28) * 8 + 44, 6)
            if message_type in type_names:
                counts[type_names[message_type]] += 1
        pos = found + frame_length

    return {
        "format": "sino",
        "structural_frame_count": structural,
        "crc_error_count": crc_errors,
        "b2b_message_count": message_1697,
        "product_frame_counts": counts,
        "time_start_gpst": iso_time(epoch_range[0]),
        "time_end_gpst": iso_time(epoch_range[1]),
    }


def _scan_unicore(mapping: mmap.mmap) -> dict:
    sync = b"\xAA\x44\xB5"
    pos = 0
    structural = 0
    crc_errors = 0
    counts = {"MASK": 0, "ORBIT_URAI": 0, "DIFF_CODE_BIAS": 0, "CLOCK": 0}
    message_names = {2302: "MASK", 2304: "ORBIT_URAI", 2306: "DIFF_CODE_BIAS", 2308: "CLOCK"}
    epoch_range: tuple[dt.datetime | None, dt.datetime | None] = (None, None)

    while True:
        found = mapping.find(sync, pos)
        if found < 0:
            break
        if found + 24 > len(mapping):
            break
        if mapping[found + 3] != 24:
            pos = found + 1
            continue
        payload_length = int.from_bytes(mapping[found + 6:found + 8], "little")
        frame_length = 24 + payload_length + 4
        if frame_length > 65540 or found + frame_length > len(mapping):
            pos = found + 1
            continue
        structural += 1
        body_end = found + frame_length - 4
        stored_crc = int.from_bytes(mapping[body_end:body_end + 4], "little")
        valid_crc = _rtk_crc32(mapping[found:body_end]) == stored_crc
        if not valid_crc:
            crc_errors += 1
        message = int.from_bytes(mapping[found + 4:found + 6], "little")
        week = int.from_bytes(mapping[found + 10:found + 12], "little")
        tow = int.from_bytes(mapping[found + 12:found + 16], "little") * 0.001
        epoch_range = _update_range(epoch_range, gps_week_time(week, tow))
        if valid_crc and message in message_names:
            counts[message_names[message]] += 1
        pos = found + frame_length

    return {
        "format": "unicore",
        "structural_frame_count": structural,
        "crc_error_count": crc_errors,
        "b2b_message_count": sum(counts.values()),
        "product_frame_counts": counts,
        "time_start_gpst": iso_time(epoch_range[0]),
        "time_end_gpst": iso_time(epoch_range[1]),
    }


def inspect_b2b(path: Path, requested_format: str) -> dict:
    with path.open("rb") as stream:
        with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as mapping:
            if requested_format == "auto":
                sample = mapping[:min(len(mapping), 1024 * 1024)]
                sino_hits = sample.count(b"\xAA\x44\x12")
                unicore_hits = sample.count(b"\xAA\x44\xB5")
                if sino_hits == 0 and unicore_hits == 0:
                    return {
                        "path": str(path.resolve()),
                        "format": "unknown",
                        "structural_frame_count": 0,
                        "crc_error_count": None,
                        "b2b_message_count": 0,
                        "product_frame_counts": {name: 0 for name in ("MASK", "ORBIT_URAI", "DIFF_CODE_BIAS", "CLOCK")},
                        "time_start_gpst": None,
                        "time_end_gpst": None,
                    }
                requested_format = "sino" if sino_hits >= unicore_hits else "unicore"
            result = _scan_sino(mapping) if requested_format == "sino" else _scan_unicore(mapping)
    result["path"] = str(path.resolve())
    result["all_required_products_present"] = all(
        result["product_frame_counts"].get(name, 0) > 0
        for name in ("MASK", "ORBIT_URAI", "DIFF_CODE_BIAS", "CLOCK")
    )
    return result


def inspect_precise_text(path: Path, kind: str) -> dict:
    epoch_range: tuple[dt.datetime | None, dt.datetime | None] = (None, None)
    count = 0
    with path.open("r", encoding="ascii", errors="replace", newline="") as stream:
        for line in stream:
            value = None
            if kind == "sp3" and line.startswith("*"):
                value = epoch_from_fields(line[1:].split()[:6])
            elif kind == "clk" and (line.startswith("AS ") or line.startswith("AR ")):
                tokens = line.split()
                if len(tokens) >= 8:
                    value = epoch_from_fields(tokens[2:8])
            if value is not None:
                count += 1
                epoch_range = _update_range(epoch_range, value)
    return {
        "path": str(path.resolve()),
        "time_start_gpst": iso_time(epoch_range[0]),
        "time_end_gpst": iso_time(epoch_range[1]),
        "epoch_record_count": count,
    }


def ranges_overlap(first_start: str | None, first_end: str | None,
                   second_start: str | None, second_end: str | None) -> bool | None:
    if not all((first_start, first_end, second_start, second_end)):
        return None
    a0, a1 = dt.datetime.fromisoformat(first_start), dt.datetime.fromisoformat(first_end)
    b0, b1 = dt.datetime.fromisoformat(second_start), dt.datetime.fromisoformat(second_end)
    return max(a0, b0) <= min(a1, b1)


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def run_preflight(args: argparse.Namespace) -> int:
    paths: dict[str, Path | None] = {
        "observation": Path(args.obs).resolve() if args.obs else None,
        "navigation": Path(args.nav).resolve() if args.nav else None,
        "b2b_raw": Path(args.b2b).resolve() if args.b2b else None,
        "config_template": Path(args.config).resolve() if args.config else None,
        "executable": Path(args.executable).resolve() if args.executable else None,
        "sp3": Path(args.sp3).resolve() if args.sp3 else None,
        "clk": Path(args.clk).resolve() if args.clk else None,
    }
    modes = [mode.upper() for mode in args.modes]
    blocking: list[str] = []
    warnings: list[str] = []
    mode_issues = {mode: [] for mode in modes}
    required_core = ("observation", "navigation", "config_template", "executable")
    for role in required_core:
        path = paths[role]
        if path is None or not path.is_file():
            blocking.append(f"缺少必需的 {role} 路径：{path or '未指定'}")
    if any(mode in ("M1", "M2") for mode in modes):
        path = paths["b2b_raw"]
        if path is None or not path.is_file():
            for mode in modes:
                if mode in ("M1", "M2"):
                    mode_issues[mode].append("缺少 B2b raw 输入")
    if "M3" in modes:
        for role in ("sp3", "clk"):
            path = paths[role]
            if path is None or not path.is_file():
                mode_issues["M3"].append(f"缺少 {role.upper()} 输入")

    config_references = []
    config_path = paths["config_template"]
    if config_path and config_path.is_file():
        config_references = inspect_config_references(config_path)
        for config_reference in config_references:
            if not config_reference["exists"]:
                blocking.append(
                    f"配置引用的输入路径不存在（{config_reference['key']}）：{config_reference['path']}"
                )

    start = parse_time(args.start)
    end = parse_time(args.end)
    if start and end and start > end:
        blocking.append("实验开始时间晚于结束时间")

    observation = None
    navigation = None
    b2b = None
    precise: dict[str, dict] = {}
    if paths["observation"] and paths["observation"].is_file():
        observation = inspect_rinex_observation(paths["observation"], start, end)
        if observation["version"] is None:
            blocking.append("无法取得观测 RINEX 版本")
        if not observation["time_start_gpst"] or not observation["time_end_gpst"]:
            blocking.append("无法取得观测时间范围")
        if start is None and observation["time_start_gpst"]:
            start = dt.datetime.fromisoformat(observation["time_start_gpst"])
        if end is None and observation["time_end_gpst"]:
            end = dt.datetime.fromisoformat(observation["time_end_gpst"])
        if start and end:
            observation = inspect_rinex_observation(paths["observation"], start, end)
            if observation["window_epoch_count"] == 0:
                blocking.append("请求时间窗内没有观测历元")
    if paths["navigation"] and paths["navigation"].is_file():
        navigation = inspect_rinex_navigation(paths["navigation"])
        if navigation["version"] is None:
            blocking.append("无法取得导航 RINEX 版本")
    if paths["b2b_raw"] and paths["b2b_raw"].is_file():
        b2b = inspect_b2b(paths["b2b_raw"], args.b2b_format)
        if b2b["format"] == "unknown":
            for mode in modes:
                if mode in ("M1", "M2"):
                    mode_issues[mode].append("无法识别 B2b 帧格式")
        if not b2b.get("all_required_products_present", False):
            missing = [name for name, count in b2b["product_frame_counts"].items() if count <= 0]
            for mode in modes:
                if mode in ("M1", "M2"):
                    mode_issues[mode].append("缺少 B2b 产品：" + ", ".join(missing))
        if b2b.get("crc_error_count"):
            warnings.append(f"B2b raw 中有 {b2b['crc_error_count']} 个结构可解析但 CRC 错误的帧")
    for role, kind in (("sp3", "sp3"), ("clk", "clk")):
        path = paths[role]
        if path and path.is_file():
            precise[role] = inspect_precise_text(path, kind)

    overlaps: dict[str, bool | None] = {}
    if observation and navigation:
        overlaps["observation_navigation"] = ranges_overlap(
            observation["time_start_gpst"], observation["time_end_gpst"],
            navigation["time_start_gpst"], navigation["time_end_gpst"],
        )
        if overlaps["observation_navigation"] is False:
            blocking.append("观测与导航时间范围不重叠")
        elif overlaps["observation_navigation"] is None:
            warnings.append("无法确认观测与导航时间范围是否重叠")
    if observation and b2b:
        overlaps["observation_b2b"] = ranges_overlap(
            observation["time_start_gpst"], observation["time_end_gpst"],
            b2b["time_start_gpst"], b2b["time_end_gpst"],
        )
        if overlaps["observation_b2b"] is False:
            for mode in modes:
                if mode in ("M1", "M2"):
                    mode_issues[mode].append("观测与 B2b 时间范围不重叠")
        elif overlaps["observation_b2b"] is None:
            for mode in modes:
                if mode in ("M1", "M2"):
                    mode_issues[mode].append("无法确认观测与 B2b 时间范围是否重叠")
    if observation and "M3" in modes:
        for role in ("sp3", "clk"):
            if role not in precise:
                continue
            key = f"observation_{role}"
            overlaps[key] = ranges_overlap(
                observation["time_start_gpst"], observation["time_end_gpst"],
                precise[role]["time_start_gpst"], precise[role]["time_end_gpst"],
            )
            if overlaps[key] is not True:
                mode_issues["M3"].append(f"无法确认观测与 {role.upper()} 时间范围重叠")

    reference = None
    reference_source = None
    if args.reference_ecef:
        reference = [float(value) for value in args.reference_ecef]
        reference_source = "parameter"
    elif observation and observation.get("approx_position_xyz_m"):
        reference = observation["approx_position_xyz_m"]
        reference_source = "rinex_header_approx_position_xyz"
        warnings.append("参考坐标自动取自 RINEX 的 APPROX POSITION XYZ；它不是经独立认证的真值")
    else:
        blocking.append("无法取得参考 ECEF 坐标")

    input_roles_by_path: dict[str, list[str]] = {}
    for role, path in paths.items():
        if path and path.is_file():
            input_roles_by_path.setdefault(str(path.resolve()), []).append(role)
    for config_reference in config_references:
        if config_reference["exists"]:
            input_roles_by_path.setdefault(config_reference["path"], []).append(
                f"config:{config_reference['key']}"
            )
    inputs = []
    for path_text, roles in input_roles_by_path.items():
        inputs.append(file_record(",".join(sorted(set(roles))), Path(path_text)))

    result = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "modes": modes,
        "requested_window": {
            "start_gpst": iso_time(start),
            "end_gpst": iso_time(end),
        },
        "blocking_issues": blocking,
        "mode_issues": mode_issues,
        "warnings": warnings,
        "observation": observation,
        "navigation": navigation,
        "b2b": b2b,
        "precise_products": precise,
        "overlaps": overlaps,
        "reference": {
            "ecef_xyz_m": reference,
            "source": reference_source,
        },
        "config_referenced_inputs": config_references,
        "inputs": inputs,
    }
    write_json(Path(args.output), result)
    return 0


def ecef_reference_lat_lon(xyz: list[float]) -> tuple[float, float]:
    x, y, z = xyz
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(12):
        sin_lat = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-15) - n
        updated = math.atan2(z, p * (1.0 - e2 * n / (n + height)))
        if abs(updated - lat) < 1e-14:
            lat = updated
            break
        lat = updated
    return lat, lon


def ecef_delta_to_enu(xyz: list[float], reference: list[float]) -> tuple[float, float, float]:
    lat, lon = ecef_reference_lat_lon(reference)
    dx, dy, dz = (xyz[i] - reference[i] for i in range(3))
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    east = -slon * dx + clon * dy
    north = -slat * clon * dx - slat * slon * dy + clat * dz
    up = clat * clon * dx + clat * slon * dy + slat * dz
    return east, north, up


def percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * probability
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def rms(values: list[float]) -> float | None:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def parse_xyz_solution(path: Path) -> tuple[list[dict], str | None]:
    rows = []
    header_format = None
    if not path.is_file():
        return rows, header_format
    with path.open("r", encoding="ascii", errors="replace") as stream:
        for line in stream:
            if line.startswith("%"):
                if "x-ecef(m)" in line:
                    header_format = "xyz"
                elif "latitude(deg)" in line:
                    header_format = "llh"
                continue
            tokens = line.split()
            if len(tokens) < 7:
                continue
            when = parse_time(f"{tokens[0]} {tokens[1]}")
            try:
                xyz = [float(tokens[2]), float(tokens[3]), float(tokens[4])]
                quality = int(tokens[5])
                satellites = int(tokens[6])
            except (ValueError, IndexError):
                continue
            rows.append({"time": when, "xyz": xyz, "quality": quality, "satellites": satellites})
    return rows, header_format


def read_log_text(paths: list[Path]) -> str:
    parts = []
    for path in paths:
        if path.is_file():
            parts.append(path.read_text(encoding="utf-8", errors="replace"))
    return "\n".join(parts)


def diagnostic_count(text: str, pattern: str) -> tuple[int | None, str]:
    count = len(re.findall(pattern, text, flags=re.IGNORECASE))
    return (count, "available_from_log") if count else (None, "unavailable_no_explicit_log_marker")


def empty_metrics(reason: str) -> tuple[dict, dict]:
    names = (
        "rms_e_m", "rms_n_m", "rms_u_m", "error_3d_p68_m",
        "error_3d_p95_m", "error_3d_max_m", "satellite_count_min",
        "satellite_count_mean", "satellite_count_max", "first_convergence_seconds",
    )
    return {name: None for name in names}, {name: reason for name in names}


def run_summarize(args: argparse.Namespace) -> int:
    mode = args.mode.upper()
    status = args.status
    output_path = Path(args.output)
    command = Path(args.command_file).read_text(encoding="utf-8", errors="replace").strip() if args.command_file and Path(args.command_file).is_file() else ""
    metrics, availability = empty_metrics("unavailable")
    result = {
        "schema_version": SCHEMA_VERSION,
        "mode": mode,
        "mode_description": args.mode_description,
        "status": status,
        "block_reason": args.block_reason,
        "process_exit_code": args.exit_code,
        "processing_epoch_count": args.processing_epochs,
        "valid_solution_epoch_count": 0,
        "valid_solution_ratio": None,
        "solution_time_start_gpst": None,
        "solution_time_end_gpst": None,
        "reference_ecef_xyz_m": args.reference_ecef,
        "reference_source": args.reference_source,
        "solution_status_counts": {},
        "solution_status_proportions": {},
        "metrics": metrics,
        "availability": availability,
        "diagnostics": {},
        "paths": {
            "config": str(Path(args.config).resolve()) if args.config else None,
            "solution": str(Path(args.solution).resolve()) if args.solution else None,
            "run_log": str(Path(args.run_log).resolve()) if args.run_log else None,
            "trace": str(Path(args.trace).resolve()) if args.trace else None,
        },
        "config_sha256": sha256_file(Path(args.config)) if args.config and Path(args.config).is_file() else None,
        "reproducible_command": command,
    }
    if status == "blocked":
        for name in result["availability"]:
            result["availability"][name] = "blocked"
        for name in ("b2b_product_missing_count", "b2b_product_expired_count", "b2b_iod_mismatch_count", "b2b_orbit_clock_not_ready_count", "b2b_code_bias_not_ready_count"):
            result["diagnostics"][name] = None
            result["availability"][name] = "blocked"
        write_json(output_path, result)
        return 0

    log_paths = [Path(args.run_log)] if args.run_log else []
    if args.trace:
        log_paths.append(Path(args.trace))
    text = read_log_text(log_paths)
    explicit_errors = re.findall(r"(?:^|[\r\n])\s*error\s*:\s*([^\r\n]+)", text, flags=re.IGNORECASE)

    solution_path = Path(args.solution)
    rows, header_format = parse_xyz_solution(solution_path)
    result["solution_format"] = header_format
    if explicit_errors:
        result["status"] = "failed"
        result["block_reason"] = "主程序报错：" + "; ".join(item.strip() for item in explicit_errors[:5])
        for name in result["availability"]:
            result["availability"][name] = "unavailable_main_program_error"
    elif header_format != "xyz":
        result["status"] = "failed"
        result["block_reason"] = "解文件不是可提取的 XYZ 格式"
        for name in result["availability"]:
            result["availability"][name] = "unavailable_solution_format"
    elif args.exit_code not in (0, None):
        result["status"] = "failed"
        result["block_reason"] = f"主程序退出码为 {args.exit_code}"
    elif not rows:
        result["status"] = "completed_no_solution"
        for name in result["availability"]:
            result["availability"][name] = "unavailable_no_valid_solution"
    else:
        result["status"] = "completed"
        result["valid_solution_epoch_count"] = len(rows)
        result["valid_solution_ratio"] = len(rows) / args.processing_epochs if args.processing_epochs else None
        result["solution_time_start_gpst"] = iso_time(rows[0]["time"])
        result["solution_time_end_gpst"] = iso_time(rows[-1]["time"])
        counts: dict[int, int] = {}
        for row in rows:
            counts[row["quality"]] = counts.get(row["quality"], 0) + 1
        result["solution_status_counts"] = {f"Q{key}_{Q_NAMES.get(key, 'unknown')}": value for key, value in sorted(counts.items())}
        result["solution_status_proportions"] = {key: value / len(rows) for key, value in result["solution_status_counts"].items()}

        if args.reference_ecef:
            enu = [ecef_delta_to_enu(row["xyz"], args.reference_ecef) for row in rows]
            east = [value[0] for value in enu]
            north = [value[1] for value in enu]
            up = [value[2] for value in enu]
            error_3d = [math.sqrt(e * e + n * n + u * u) for e, n, u in enu]
            result["metrics"].update({
                "rms_e_m": rms(east),
                "rms_n_m": rms(north),
                "rms_u_m": rms(up),
                "error_3d_p68_m": percentile(error_3d, 0.68),
                "error_3d_p95_m": percentile(error_3d, 0.95),
                "error_3d_max_m": max(error_3d),
            })
            for name in ("rms_e_m", "rms_n_m", "rms_u_m", "error_3d_p68_m", "error_3d_p95_m", "error_3d_max_m"):
                result["availability"][name] = "available"

            threshold = args.convergence_threshold
            hold = args.convergence_hold
            interval = args.observation_interval
            convergence_index = None
            for index in range(0, len(rows) - hold + 1):
                window_errors = error_3d[index:index + hold]
                window_rows = rows[index:index + hold]
                gaps_ok = True
                if interval and interval > 0:
                    gaps_ok = all(
                        (window_rows[j]["time"] - window_rows[j - 1]["time"]).total_seconds() <= interval * 1.5
                        for j in range(1, len(window_rows))
                    )
                if gaps_ok and all(value <= threshold for value in window_errors):
                    convergence_index = index
                    break
            if convergence_index is None:
                result["availability"]["first_convergence_seconds"] = "not_reached"
            else:
                experiment_start = parse_time(args.experiment_start)
                result["metrics"]["first_convergence_seconds"] = (
                    rows[convergence_index]["time"] - experiment_start
                ).total_seconds() if experiment_start else None
                result["availability"]["first_convergence_seconds"] = "available" if experiment_start else "unavailable_experiment_start"
        else:
            for name in ("rms_e_m", "rms_n_m", "rms_u_m", "error_3d_p68_m", "error_3d_p95_m", "error_3d_max_m", "first_convergence_seconds"):
                result["availability"][name] = "unavailable_reference"

        satellites = [row["satellites"] for row in rows]
        result["metrics"].update({
            "satellite_count_min": min(satellites),
            "satellite_count_mean": statistics.fmean(satellites),
            "satellite_count_max": max(satellites),
        })
        for name in ("satellite_count_min", "satellite_count_mean", "satellite_count_max"):
            result["availability"][name] = "available"

    diagnostic_patterns = {
        "b2b_orbit_clock_not_ready_count": r"b2b orbit/clock not ready",
        "b2b_code_bias_not_ready_count": r"b2b code bias not ready",
        "b2b_product_missing_count": r"(?:b2b[^\r\n]*(?:missing|not found)|no b2b broadcast ephemeris)",
        "b2b_product_expired_count": r"b2b[^\r\n]*(?:expired|age (?:invalid|out of range))",
        "b2b_iod_mismatch_count": r"(?:b2b[^\r\n]*iod[^\r\n]*mismatch|iod[^\r\n]*mismatch[^\r\n]*b2b)",
    }
    for name, pattern in diagnostic_patterns.items():
        if mode in ("M0", "M3"):
            result["diagnostics"][name] = None
            result["availability"][name] = "not_applicable"
        else:
            count, reason = diagnostic_count(text, pattern)
            result["diagnostics"][name] = count
            result["availability"][name] = reason

    write_json(output_path, result)
    return 0


CSV_COLUMNS = [
    "mode", "status", "process_exit_code", "processing_epoch_count",
    "valid_solution_epoch_count", "valid_solution_ratio", "first_convergence_seconds",
    "rms_e_m", "rms_n_m", "rms_u_m", "error_3d_p68_m", "error_3d_p95_m",
    "error_3d_max_m", "satellite_count_min", "satellite_count_mean",
    "satellite_count_max", "solution_status_proportions",
    "b2b_product_missing_count", "b2b_product_expired_count",
    "b2b_iod_mismatch_count", "b2b_orbit_clock_not_ready_count",
    "b2b_code_bias_not_ready_count", "block_reason",
]


def display_value(mode: dict, name: str) -> object:
    if name in mode.get("metrics", {}):
        value = mode["metrics"][name]
    elif name in mode.get("diagnostics", {}):
        value = mode["diagnostics"][name]
    else:
        value = mode.get(name)
    if value is None and name == "block_reason":
        return ""
    if value is None:
        return mode.get("availability", {}).get(name, "unavailable")
    if isinstance(value, dict):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return value


def markdown_value(mode: dict, name: str, digits: int = 3) -> str:
    value = display_value(mode, name)
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    if isinstance(value, str):
        labels = {
            "blocked": "阻塞（`blocked`）",
            "not_reached": "未达到（`not_reached`）",
            "unavailable": "不可用（`unavailable`）",
        }
        if value in labels:
            return labels[value]
    return str(value)


OVERALL_STATUS_ZH = {
    "BLOCKED": "阻塞",
    "COMPLETE": "全部完成",
    "PARTIAL": "部分完成",
    "FAILED": "失败",
}

MODE_STATUS_ZH = {
    "blocked": "阻塞",
    "completed": "完成",
    "completed_no_solution": "完成但无有效解",
    "failed": "失败",
    "pending": "等待运行",
    "ran": "已运行",
}

REFERENCE_SOURCE_ZH = {
    "parameter": "参数指定",
    "rinex_header_approx_position_xyz": "RINEX 头近似坐标",
}

LEGACY_TEXT_ZH = {
    "The current main program exposes no independent configuration switch to disable PPP-B2b code bias while retaining B2b orbit/clock. M2 cannot be relabeled as M1.":
        "当前主程序没有在保留 PPP-B2b 轨道/钟差的同时独立关闭码偏差的配置开关；不得把 M2 改名冒充 M1。",
}


def status_with_machine_value(value: str, labels: dict[str, str]) -> str:
    label = labels.get(value, value)
    return f"{label}（`{value}`）" if label != value else f"`{value}`"


def human_text(value: object) -> str:
    text = "" if value is None else str(value)
    for source, translated in LEGACY_TEXT_ZH.items():
        text = text.replace(source, translated)
    if text.startswith("# BLOCKED:"):
        text = "# 阻塞（BLOCKED）：" + text[len("# BLOCKED:"):].lstrip()
    return text


def exit_code_text(mode: dict) -> str:
    value = mode.get("process_exit_code")
    return "未运行" if value is None else str(value)


def run_aggregate(args: argparse.Namespace) -> int:
    experiment_dir = Path(args.experiment_dir).resolve()
    preflight = json.loads(Path(args.preflight).read_text(encoding="utf-8"))
    modes = [json.loads(Path(path).read_text(encoding="utf-8")) for path in args.mode_result]
    completed = [mode for mode in modes if mode["status"] in ("completed", "completed_no_solution")]
    blocked = [mode for mode in modes if mode["status"] == "blocked"]
    failed = [mode for mode in modes if mode["status"] == "failed"]
    if preflight.get("blocking_issues"):
        overall = "BLOCKED"
    elif len(completed) == len(modes):
        overall = "COMPLETE"
    elif completed:
        overall = "PARTIAL"
    elif failed:
        overall = "FAILED"
    else:
        overall = "BLOCKED"

    summary = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "experiment_directory": str(experiment_dir),
        "overall_status": overall,
        "preflight": preflight,
        "modes": modes,
    }
    write_json(experiment_dir / "summary.json", summary)

    with (experiment_dir / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for mode in modes:
            writer.writerow({name: display_value(mode, name) for name in CSV_COLUMNS})

    with (experiment_dir / "inputs_sha256.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("role", "path", "size_bytes", "sha256", "generated"))
        writer.writeheader()
        writer.writerows(preflight.get("inputs", []))

    obs = preflight.get("observation") or {}
    nav = preflight.get("navigation") or {}
    b2b = preflight.get("b2b") or {}
    products = b2b.get("product_frame_counts") or {}
    signals = obs.get("signals") or {}
    signal_text = "; ".join(f"{system}: {' '.join(values)}" for system, values in signals.items()) or "unavailable"
    known_nav_types = {"LNAV", "FDMA", "INAV", "FNAV", "CNV1", "CNV2", "CNV3", "D1", "D2", "SBAS", "LEGACY"}
    nav_parts = []
    for system, values in (nav.get("ephemeris_types") or {}).items():
        known = [value for value in values if value in known_nav_types]
        other_count = len(values) - len(known)
        text = ",".join(known) if known else "无常规类型"
        if other_count:
            text += f"，其他类型 {other_count} 种（详见 `preflight.json`）"
        nav_parts.append(f"{system}: {text}")
    nav_text = "；".join(nav_parts) or "unavailable"
    report = [
        "# PPP-B2b 实验报告",
        "",
        f"- 总体状态：**{status_with_machine_value(overall, OVERALL_STATUS_ZH)}**",
        f"- 实验目录：`{experiment_dir}`",
        f"- 时间窗（GPST）：`{preflight.get('requested_window', {}).get('start_gpst')}` 至 `{preflight.get('requested_window', {}).get('end_gpst')}`",
        f"- 参考 ECEF（m）：`{preflight.get('reference', {}).get('ecef_xyz_m')}`（{REFERENCE_SOURCE_ZH.get(preflight.get('reference', {}).get('source'), preflight.get('reference', {}).get('source'))}）",
        "",
        "## 输入预检",
        "",
        f"- 观测 RINEX：版本 `{obs.get('version')}`，范围 `{obs.get('time_start_gpst')}` 至 `{obs.get('time_end_gpst')}`，时间窗内历元数 `{obs.get('window_epoch_count')}`。",
        f"- 可用观测信号：{signal_text}",
        f"- 导航 RINEX：版本 `{nav.get('version')}`，范围 `{nav.get('time_start_gpst')}` 至 `{nav.get('time_end_gpst')}`。",
        f"- 导航星历类型：{nav_text}",
        f"- B2b raw：格式 `{b2b.get('format', '未请求')}`，范围 `{b2b.get('time_start_gpst')}` 至 `{b2b.get('time_end_gpst')}`，CRC 错误 `{b2b.get('crc_error_count')}`。",
        f"- B2b 产品帧：MASK `{products.get('MASK', 'unavailable')}`，轨道/URAI `{products.get('ORBIT_URAI', 'unavailable')}`，码偏差 `{products.get('DIFF_CODE_BIAS', 'unavailable')}`，钟差 `{products.get('CLOCK', 'unavailable')}`。",
        f"- 时间重叠检查：`{json.dumps(preflight.get('overlaps', {}), ensure_ascii=False, sort_keys=True)}`。",
        "",
        "## 模式汇总",
        "",
        "| 模式 | 状态 | 退出码 | 处理历元 | 有效解 | E/N/U RMS（m） | 三维 P68/P95/最大（m） | 卫星数 最小/平均/最大 | 首次收敛（s） |",
        "|---|---|---:|---:|---:|---|---|---|---|",
    ]
    for mode in modes:
        report.append(
            f"| {mode['mode']} | {status_with_machine_value(mode['status'], MODE_STATUS_ZH)} | {exit_code_text(mode)} | "
            f"{mode.get('processing_epoch_count')} | {mode.get('valid_solution_epoch_count')} | "
            f"{markdown_value(mode, 'rms_e_m')}/{markdown_value(mode, 'rms_n_m')}/{markdown_value(mode, 'rms_u_m')} | "
            f"{markdown_value(mode, 'error_3d_p68_m')}/{markdown_value(mode, 'error_3d_p95_m')}/{markdown_value(mode, 'error_3d_max_m')} | "
            f"{markdown_value(mode, 'satellite_count_min')}/{markdown_value(mode, 'satellite_count_mean', 2)}/{markdown_value(mode, 'satellite_count_max')} | "
            f"{markdown_value(mode, 'first_convergence_seconds')} |"
        )
    report.extend([
        "",
        "## 解状态与 B2b 诊断",
        "",
    ])
    for mode in modes:
        report.extend([
            f"### {mode['mode']}",
            "",
            f"- 解状态比例：`{json.dumps(mode.get('solution_status_proportions', {}), ensure_ascii=False, sort_keys=True)}`",
            f"- 日志明确提供的 B2b 诊断计数：`{json.dumps(mode.get('diagnostics', {}), ensure_ascii=False, sort_keys=True)}`",
            f"- 指标可用性说明：`{json.dumps(mode.get('availability', {}), ensure_ascii=False, sort_keys=True)}`",
            f"- 可复现命令：`{human_text(mode.get('reproducible_command', ''))}`",
            "",
        ])
    blockers = list(preflight.get("blocking_issues", []))
    for mode in modes:
        if mode.get("block_reason"):
            blockers.append(f"{mode['mode']}：{human_text(mode['block_reason'])}")
    report.extend([
        "## 阻塞项与限制",
        "",
    ])
    if blockers:
        report.extend(f"- {item}" for item in blockers)
    else:
        report.append("- 本次运行未报告阻塞项。")
    report.extend([
        "",
        "当前输出或日志没有暴露某项指标时，报告写为 `unavailable`；缺少日志标记时不会推断其计数为 0。",
        "",
    ])
    (experiment_dir / "experiment_report.md").write_text("\n".join(report), encoding="utf-8")

    if blockers or blocked:
        blocked_lines = [
            "# 阻塞报告（BLOCKED）",
            "",
            "至少一个请求模式无法在不修改当前主程序的前提下被真实表示或运行。",
            "",
            "## 最小阻塞项",
            "",
        ]
        blocked_lines.extend(f"- {item}" for item in blockers)
        blocked_lines.extend([
            "",
            "工具没有伪造被阻塞模式的结果。其他独立模式可能已经完成，请查看 `summary.csv` 和 `experiment_report.md`。",
            "",
        ])
        (experiment_dir / "BLOCKED.md").write_text("\n".join(blocked_lines), encoding="utf-8")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = ChineseArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(
        dest="command", required=True, parser_class=ChineseArgumentParser
    )

    preflight = subparsers.add_parser("preflight", help="检查输入并写出预检 JSON")
    preflight.add_argument("--obs", required=True)
    preflight.add_argument("--nav", required=True)
    preflight.add_argument("--b2b")
    preflight.add_argument("--b2b-format", choices=("auto", "unicore", "sino"), default="auto")
    preflight.add_argument("--config", required=True)
    preflight.add_argument("--executable", required=True)
    preflight.add_argument("--sp3")
    preflight.add_argument("--clk")
    preflight.add_argument("--start")
    preflight.add_argument("--end")
    preflight.add_argument("--modes", nargs="+", required=True)
    preflight.add_argument("--reference-ecef", nargs=3, type=float)
    preflight.add_argument("--output", required=True)
    preflight.set_defaults(func=run_preflight)

    summarize = subparsers.add_parser("summarize", help="汇总单个实验模式")
    summarize.add_argument("--mode", required=True)
    summarize.add_argument("--mode-description", required=True)
    summarize.add_argument("--status", choices=("pending", "blocked", "ran"), required=True)
    summarize.add_argument("--block-reason")
    summarize.add_argument("--exit-code", type=int)
    summarize.add_argument("--processing-epochs", type=int, default=0)
    summarize.add_argument("--reference-ecef", nargs=3, type=float)
    summarize.add_argument("--reference-source")
    summarize.add_argument("--experiment-start")
    summarize.add_argument("--observation-interval", type=float)
    summarize.add_argument("--convergence-threshold", type=float, default=0.10)
    summarize.add_argument("--convergence-hold", type=int, default=10)
    summarize.add_argument("--config")
    summarize.add_argument("--solution")
    summarize.add_argument("--run-log")
    summarize.add_argument("--trace")
    summarize.add_argument("--command-file")
    summarize.add_argument("--output", required=True)
    summarize.set_defaults(func=run_summarize)

    aggregate = subparsers.add_parser("aggregate", help="写出实验汇总文件")
    aggregate.add_argument("--experiment-dir", required=True)
    aggregate.add_argument("--preflight", required=True)
    aggregate.add_argument("--mode-result", action="append", required=True)
    aggregate.set_defaults(func=run_aggregate)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.func(args)
    except Exception as exc:  # fail-fast，并报告真实错误
        print(f"pppb2b_validation：{type(exc).__name__}：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
