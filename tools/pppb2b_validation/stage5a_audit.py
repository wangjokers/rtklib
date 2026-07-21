#!/usr/bin/env python3
"""Generate the Stage 5A PPP-B2b baseline audit statistics.

Inputs are opened read-only. Generated files are created only in an explicitly
selected new output directory; existing output files are never overwritten.
The script uses only the Python standard library and never downloads data.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import math
import mmap
from pathlib import Path
import re
import statistics
import subprocess
import sys
import zlib


REFERENCE_ECEF = (-2160815.1967, 4383231.0078, 4084983.5530)
EXPECTED_EPOCHS = 2880
EXPECTED_SOLUTIONS = 2696
PPP_MINIMUM_SATELLITES = 4
DTTOL = 0.005


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def write_new_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {path}")
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


def write_new_json(path: Path, value: object) -> None:
    write_new_text(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def parse_epoch(text: str) -> dt.datetime:
    try:
        return dt.datetime.fromisoformat(text)
    except ValueError:
        pass
    for fmt in ("%Y/%m/%d %H:%M:%S.%f", "%Y/%m/%d %H:%M:%S"):
        try:
            return dt.datetime.strptime(text, fmt)
        except ValueError:
            pass
    raise ValueError(f"unsupported epoch: {text!r}")


def iso_epoch(value: dt.datetime) -> str:
    return value.isoformat(timespec="milliseconds")


def percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def read_observation_epochs(path: Path) -> list[dt.datetime]:
    epochs: list[dt.datetime] = []
    in_header = True
    with path.open("r", encoding="ascii", errors="replace", newline="") as stream:
        for line in stream:
            if in_header:
                if "END OF HEADER" in line:
                    in_header = False
                continue
            if not line.startswith(">"):
                continue
            fields = line[1:].split()
            if len(fields) < 6:
                continue
            second = float(fields[5])
            base = dt.datetime(
                int(fields[0]), int(fields[1]), int(fields[2]),
                int(fields[3]), int(fields[4]),
            )
            epochs.append(base + dt.timedelta(seconds=second))
    return epochs


def parse_pos(path: Path) -> tuple[dict[str, str], dict[dt.datetime, dict], dict]:
    header: dict[str, str] = {}
    solutions: dict[dt.datetime, dict] = {}
    errors: list[float] = []
    with path.open("r", encoding="ascii", errors="replace", newline="") as stream:
        for line in stream:
            if line.startswith("%"):
                match = re.match(r"^%\s*([^:]+?)\s*:\s*(.*)$", line.rstrip())
                if match:
                    key = match.group(1).strip()
                    value = match.group(2).strip()
                    if key in header:
                        header[key] += " | " + value
                    else:
                        header[key] = value
                continue
            fields = line.split()
            if len(fields) < 9 or not re.match(r"^\d{4}/\d{2}/\d{2}$", fields[0]):
                continue
            time = parse_epoch(fields[0] + " " + fields[1])
            xyz = tuple(float(fields[index]) for index in (2, 3, 4))
            quality = int(fields[5])
            satellites = int(fields[6])
            error = math.sqrt(sum((xyz[i] - REFERENCE_ECEF[i]) ** 2 for i in range(3)))
            errors.append(error)
            solutions[time] = {
                "quality": quality,
                "satellites": satellites,
                "x_m": xyz[0],
                "y_m": xyz[1],
                "z_m": xyz[2],
                "error_3d_m": error,
            }
    metrics = {
        "solution_epoch_count": len(solutions),
        "mean_error_3d_m": statistics.fmean(errors) if errors else None,
        "rms_error_3d_m": math.sqrt(statistics.fmean([value * value for value in errors])) if errors else None,
        "p95_error_3d_m": percentile(errors, 0.95),
        "max_error_3d_m": max(errors) if errors else None,
        "last_10_mean_error_3d_m": statistics.fmean(errors[-10:]) if len(errors) >= 10 else None,
        "minimum_solution_satellites": min((row["satellites"] for row in solutions.values()), default=None),
        "maximum_solution_satellites": max((row["satellites"] for row in solutions.values()), default=None),
        "mean_solution_satellites": statistics.fmean([row["satellites"] for row in solutions.values()]) if solutions else None,
    }
    return header, solutions, metrics


def _get_bits(data: bytes | mmap.mmap, bit_offset: int, bit_length: int) -> int:
    value = 0
    for bit in range(bit_offset, bit_offset + bit_length):
        byte = data[bit // 8]
        value = (value << 1) | ((byte >> (7 - bit % 8)) & 1)
    return value


def _get_signed_bits(data: bytes | mmap.mmap, bit_offset: int, bit_length: int) -> int:
    value = _get_bits(data, bit_offset, bit_length)
    if value & (1 << (bit_length - 1)):
        value -= 1 << bit_length
    return value


def _rtk_crc32(data: bytes | mmap.mmap) -> int:
    return (zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF


def _crc24q_bits(data: bytes | mmap.mmap, bit_offset: int, bit_length: int) -> int:
    crc = 0
    for bit in range(bit_offset, bit_offset + bit_length):
        input_bit = (data[bit // 8] >> (7 - bit % 8)) & 1
        feedback = ((crc >> 23) & 1) ^ input_bit
        crc = (crc << 1) & 0xFFFFFF
        if feedback:
            crc ^= 0x864CFB
    return crc


def raw_frame_audit(path: Path) -> dict:
    type_counts: collections.Counter[int] = collections.Counter()
    service_type_counts: collections.Counter[int] = collections.Counter()
    prn_counts: collections.Counter[int] = collections.Counter()
    status_counts: collections.Counter[int] = collections.Counter()
    single_pass: collections.Counter[int] = collections.Counter()
    single_filtered: collections.Counter[int] = collections.Counter()
    per_prn_pass: collections.Counter[int] = collections.Counter()
    service_single_pass: collections.Counter[int] = collections.Counter()
    current_mask: tuple[int, int] | None = None
    prn_masks: dict[int, tuple[int, int]] = {}
    structural_errors = outer_crc_errors = inner_crc_errors = 0
    prn_mismatches = time_backwards = 0
    frame_count = message_1697_count = 0
    consumed_bytes = 0
    previous_time_ms: int | None = None
    raw_urai_zero_records = published_urai_zero_records = 0
    raw_urai_63_records = published_urai_63_records = 0

    with path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
        offset = 0
        while offset < len(data):
            if offset + 32 > len(data) or data[offset:offset + 3] != b"\xAA\x44\x12":
                structural_errors += 1
                next_sync = data.find(b"\xAA\x44\x12", offset + 1)
                if next_sync < 0:
                    break
                offset = next_sync
                continue
            header_length = data[offset + 3]
            payload_length = int.from_bytes(data[offset + 8:offset + 10], "little")
            frame_length = header_length + payload_length + 4
            if header_length != 28 or payload_length != 127 or offset + frame_length > len(data):
                structural_errors += 1
                offset += 1
                continue
            frame_count += 1
            body_end = offset + frame_length - 4
            stored_outer_crc = int.from_bytes(data[body_end:body_end + 4], "little")
            outer_ok = _rtk_crc32(data[offset:body_end]) == stored_outer_crc
            if not outer_ok:
                outer_crc_errors += 1
            message = int.from_bytes(data[offset + 4:offset + 6], "little")
            if message == 1697:
                message_1697_count += 1
            week = int.from_bytes(data[offset + 14:offset + 16], "little")
            tow_ms = int.from_bytes(data[offset + 16:offset + 20], "little")
            absolute_ms = week * 604800000 + tow_ms
            if previous_time_ms is not None and absolute_ms < previous_time_ms:
                time_backwards += 1
            previous_time_ms = absolute_ms
            payload_bit = (offset + 28) * 8
            prn32 = int.from_bytes(data[offset + 28:offset + 32], "little")
            prn6 = _get_bits(data, payload_bit + 32, 6)
            status = _get_bits(data, payload_bit + 38, 6)
            message_type = _get_bits(data, payload_bit + 44, 6)
            stored_inner_crc = _get_bits(data, payload_bit + 506, 24)
            inner_crc = _crc24q_bits(data, payload_bit + 44, 462)
            if inner_crc != stored_inner_crc:
                inner_crc_errors += 1
            if prn32 != prn6:
                prn_mismatches += 1
            prn_counts[prn6] += 1
            status_counts[status] += 1
            type_counts[message_type] += 1
            if status & 0x20:
                service_type_counts[message_type] += 1

            data_bit = payload_bit + 50
            iodssr = _get_bits(data, data_bit + 21, 2) if message_type in (1, 2, 3, 4) else None
            iodp = _get_bits(data, data_bit + 23, 4) if message_type in (1, 4) else None
            if outer_ok and message == 1697:
                if message_type == 1 and iodssr is not None and iodp is not None:
                    current_mask = (iodssr, iodp)
                    prn_masks[prn6] = current_mask
                    single_pass[1] += 1
                    per_prn_pass[1] += 1
                    if status & 0x20:
                        service_single_pass[1] += 1
                elif message_type in (2, 3, 4) and iodssr is not None:
                    current_ok = current_mask is not None and current_mask[0] == iodssr
                    prn_ok = prn6 in prn_masks and prn_masks[prn6][0] == iodssr
                    if message_type == 4:
                        current_ok = current_ok and current_mask[1] == iodp
                        prn_ok = prn_ok and prn_masks[prn6][1] == iodp
                    (single_pass if current_ok else single_filtered)[message_type] += 1
                    if prn_ok:
                        per_prn_pass[message_type] += 1
                    if current_ok and status & 0x20:
                        service_single_pass[message_type] += 1

                    if message_type == 2:
                        record_bit = data_bit + 23
                        for _ in range(6):
                            slot = _get_bits(data, record_bit, 9)
                            radial = _get_signed_bits(data, record_bit + 22, 15)
                            along = _get_signed_bits(data, record_bit + 37, 13)
                            cross = _get_signed_bits(data, record_bit + 50, 13)
                            urai = _get_bits(data, record_bit + 63, 6)
                            valid_record = slot > 0 and abs(radial) < 16383 and abs(along) < 4095 and abs(cross) < 4095
                            if valid_record and urai == 0:
                                raw_urai_zero_records += 1
                                if current_ok:
                                    published_urai_zero_records += 1
                            if valid_record and urai == 63:
                                raw_urai_63_records += 1
                                if current_ok:
                                    published_urai_63_records += 1
                            record_bit += 69
            offset += frame_length
            consumed_bytes = offset

    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "consumed_bytes": consumed_bytes,
        "frame_count": frame_count,
        "message_1697_count": message_1697_count,
        "structural_error_count": structural_errors,
        "outer_crc32_error_count": outer_crc_errors,
        "inner_crc24q_error_count": inner_crc_errors,
        "prn32_prn6_mismatch_count": prn_mismatches,
        "receiver_time_backward_count": time_backwards,
        "type_counts": {str(key): type_counts[key] for key in sorted(type_counts)},
        "prn6_counts": {str(key): prn_counts[key] for key in sorted(prn_counts)},
        "status6_counts": {str(key): status_counts[key] for key in sorted(status_counts)},
        "service_unavailable_frame_count": sum(service_type_counts.values()),
        "service_unavailable_type_counts": {str(key): service_type_counts[key] for key in sorted(service_type_counts)},
        "single_mask_context_pass_counts": {str(key): single_pass[key] for key in (1, 2, 3, 4)},
        "single_mask_context_filtered_counts": {str(key): single_filtered[key] for key in (2, 3, 4)},
        "per_prn_context_simulation_pass_counts": {str(key): per_prn_pass[key] for key in (1, 2, 3, 4)},
        "service_unavailable_frames_consumed_by_current_context_counts": {str(key): service_single_pass[key] for key in (1, 2, 3, 4)},
        "type_5_6_7_counts": {str(key): type_counts[key] for key in (5, 6, 7)},
        "urai_zero_raw_valid_record_count": raw_urai_zero_records,
        "urai_zero_current_context_published_record_count": published_urai_zero_records,
        "urai_63_raw_valid_record_count": raw_urai_63_records,
        "urai_63_current_context_published_record_count": published_urai_63_records,
        "ldpc_checked": False,
    }


def read_satellite_rows(path: Path) -> dict[dt.datetime, list[dict]]:
    grouped: dict[dt.datetime, list[dict]] = collections.defaultdict(list)
    integer_fields = {
        "epoch_index", "prn", "code_f1", "code_f2", "excluded_config",
        "dual_observation", "above_elevation_mask", "orbit_present",
        "clock_present",
        "orbit_effective_age", "clock_effective_age", "orbit_strict_age",
        "clock_strict_age", "iodssr_match", "iodcorr_match", "numeric_valid",
        "urai_current_usable", "urai_icd_reliable", "cnv1_iodn_match",
        "cbias_f1_ready", "cbias_f2_ready", "satpos_b2b_ok",
        "product_ready", "current_usable", "icd_reliable_usable",
    }
    float_fields = {"elevation_deg", "orbit_age_s", "clock_age_s"}
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            time = parse_epoch(row["time_gpst"])
            for name in integer_fields:
                row[name] = int(row[name])
            for name in float_fields:
                row[name] = float(row[name])
            grouped[time].append(row)
    return grouped


TRACE_PATTERNS = {
    "trace_orbit_clock_not_ready": "b2b orbit/clock not ready",
    "trace_code_bias_not_ready": "b2b code bias not ready",
    "trace_no_cnv1_iodn": "no b2b broadcast ephemeris",
    "trace_no_ephemeris": "no ephemeris",
    "trace_no_valid_obs": "no valid obs data",
    "trace_invalid_urai": "invalid b2b urai",
}


def nearest_observation_epoch(value: dt.datetime) -> dt.datetime:
    midnight = value.replace(hour=0, minute=0, second=0, microsecond=0)
    seconds = (value - midnight).total_seconds()
    rounded = round(seconds / 30.0) * 30.0
    return midnight + dt.timedelta(seconds=rounded)


def parse_trace(path: Path) -> tuple[dict[dt.datetime, collections.Counter], collections.Counter]:
    by_epoch: dict[dt.datetime, collections.Counter] = collections.defaultdict(collections.Counter)
    totals: collections.Counter = collections.Counter()
    timestamp_re = re.compile(r"(2026/05/2[34]\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?)")
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        for line in stream:
            labels = [name for name, marker in TRACE_PATTERNS.items() if marker in line]
            if not labels:
                continue
            match = timestamp_re.search(line)
            epoch = nearest_observation_epoch(parse_epoch(match.group(1))) if match else None
            for label in labels:
                totals[label] += 1
                if epoch is not None:
                    by_epoch[epoch][label] += 1
    return by_epoch, totals


def aggregate_epoch(
    time: dt.datetime,
    rows: list[dict],
    solution: dict | None,
    trace_counts: collections.Counter,
) -> dict:
    configured = [row for row in rows if not row["excluded_config"]]
    dual = [row for row in configured if row["dual_observation"]]
    candidates = [row for row in dual if row["above_elevation_mask"]]
    reason_counts = collections.Counter(
        row["invalid_reason"] for row in dual if row["invalid_reason"] != "NONE"
    )
    current_usable = sum(row["current_usable"] for row in candidates)
    result = {
        "time_gpst": iso_epoch(time),
        "solution_available": int(solution is not None),
        "solution_quality": solution["quality"] if solution else "",
        "solution_satellites": solution["satellites"] if solution else "",
        "error_3d_m": f"{solution['error_3d_m']:.6f}" if solution else "",
        "observed_bds_satellites": len(rows),
        "configured_bds_satellites": len(configured),
        "dual_frequency_satellites": len(dual),
        "above_elevation_satellites": len(candidates),
        "matching_cnv1_iodn_satellites": sum(row["cnv1_iodn_match"] for row in dual),
        "orbit_product_present_satellites": sum(row["orbit_present"] for row in dual),
        "orbit_product_effective_satellites": sum(row["orbit_effective_age"] for row in dual),
        "clock_product_present_satellites": sum(row["clock_present"] for row in dual),
        "clock_product_effective_satellites": sum(row["clock_effective_age"] for row in dual),
        "code_bias_ready_satellites": sum(row["cbias_f1_ready"] and row["cbias_f2_ready"] for row in dual),
        "strict_product_satellites": sum(row["product_class"] == "STRICT" for row in dual),
        "degraded_product_satellites": sum(row["product_class"] == "DEGRADED" for row in dual),
        "invalid_product_satellites": sum(row["product_class"] == "INVALID" for row in dual),
        "current_usable_satellites": current_usable,
        "icd_reliable_usable_satellites": sum(row["icd_reliable_usable"] for row in candidates),
        "iodssr_mismatch_satellites": sum(row["orbit_present"] and row["clock_present"] and not row["iodssr_match"] for row in dual),
        "iodcorr_mismatch_satellites": sum(row["orbit_present"] and row["clock_present"] and not row["iodcorr_match"] for row in dual),
        "iodn_cnv1_missing_satellites": sum(row["orbit_present"] and not row["cnv1_iodn_match"] for row in dual),
        "orbit_stale_satellites": sum(row["orbit_present"] and row["orbit_age_s"] > 126.0 + DTTOL for row in dual),
        "clock_stale_satellites": sum(row["clock_present"] and row["clock_age_s"] > 42.0 + DTTOL for row in dual),
        "urai_current_unusable_satellites": sum(row["orbit_present"] and not row["urai_current_usable"] for row in dual),
        "urai_icd_unreliable_satellites": sum(row["orbit_present"] and not row["urai_icd_reliable"] for row in dual),
        "usable_satellites_below_ppp_minimum": int(current_usable < PPP_MINIMUM_SATELLITES),
        "invalid_reason_counts": json.dumps(dict(sorted(reason_counts.items())), ensure_ascii=False, sort_keys=True),
    }
    for label in TRACE_PATTERNS:
        result[label] = trace_counts[label]
    return result


def gap_rows(epoch_rows: list[dict]) -> list[dict]:
    gaps: list[list[dict]] = []
    current: list[dict] = []
    for row in epoch_rows:
        if not row["solution_available"]:
            if current:
                previous = parse_epoch(current[-1]["time_gpst"])
                this = parse_epoch(row["time_gpst"])
                if (this - previous).total_seconds() > 30.0 + 1E-6:
                    gaps.append(current)
                    current = []
            current.append(row)
        elif current:
            gaps.append(current)
            current = []
    if current:
        gaps.append(current)

    output: list[dict] = []
    for gap_id, rows in enumerate(gaps, 1):
        reason_counts: collections.Counter[str] = collections.Counter()
        for row in rows:
            reason_counts.update(json.loads(row["invalid_reason_counts"]))
        dominant = [item[0] for item in reason_counts.most_common(2)]
        below = sum(row["usable_satellites_below_ppp_minimum"] for row in rows)
        no_valid_obs = sum(row["trace_no_valid_obs"] for row in rows)
        if below == len(rows):
            primary = "CURRENT_USABLE_SATELLITES_BELOW_4"
        elif no_valid_obs:
            primary = "PPP_NO_VALID_OBSERVATION_OR_POSTFIT"
        else:
            primary = "PPP_FILTER_OR_GEOMETRY_AFTER_PRODUCT_PRECHECK"
        secondary = dominant[0] if dominant else "NO_DOMINANT_PRODUCT_REASON"
        tertiary = dominant[1] if len(dominant) > 1 else ""
        usable = [row["current_usable_satellites"] for row in rows]
        output.append({
            "gap_id": gap_id,
            "start_time_gpst": rows[0]["time_gpst"],
            "end_time_gpst": rows[-1]["time_gpst"],
            "missing_epoch_count": len(rows),
            "duration_seconds": len(rows) * 30,
            "minimum_current_usable_satellites": min(usable),
            "mean_current_usable_satellites": f"{statistics.fmean(usable):.3f}",
            "maximum_current_usable_satellites": max(usable),
            "epochs_below_ppp_minimum": below,
            "primary_reason": primary,
            "secondary_reason": secondary,
            "tertiary_reason": tertiary,
            "reason_counts": json.dumps(dict(sorted(reason_counts.items())), ensure_ascii=False, sort_keys=True),
        })
    return output


def write_csv_new(path: Path, rows: list[dict], columns: list[str]) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def git_text(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout.strip()


def generate_manifest(output_dir: Path, inputs: list[Path]) -> None:
    manifest = output_dir / "manifest.sha256"
    if manifest.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {manifest}")
    records: list[tuple[str, Path]] = []
    for path in inputs:
        records.append((str(path.resolve()), path))
    for path in sorted(output_dir.rglob("*")):
        if path.is_file() and path != manifest:
            records.append((str(path.relative_to(output_dir)).replace("\\", "/"), path))
    lines = [f"{sha256_file(path)} *{label}" for label, path in records]
    write_new_text(manifest, "\n".join(lines) + "\n")


def run(args: argparse.Namespace) -> int:
    output_dir = Path(args.output_dir).resolve()
    if not output_dir.is_dir():
        raise FileNotFoundError(f"output directory must already exist: {output_dir}")
    paths = {
        "observation_mo": Path(args.obs).resolve(),
        "navigation_mn": Path(args.nav).resolve(),
        "sino_b2b_raw": Path(args.b2b).resolve(),
        "position_output": Path(args.pos).resolve(),
        "trace_output": Path(args.trace).resolve(),
        "satellite_status": Path(args.satellite_csv).resolve(),
        "configuration": Path(args.config).resolve(),
        "executable": Path(args.executable).resolve(),
        "run_metadata": Path(args.run_metadata).resolve(),
    }
    for role, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"missing {role}: {path}")

    observation_epochs = read_observation_epochs(paths["observation_mo"])
    pos_header, solutions, pos_metrics = parse_pos(paths["position_output"])
    satellite_rows = read_satellite_rows(paths["satellite_status"])
    trace_by_epoch, trace_totals = parse_trace(paths["trace_output"])
    raw_audit = raw_frame_audit(paths["sino_b2b_raw"])

    epoch_rows = [
        aggregate_epoch(time, satellite_rows.get(time, []), solutions.get(time), trace_by_epoch.get(time, collections.Counter()))
        for time in observation_epochs
    ]
    gaps = gap_rows(epoch_rows)
    epoch_columns = list(epoch_rows[0]) if epoch_rows else []
    gap_columns = list(gaps[0]) if gaps else [
        "gap_id", "start_time_gpst", "end_time_gpst", "missing_epoch_count",
        "duration_seconds", "minimum_current_usable_satellites",
        "mean_current_usable_satellites", "maximum_current_usable_satellites",
        "epochs_below_ppp_minimum", "primary_reason", "secondary_reason",
        "tertiary_reason", "reason_counts",
    ]
    write_csv_new(output_dir / "epoch_status.csv", epoch_rows, epoch_columns)
    write_csv_new(output_dir / "gap_intervals.csv", gaps, gap_columns)
    write_new_json(output_dir / "raw_frame_audit.json", raw_audit)

    repo = Path(args.repo).resolve()
    input_records = {
        role: {
            "path": str(path),
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for role, path in paths.items()
    }
    missing_count = len(observation_epochs) - len(solutions)
    strict_solution_epochs = sum(row["solution_available"] and row["strict_product_satellites"] >= PPP_MINIMUM_SATELLITES for row in epoch_rows)
    degraded_solution_epochs = sum(row["solution_available"] and row["degraded_product_satellites"] > 0 for row in epoch_rows)
    summary = {
        "schema_version": "stage5a-1.0",
        "generated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "stage_definition": "Stage 5A real end-to-end baseline freeze and acceptance-gap audit",
        "git": {
            "commit": git_text(repo, "rev-parse", "HEAD"),
            "commit_short": git_text(repo, "rev-parse", "--short", "HEAD"),
            "status_short": git_text(repo, "status", "--short", "--untracked-files=all").splitlines(),
            "diff_stat": git_text(repo, "diff", "--stat").splitlines(),
        },
        "inputs_and_outputs": input_records,
        "run_metadata": json.loads(paths["run_metadata"].read_text(encoding="utf-8")),
        "pos_header": pos_header,
        "baseline": {
            "observation_epoch_count": len(observation_epochs),
            "solution_epoch_count": len(solutions),
            "missing_solution_epoch_count": missing_count,
            "solution_rate_percent": len(solutions) / len(observation_epochs) * 100.0 if observation_epochs else None,
            "gap_interval_count": len(gaps),
            "longest_gap_epochs": max((row["missing_epoch_count"] for row in gaps), default=0),
            "longest_gap_seconds": max((row["duration_seconds"] for row in gaps), default=0),
            "epochs_below_ppp_minimum": sum(row["usable_satellites_below_ppp_minimum"] for row in epoch_rows),
            "strict_solution_epochs": strict_solution_epochs,
            "solution_epochs_using_at_least_one_degraded_product": degraded_solution_epochs,
            **pos_metrics,
        },
        "published_baseline_comparison": {
            "expected_observation_epochs": EXPECTED_EPOCHS,
            "observation_epoch_delta": len(observation_epochs) - EXPECTED_EPOCHS,
            "expected_solution_epochs": EXPECTED_SOLUTIONS,
            "solution_epoch_delta": len(solutions) - EXPECTED_SOLUTIONS,
            "expected_solution_rate_percent": 93.61,
            "solution_rate_delta_percentage_points": (len(solutions) / len(observation_epochs) * 100.0 - 93.61) if observation_epochs else None,
            "expected_mean_error_3d_m_approx": 1.67,
            "mean_error_3d_delta_m": (pos_metrics["mean_error_3d_m"] - 1.67) if pos_metrics["mean_error_3d_m"] is not None else None,
            "expected_longest_gap_epochs_approx": 10,
            "longest_gap_epoch_delta": max((row["missing_epoch_count"] for row in gaps), default=0) - 10,
        },
        "trace_totals": dict(sorted(trace_totals.items())),
        "raw_frame_audit": raw_audit,
        "epoch_metric_totals": {
            name: sum(int(row[name]) for row in epoch_rows)
            for name in (
                "iodssr_mismatch_satellites", "iodcorr_mismatch_satellites",
                "iodn_cnv1_missing_satellites", "orbit_stale_satellites",
                "clock_stale_satellites", "urai_current_unusable_satellites",
                "urai_icd_unreliable_satellites",
            )
        },
        "limitations": [
            "The candidate ECEF truth is not yet accepted as a frame/epoch/reference-point truth.",
            "The diagnostic elevation gate uses the RINEX station approximate position; PPP post-fit rejection remains observable only in trace/output.",
            "LDPC parity over the final 486 bits is not verified.",
            "Per-PRN MASK context is an offline comparison only, not a proposed production design.",
        ],
    }
    write_new_json(output_dir / "summary.json", summary)
    generate_manifest(
        output_dir,
        [
            paths["observation_mo"], paths["navigation_mn"],
            paths["sino_b2b_raw"], paths["configuration"],
            paths["executable"],
        ],
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, help="rtklib Git worktree root")
    parser.add_argument("--obs", required=True)
    parser.add_argument("--nav", required=True)
    parser.add_argument("--b2b", required=True)
    parser.add_argument("--pos", required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--satellite-csv", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--run-metadata", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except Exception as exc:
        print(f"stage5a_audit: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
