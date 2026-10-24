#!/usr/bin/env python3
"""任务10 IMU数据集离线检查与回放入口。

当前版本提供机械检查、设计集分析与调参、参数冻结和一次性独立验证。
首次验证命令拒绝覆盖既有结果，并在读取验证信号前留下运行标记。

设计约束：
1. 参数冻结前不对validation的传感器数值做统计、绘图或算法运算；
2. check只解析控制字段，传感器raw/物理量字段只检查列是否存在；
3. 所有校验失败都汇总输出，便于一次修复，而不是遇到首错就停止；
4. 仅使用Python标准库，保证工具在干净环境中可以复现。
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import re
import statistics
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable, Sequence


TOOL_VERSION = "task10_replay_v4"
UINT32_MASK = 0xFFFFFFFF
SHA256_RE = re.compile(r"^[0-9A-Fa-f]{64}$")
SEGMENT_ID_RE = re.compile(r"^[A-Z0-9_]+$")

EXPECTED_FIELDS = (
    "capture_index",
    "poll_tick",
    "late_ticks",
    "snapshot_result",
    "publish_seq",
    "imu_seq",
    "sample_tick",
    "delta_tick",
    "is_new_sample",
    "state",
    "valid",
    "stale",
    "age_ticks",
    "last_error",
    "config_version",
    "ax_raw",
    "ay_raw",
    "az_raw",
    "gx_raw",
    "gy_raw",
    "gz_raw",
    "ax_ug",
    "ay_ug",
    "az_ug",
    "gx_udps",
    "gy_udps",
    "gz_udps",
)

REQUIRED_MANIFEST_FIELDS = (
    "dataset_version",
    "segment_id",
    "split",
    "class",
    "label_mode",
    "duration_s",
    "eval_start_ms",
    "eval_end_ms",
    "reference_count",
    "placement",
    "usb_cable_direction",
    "project_head",
    "project_dirty",
    "project_diff_sha256",
    "sdk_head",
    "main_bin_sha256",
    "build_config",
    "capture_format_version",
    "tick_hz",
    "imu_config_version",
    "raw_uart_sha256",
    "raw_csv_sha256",
    "label_csv_sha256",
    "video_sha256",
    "capture_time",
    "note",
)

# 固定正式批次可以防止脚本误把保留的失败日志纳入算法输入。
EXPECTED_SEGMENTS = {
    "D_STILL_01": ("design", "still", 0),
    "D_STILL_02": ("design", "still", 0),
    "D_STILL_03": ("design", "still", 0),
    "D_NONSTEP_01_R1": ("design", "nonstep", 0),
    "D_NONSTEP_02_R2": ("design", "nonstep", 0),
    "D_NONSTEP_03_R1": ("design", "nonstep", 0),
    "D_WALK_01": ("design", "walk", 25),
    "D_WALK_02": ("design", "walk", 33),
    "D_WALK_03_R2": ("design", "walk", 36),
    "V_STILL_01": ("validation", "still", 0),
    "V_NONSTEP_01": ("validation", "nonstep", 0),
    "V_WALK_01": ("validation", "walk", 31),
}


@dataclass
class SegmentCheck:
    """单个正式片段的机械检查摘要，不包含传感器数值统计。"""

    segment_id: str
    split: str
    sample_class: str
    rows: int
    elapsed_ticks: int
    new_samples: int
    duplicates: int
    seq_gap_events: int
    invalid_rows: int
    stale_rows: int
    snapshot_errors: int
    config_mismatches: int
    late_polls: int
    schedule_skips: int
    max_late_ticks: int
    uart_csv_exact: bool


@dataclass(frozen=True)
class DesignFeatureSample:
    """设计集单样本派生特征；时间统一相对采集开始。"""

    segment_id: str
    sample_class: str
    time_ms: int
    sample_tick: int
    imu_seq: int
    ax_ug: int
    ay_ug: int
    az_ug: int
    a_mag_ug: int
    dynamic_ug: int


@dataclass(frozen=True)
class DesignFeatureSummary:
    """单片段特征摘要，用于理解设计集而不是直接冻结阈值。"""

    segment_id: str
    sample_class: str
    reference_count: int
    eval_rows: int
    eval_duration_ms: int
    sample_dt_min_ms: int
    sample_dt_mean_ms: float
    sample_dt_max_ms: int
    a_mag_mean_ug: float
    dynamic_mean_ug: float
    dynamic_p50_ug: int
    dynamic_p75_ug: int
    dynamic_p90_ug: int
    dynamic_p95_ug: int
    dynamic_p99_ug: int
    dynamic_max_ug: int


@dataclass(frozen=True)
class AlgorithmParameters:
    """B0/B1统一参数表示；B0不使用的字段固定为0。"""

    algorithm: str
    high_threshold_ug: int
    low_threshold_ug: int
    refractory_ms: int
    ewma_tau_ms: int
    warmup_ms: int = 200
    reset_gap_ms: int = 100


@dataclass(frozen=True)
class AlgorithmEvent:
    segment_id: str
    algorithm: str
    event_index: int
    event_time_ms: int
    dynamic_ug: int
    filtered_dynamic_ug: int


@dataclass(frozen=True)
class CandidateResult:
    parameters: AlgorithmParameters
    segment_counts: dict[str, int]
    walk_abs_error_total: int
    walk_relative_error_mean: float
    walk_max_abs_error: int
    still_false_count: int
    nonstep_false_count: int
    score: int


class DatasetChecker:
    """集中保存错误和警告，结束时一次性给出完整诊断。"""

    def __init__(self, dataset_root: Path) -> None:
        self.root = dataset_root.resolve()
        self.task_root = self.root.parent
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.segment_checks: list[SegmentCheck] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def require_file(self, path: Path, label: str) -> bool:
        if not path.is_file():
            self.error(f"{label}不存在: {path}")
            return False
        return True

    def run(self) -> None:
        """按身份、manifest、片段、封存和总哈希的顺序执行检查。"""

        if not self.root.is_dir():
            self.error(f"数据集目录不存在: {self.root}")
            return

        manifest_path = self.root / "manifest.csv"
        if not self.require_file(manifest_path, "manifest"):
            return

        manifest_rows = self._load_manifest(manifest_path)
        identity = self._read_key_values(
            self.root / "firmware_identity.txt", "firmware_identity"
        )
        self._check_identity_files(identity)
        self._check_manifest_identity(manifest_rows, identity)

        for row in manifest_rows:
            self._check_segment(row)

        self._check_dataset_inventory(manifest_rows)
        self._check_validation_seal()
        self._check_hash_inventory()

    def _load_manifest(self, path: Path) -> list[dict[str, str]]:
        try:
            with path.open("r", encoding="utf-8-sig", newline="") as handle:
                reader = csv.DictReader(handle)
                header = tuple(reader.fieldnames or ())
                missing = [name for name in REQUIRED_MANIFEST_FIELDS if name not in header]
                extra = [name for name in header if name not in REQUIRED_MANIFEST_FIELDS]
                if missing:
                    self.error(f"manifest缺少字段: {', '.join(missing)}")
                if extra:
                    self.warn(f"manifest存在扩展字段: {', '.join(extra)}")
                rows = list(reader)
        except (OSError, csv.Error) as exc:
            self.error(f"无法读取manifest: {exc}")
            return []

        if len(rows) != len(EXPECTED_SEGMENTS):
            self.error(
                f"manifest行数应为{len(EXPECTED_SEGMENTS)}，实际为{len(rows)}"
            )

        seen: set[str] = set()
        for line_number, row in enumerate(rows, start=2):
            segment_id = (row.get("segment_id") or "").strip()
            if not SEGMENT_ID_RE.fullmatch(segment_id):
                self.error(f"manifest第{line_number}行segment_id非法: {segment_id!r}")
                continue
            if segment_id in seen:
                self.error(f"manifest重复segment_id: {segment_id}")
            seen.add(segment_id)
            expected = EXPECTED_SEGMENTS.get(segment_id)
            if expected is None:
                self.error(f"manifest包含非正式片段: {segment_id}")
                continue
            expected_split, expected_class, expected_count = expected
            self._expect_equal(row, segment_id, "split", expected_split)
            self._expect_equal(row, segment_id, "class", expected_class)
            self._expect_equal(row, segment_id, "label_mode", "count_only")
            self._expect_equal(row, segment_id, "duration_s", "45")
            self._expect_equal(row, segment_id, "eval_start_ms", "6000")
            self._expect_equal(row, segment_id, "eval_end_ms", "42000")
            self._expect_equal(
                row, segment_id, "reference_count", str(expected_count)
            )
            self._expect_equal(row, segment_id, "dataset_version", "task10_v1")

            if expected_class == "walk":
                self._expect_equal(row, segment_id, "label_csv_sha256", "NA_COUNT_ONLY")
                note = row.get("note", "")
                if "confidence=medium" not in note or "possible_manual_count_error" not in note:
                    self.error(f"{segment_id}: note未记录人工计数可信度或可能误差")
            else:
                self._expect_equal(
                    row, segment_id, "label_csv_sha256", "NA_NOT_REQUIRED"
                )
                self._expect_equal(row, segment_id, "video_sha256", "NA_NOT_CAPTURED")

        missing_ids = sorted(set(EXPECTED_SEGMENTS) - seen)
        if missing_ids:
            self.error(f"manifest缺少正式片段: {', '.join(missing_ids)}")
        return rows

    def _expect_equal(
        self, row: dict[str, str], segment_id: str, field: str, expected: str
    ) -> None:
        actual = (row.get(field) or "").strip()
        if actual != expected:
            self.error(
                f"{segment_id}: manifest.{field}应为{expected!r}，实际为{actual!r}"
            )

    def _read_key_values(self, path: Path, label: str) -> dict[str, str]:
        if not self.require_file(path, label):
            return {}
        values: dict[str, str] = {}
        try:
            for line_number, raw_line in enumerate(
                path.read_text(encoding="utf-8-sig").splitlines(), start=1
            ):
                line = raw_line.strip()
                if not line:
                    continue
                if "=" not in line:
                    self.error(f"{label}第{line_number}行不是key=value: {line!r}")
                    continue
                key, value = line.split("=", 1)
                if key in values:
                    self.error(f"{label}重复字段: {key}")
                values[key] = value
        except OSError as exc:
            self.error(f"无法读取{label}: {exc}")
        return values

    def _check_identity_files(self, identity: dict[str, str]) -> None:
        required = {
            "dataset_version": "task10_v1",
            "project_head": "1b289f22bf2047d077cdfb666d1cbd8494c12697",
            "project_dirty": "false",
            "project_diff_sha256": hashlib.sha256(b"").hexdigest().upper(),
            "sdk_head": "ed3d0dfe01c3e897dc15f5723c66efce235e9dab",
            "build_config": "sf32lb52-lchspi-ulp_hcpu",
            "capture_format_version": "1",
            "tick_hz": "1000",
            "imu_config_version": "1",
            "main_bin_sha256": "AD03934CCA86014B2CA6D95912537538B857F4682D154F45AF171060BC235ACC",
        }
        for key, expected in required.items():
            actual = identity.get(key)
            if actual != expected:
                self.error(
                    f"firmware_identity.{key}应为{expected!r}，实际为{actual!r}"
                )

        patch_path = self.root / "project_state.patch"
        if self.require_file(patch_path, "project_state.patch"):
            patch_hash = sha256_file(patch_path)
            if patch_hash != identity.get("project_diff_sha256"):
                self.error("project_state.patch哈希与firmware_identity不一致")

        for binary_path in (self.root / "main.bin", self.root / "firmware" / "main.bin"):
            if self.require_file(binary_path, "main.bin"):
                if sha256_file(binary_path) != identity.get("main_bin_sha256"):
                    self.error(f"main.bin哈希与firmware_identity不一致: {binary_path}")

    def _check_manifest_identity(
        self, rows: Sequence[dict[str, str]], identity: dict[str, str]
    ) -> None:
        mapping = {
            "dataset_version": "dataset_version",
            "project_head": "project_head",
            "project_dirty": "project_dirty",
            "project_diff_sha256": "project_diff_sha256",
            "sdk_head": "sdk_head",
            "main_bin_sha256": "main_bin_sha256",
            "build_config": "build_config",
            "capture_format_version": "capture_format_version",
            "tick_hz": "tick_hz",
            "imu_config_version": "imu_config_version",
        }
        for row in rows:
            segment_id = row.get("segment_id", "<unknown>")
            for manifest_field, identity_field in mapping.items():
                if row.get(manifest_field) != identity.get(identity_field):
                    self.error(
                        f"{segment_id}: manifest.{manifest_field}与firmware_identity不一致"
                    )

    def _check_segment(self, row: dict[str, str]) -> None:
        segment_id = row.get("segment_id", "")
        if segment_id not in EXPECTED_SEGMENTS:
            return

        uart_path = self.root / "raw_uart" / f"{segment_id}.txt"
        csv_path = self.root / "raw_csv" / f"{segment_id}.csv"
        if not self.require_file(uart_path, f"{segment_id} UART"):
            return
        if not self.require_file(csv_path, f"{segment_id} CSV"):
            return

        self._check_recorded_hash(
            uart_path, row.get("raw_uart_sha256", ""), f"{segment_id} UART"
        )
        self._check_recorded_hash(
            csv_path, row.get("raw_csv_sha256", ""), f"{segment_id} CSV"
        )

        if row.get("class") == "walk":
            video_path = self.root / "video" / f"{segment_id}.mp4"
            if self.require_file(video_path, f"{segment_id} video"):
                self._check_recorded_hash(
                    video_path,
                    row.get("video_sha256", ""),
                    f"{segment_id} video",
                )

        uart_lines = read_lines(uart_path)
        csv_lines = read_lines(csv_path)
        extracted_lines = extract_dataset_lines(uart_lines)
        exact = extracted_lines == csv_lines
        if not exact:
            self.error(f"{segment_id}: raw_csv不是raw_uart的逐行精确提取")

        parsed = self._parse_segment_csv(csv_path, row)
        if parsed is None:
            return
        parsed.uart_csv_exact = exact
        self.segment_checks.append(parsed)

    def _check_recorded_hash(self, path: Path, recorded: str, label: str) -> None:
        if not SHA256_RE.fullmatch(recorded or ""):
            self.error(f"{label}的manifest哈希格式非法: {recorded!r}")
            return
        actual = sha256_file(path)
        if actual != recorded.upper():
            self.error(f"{label}哈希不一致")

    def _parse_segment_csv(
        self, path: Path, manifest_row: dict[str, str]
    ) -> SegmentCheck | None:
        segment_id = manifest_row["segment_id"]
        try:
            with path.open("r", encoding="utf-8-sig", newline="") as handle:
                rows = [row for row in csv.reader(handle) if row]
        except (OSError, csv.Error) as exc:
            self.error(f"{segment_id}: CSV读取失败: {exc}")
            return None

        if len(rows) < 3:
            self.error(f"{segment_id}: CSV记录不足")
            return None
        if rows[0][0] != "#META" or rows[1][0] != "#FIELDS" or rows[-1][0] != "#END":
            self.error(f"{segment_id}: META/FIELDS/END位置错误")
            return None

        meta = parse_key_value_cells(rows[0][1:], self.errors, f"{segment_id} META")
        end = parse_key_value_cells(rows[-1][1:], self.errors, f"{segment_id} END")
        fields = tuple(rows[1][1:])
        if fields != EXPECTED_FIELDS:
            self.error(f"{segment_id}: #FIELDS与冻结格式不一致")
            return None
        index = {name: position + 1 for position, name in enumerate(fields)}

        expected_meta = {
            "dataset_version": manifest_row["dataset_version"],
            "capture_format_version": manifest_row["capture_format_version"],
            "segment_id": segment_id,
            "split": manifest_row["split"],
            "class": manifest_row["class"],
            "requested_duration_s": manifest_row["duration_s"],
            "tick_hz": manifest_row["tick_hz"],
            "imu_config_version": manifest_row["imu_config_version"],
        }
        for key, expected in expected_meta.items():
            if meta.get(key) != expected:
                self.error(
                    f"{segment_id}: META.{key}应为{expected!r}，实际为{meta.get(key)!r}"
                )

        period_ticks = parse_int(meta.get("period_ticks"), self.errors, f"{segment_id} period_ticks")
        nominal_period_ms = parse_int(
            meta.get("nominal_period_ms"), self.errors, f"{segment_id} nominal_period_ms"
        )
        tick_hz = parse_int(meta.get("tick_hz"), self.errors, f"{segment_id} tick_hz")
        duration_s = parse_int(
            meta.get("requested_duration_s"), self.errors, f"{segment_id} duration"
        )
        config_version = parse_int(
            meta.get("imu_config_version"), self.errors, f"{segment_id} config_version"
        )
        if None in (period_ticks, nominal_period_ms, tick_hz, duration_s, config_version):
            return None
        if period_ticks <= 0 or tick_hz <= 0 or duration_s <= 0:
            self.error(f"{segment_id}: 周期、tick_hz或时长必须大于0")
            return None
        if period_ticks * 1000 != nominal_period_ms * tick_hz:
            self.error(f"{segment_id}: nominal_period_ms与period_ticks不一致")
        expected_rows = duration_s * tick_hz // period_ticks

        data_rows = rows[2:-1]
        if len(data_rows) != expected_rows:
            self.error(
                f"{segment_id}: 数据行数应为{expected_rows}，实际为{len(data_rows)}"
            )

        new_samples = 0
        duplicates = 0
        seq_gap_events = 0
        invalid_rows = 0
        stale_rows = 0
        snapshot_errors = 0
        config_mismatches = 0
        late_polls = 0
        schedule_skips = 0
        max_late_ticks = 0
        first_poll: int | None = None
        last_poll: int | None = None
        previous_poll: int | None = None
        previous_imu_seq: int | None = None

        for row_number, data_row in enumerate(data_rows, start=0):
            context = f"{segment_id} D[{row_number}]"
            if len(data_row) != len(EXPECTED_FIELDS) + 1 or data_row[0] != "D":
                self.error(f"{context}: 列数或记录类型错误")
                continue

            # validation也只读取这些调度/状态字段；传感器数值列不参与check计算。
            values: dict[str, int] = {}
            for field in (
                "capture_index",
                "poll_tick",
                "late_ticks",
                "snapshot_result",
                "publish_seq",
                "imu_seq",
                "sample_tick",
                "delta_tick",
                "is_new_sample",
                "state",
                "valid",
                "stale",
                "age_ticks",
                "last_error",
                "config_version",
            ):
                parsed_value = parse_int(data_row[index[field]], self.errors, f"{context}.{field}")
                if parsed_value is None:
                    break
                values[field] = parsed_value
            if len(values) != 15:
                continue

            if values["capture_index"] != row_number:
                self.error(
                    f"{context}: capture_index应为{row_number}，实际为{values['capture_index']}"
                )
            poll_tick = values["poll_tick"] & UINT32_MASK
            imu_seq = values["imu_seq"] & UINT32_MASK
            if first_poll is None:
                first_poll = poll_tick
                if values["delta_tick"] != 0:
                    self.error(f"{context}: 首行delta_tick应为0")
            else:
                assert previous_poll is not None
                actual_delta = u32_delta(poll_tick, previous_poll)
                if values["delta_tick"] != actual_delta:
                    self.error(f"{context}: delta_tick与poll_tick差值不一致")
                if actual_delta > period_ticks:
                    schedule_skips += max(0, actual_delta // period_ticks - 1)
            previous_poll = poll_tick
            last_poll = poll_tick

            if values["is_new_sample"] == 1:
                new_samples += 1
                if previous_imu_seq is not None:
                    seq_delta = u32_delta(imu_seq, previous_imu_seq)
                    if seq_delta != 1:
                        seq_gap_events += 1
                previous_imu_seq = imu_seq
            else:
                duplicates += 1

            invalid_rows += int(values["valid"] != 1)
            stale_rows += int(values["stale"] != 0)
            snapshot_errors += int(values["snapshot_result"] != 0)
            config_mismatches += int(values["config_version"] != config_version)
            late_polls += int(values["late_ticks"] != 0)
            max_late_ticks = max(max_late_ticks, values["late_ticks"])

        end_values = {
            name: parse_int(end.get(name), self.errors, f"{segment_id} END.{name}")
            for name in (
                "result",
                "poll_count",
                "new_sample_count",
                "duplicate_count",
                "seq_gap_count",
                "invalid_count",
                "stale_count",
                "snapshot_error_count",
                "config_mismatch_count",
                "late_poll_count",
                "schedule_skip_count",
                "max_late_ticks",
                "first_tick",
                "last_tick",
                "capture_end_tick",
                "elapsed_tick",
            )
        }
        if any(value is None for value in end_values.values()):
            return None

        computed = {
            "result": 0,
            "poll_count": len(data_rows),
            "new_sample_count": new_samples,
            "duplicate_count": duplicates,
            "seq_gap_count": seq_gap_events,
            "invalid_count": invalid_rows,
            "stale_count": stale_rows,
            "snapshot_error_count": snapshot_errors,
            "config_mismatch_count": config_mismatches,
            "late_poll_count": late_polls,
            "schedule_skip_count": schedule_skips,
            "max_late_ticks": max_late_ticks,
        }
        for key, expected in computed.items():
            if end_values[key] != expected:
                self.error(
                    f"{segment_id}: END.{key}应为{expected}，实际为{end_values[key]}"
                )

        if first_poll is not None and end_values["first_tick"] != first_poll:
            self.error(f"{segment_id}: END.first_tick与首行poll_tick不一致")
        if last_poll is not None and end_values["last_tick"] != last_poll:
            self.error(f"{segment_id}: END.last_tick与末行poll_tick不一致")
        expected_elapsed = duration_s * tick_hz
        if end_values["elapsed_tick"] != expected_elapsed:
            self.error(
                f"{segment_id}: END.elapsed_tick应为{expected_elapsed}，实际为{end_values['elapsed_tick']}"
            )
        if first_poll is not None:
            capture_span = u32_delta(end_values["capture_end_tick"], first_poll)
            if capture_span != end_values["elapsed_tick"]:
                self.error(f"{segment_id}: capture_end_tick与elapsed_tick不一致")

        return SegmentCheck(
            segment_id=segment_id,
            split=manifest_row["split"],
            sample_class=manifest_row["class"],
            rows=len(data_rows),
            elapsed_ticks=end_values["elapsed_tick"],
            new_samples=new_samples,
            duplicates=duplicates,
            seq_gap_events=seq_gap_events,
            invalid_rows=invalid_rows,
            stale_rows=stale_rows,
            snapshot_errors=snapshot_errors,
            config_mismatches=config_mismatches,
            late_polls=late_polls,
            schedule_skips=schedule_skips,
            max_late_ticks=max_late_ticks,
            uart_csv_exact=False,
        )

    def _check_dataset_inventory(self, rows: Sequence[dict[str, str]]) -> None:
        labels_dir = self.root / "labels"
        if labels_dir.is_dir() and any(path.is_file() for path in labels_dir.iterdir()):
            self.error("count_only数据集的labels目录必须为空")

        expected_csv = {f"{row['segment_id']}.csv" for row in rows}
        actual_csv = {path.name for path in (self.root / "raw_csv").glob("*.csv")}
        if actual_csv != expected_csv:
            self.error(
                "raw_csv正式文件集合与manifest不一致: "
                f"missing={sorted(expected_csv-actual_csv)}, extra={sorted(actual_csv-expected_csv)}"
            )

        expected_videos = {
            f"{row['segment_id']}.mp4" for row in rows if row.get("class") == "walk"
        }
        actual_videos = {path.name for path in (self.root / "video").glob("*.mp4")}
        if actual_videos != expected_videos:
            self.error(
                "video文件集合与四段正式步行不一致: "
                f"missing={sorted(expected_videos-actual_videos)}, extra={sorted(actual_videos-expected_videos)}"
            )

    def _check_validation_seal(self) -> None:
        seal_path = self.root / "validation_seal.txt"
        seal = self._read_key_values(seal_path, "validation_seal")
        if not seal:
            return

        expected_fixed = {
            "dataset_version": "task10_v1",
            "seal_status": "sealed_before_parameter_tuning",
            "validation_label_sha256": "NA_COUNT_ONLY",
            "validation_segments": "V_STILL_01,V_NONSTEP_01,V_WALK_01",
            "validation_reference_counts": "V_STILL_01:0,V_NONSTEP_01:0,V_WALK_01:31",
            "validation_label_mode": "count_only",
            "pre_seal_algorithm_runs": "0",
            "pre_seal_signal_plots": "0",
            "pre_seal_parameter_tuning": "0",
        }
        for key, expected in expected_fixed.items():
            if seal.get(key) != expected:
                self.error(
                    f"validation_seal.{key}应为{expected!r}，实际为{seal.get(key)!r}"
                )

        direct_hashes = {
            "manifest_sha256": self.root / "manifest.csv",
            "firmware_identity_sha256": self.root / "firmware_identity.txt",
            "protocol_sha256": self.task_root / "任务10采集与验证协议.md",
        }
        for key, path in direct_hashes.items():
            if self.require_file(path, key):
                if seal.get(key) != sha256_file(path):
                    self.error(f"validation_seal.{key}与当前文件不一致")

        validation_raw = (
            "raw_uart/V_STILL_01.txt",
            "raw_uart/V_NONSTEP_01.txt",
            "raw_uart/V_WALK_01.txt",
            "raw_csv/V_STILL_01.csv",
            "raw_csv/V_NONSTEP_01.csv",
            "raw_csv/V_WALK_01.csv",
        )
        validation_video = ("video/V_WALK_01.mp4",)
        if seal.get("validation_raw_sha256") != aggregate_hash(self.root, validation_raw):
            self.error("validation_seal.validation_raw_sha256不一致")
        if seal.get("validation_video_sha256") != aggregate_hash(
            self.root, validation_video
        ):
            self.error("validation_seal.validation_video_sha256不一致")

    def _check_hash_inventory(self) -> None:
        inventory_path = self.root / "hashes.sha256"
        if not self.require_file(inventory_path, "hashes.sha256"):
            return
        seen: set[str] = set()
        try:
            lines = read_lines(inventory_path)
        except OSError as exc:
            self.error(f"无法读取hashes.sha256: {exc}")
            return
        for line_number, line in enumerate(lines, start=1):
            match = re.fullmatch(r"([0-9A-Fa-f]{64})  (.+)", line)
            if match is None:
                self.error(f"hashes.sha256第{line_number}行格式错误")
                continue
            recorded_hash, relative_name = match.groups()
            if relative_name in seen:
                self.error(f"hashes.sha256重复路径: {relative_name}")
                continue
            seen.add(relative_name)
            candidate = (self.root / Path(relative_name.replace("/", "\\"))).resolve()
            # 哈希清单只允许覆盖dataset和同级task10协议，防止路径意外越界。
            if not is_relative_to(candidate, self.task_root):
                self.error(f"hashes.sha256路径越出task10目录: {relative_name}")
                continue
            if not candidate.is_file():
                self.error(f"hashes.sha256目标不存在: {relative_name}")
                continue
            if sha256_file(candidate) != recorded_hash.upper():
                self.error(f"hashes.sha256不匹配: {relative_name}")

        required_names = {
            "manifest.csv",
            "validation_seal.txt",
            "firmware_identity.txt",
            "project_state.patch",
            "firmware/main.bin",
            "../任务10采集与验证协议.md",
        }
        required_names.update(f"raw_uart/{name}.txt" for name in EXPECTED_SEGMENTS)
        required_names.update(f"raw_csv/{name}.csv" for name in EXPECTED_SEGMENTS)
        required_names.update(
            f"video/{name}.mp4"
            for name, (_, sample_class, _) in EXPECTED_SEGMENTS.items()
            if sample_class == "walk"
        )
        missing = sorted(required_names - seen)
        if missing:
            self.error(f"hashes.sha256缺少正式证据: {', '.join(missing)}")

    def report(self) -> dict[str, object]:
        return {
            "tool_version": TOOL_VERSION,
            "dataset_root": str(self.root),
            "status": "PASS" if not self.errors else "FAIL",
            "validation_policy": (
                "mechanical_only_before_parameter_freeze; no validation sensor-value statistics"
            ),
            "segments": [asdict(item) for item in self.segment_checks],
            "errors": self.errors,
            "warnings": self.warnings,
        }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def aggregate_hash(root: Path, relative_names: Iterable[str]) -> str:
    """按validation_seal冻结的格式计算一组文件的聚合哈希。"""

    lines = []
    for relative_name in sorted(relative_names):
        path = root / Path(relative_name.replace("/", "\\"))
        lines.append(f"{relative_name}={sha256_file(path)}")
    payload = ("\n".join(lines) + "\n").encode("utf-8")
    return hashlib.sha256(payload).hexdigest().upper()


def read_lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8-sig").splitlines()


def extract_dataset_lines(lines: Sequence[str]) -> list[str]:
    """复现UART到raw_csv的冻结提取规则，不改写字段内容。"""

    extracted: list[str] = []
    for line in lines:
        meta_position = line.find("#META,")
        if meta_position >= 0:
            extracted.append(line[meta_position:])
            continue
        if line.startswith(("#FIELDS,", "D,", "#END,")):
            extracted.append(line)
    return extracted


def parse_key_value_cells(
    cells: Sequence[str], errors: list[str], context: str
) -> dict[str, str]:
    values: dict[str, str] = {}
    for cell in cells:
        if "=" not in cell:
            errors.append(f"{context}: 不是key=value: {cell!r}")
            continue
        key, value = cell.split("=", 1)
        if key in values:
            errors.append(f"{context}: 重复字段{key}")
        values[key] = value
    return values


def parse_int(value: str | None, errors: list[str], context: str) -> int | None:
    if value is None:
        errors.append(f"{context}: 缺少整数值")
        return None
    try:
        return int(value, 10)
    except ValueError:
        errors.append(f"{context}: 非法整数{value!r}")
        return None


def u32_delta(current: int, previous: int) -> int:
    return (current - previous) & UINT32_MASK


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def print_human_report(report: dict[str, object]) -> None:
    print(f"IMU dataset check: {report['status']}")
    print(f"tool_version: {report['tool_version']}")
    print(f"dataset_root: {report['dataset_root']}")
    print("validation_policy: mechanical_only_before_parameter_freeze")
    print()
    print(
        "segment_id        split       class    rows  elapsed  new   dup gap "
        "invalid stale snap cfg late skip maxlate exact"
    )
    for raw_item in report["segments"]:  # type: ignore[index]
        item = raw_item  # type: ignore[assignment]
        print(
            f"{item['segment_id']:<17} {item['split']:<11} {item['sample_class']:<8} "
            f"{item['rows']:>5} {item['elapsed_ticks']:>8} {item['new_samples']:>5} "
            f"{item['duplicates']:>3} {item['seq_gap_events']:>3} "
            f"{item['invalid_rows']:>7} {item['stale_rows']:>5} "
            f"{item['snapshot_errors']:>4} {item['config_mismatches']:>3} "
            f"{item['late_polls']:>4} {item['schedule_skips']:>4} "
            f"{item['max_late_ticks']:>7} {str(item['uart_csv_exact']):>5}"
        )

    warnings = report["warnings"]
    errors = report["errors"]
    if warnings:
        print("\nWARNINGS:")
        for warning in warnings:  # type: ignore[assignment]
            print(f"- {warning}")
    if errors:
        print("\nERRORS:")
        for error in errors:  # type: ignore[assignment]
            print(f"- {error}")
    print(
        f"\nsummary: segments={len(report['segments'])}, "
        f"warnings={len(warnings)}, errors={len(errors)}"
    )


def write_json_report(path: Path, report: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def percentile_nearest_rank(values: Sequence[int], percentile: int) -> int:
    """使用nearest-rank定义，避免不同统计库插值规则造成结果漂移。"""

    if not values:
        raise ValueError("percentile输入不能为空")
    if not 0 < percentile <= 100:
        raise ValueError("percentile必须位于(0, 100]")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile * len(ordered) / 100))
    return ordered[rank - 1]


def write_csv_atomic(
    path: Path, fieldnames: Sequence[str], rows: Iterable[dict[str, object]]
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def load_feature_samples(
    dataset_root: Path,
    manifest_row: dict[str, str],
    expected_split: str,
) -> tuple[list[DesignFeatureSample], list[int]]:
    """读取指定split，并把manifest评价窗口转换为统一回放特征。"""

    segment_id = manifest_row["segment_id"]
    expected_prefix = "D_" if expected_split == "design" else "V_"
    if (
        manifest_row["split"] != expected_split
        or not segment_id.startswith(expected_prefix)
    ):
        raise ValueError(f"拒绝读取非{expected_split}片段: {segment_id}")

    csv_path = dataset_root / "raw_csv" / f"{segment_id}.csv"
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = [row for row in csv.reader(handle) if row]
    meta = parse_key_value_cells(rows[0][1:], [], f"{segment_id} META")
    fields = tuple(rows[1][1:])
    if fields != EXPECTED_FIELDS:
        raise ValueError(f"{segment_id}: #FIELDS与冻结格式不一致")
    index = {name: position + 1 for position, name in enumerate(fields)}

    capture_start_tick = int(meta["capture_start_tick"], 10) & UINT32_MASK
    tick_hz = int(meta["tick_hz"], 10)
    eval_start_ms = int(manifest_row["eval_start_ms"], 10)
    eval_end_ms = int(manifest_row["eval_end_ms"], 10)
    samples: list[DesignFeatureSample] = []
    sample_deltas_ms: list[int] = []
    previous_sample_tick: int | None = None

    for data_row in rows[2:-1]:
        if data_row[0] != "D":
            continue
        poll_tick = int(data_row[index["poll_tick"]], 10) & UINT32_MASK
        elapsed_ticks = u32_delta(poll_tick, capture_start_tick)
        time_ms = elapsed_ticks * 1000 // tick_hz
        if not eval_start_ms <= time_ms < eval_end_ms:
            continue

        # validation只允许在冻结参数文件和侧车哈希均校验通过后进入这里。
        sample_tick = int(data_row[index["sample_tick"]], 10) & UINT32_MASK
        imu_seq = int(data_row[index["imu_seq"]], 10) & UINT32_MASK
        ax_ug = int(data_row[index["ax_ug"]], 10)
        ay_ug = int(data_row[index["ay_ug"]], 10)
        az_ug = int(data_row[index["az_ug"]], 10)
        square_sum = ax_ug * ax_ug + ay_ug * ay_ug + az_ug * az_ug
        a_mag_ug = math.isqrt(square_sum)
        dynamic_ug = abs(a_mag_ug - 1_000_000)

        if previous_sample_tick is not None:
            delta_ticks = u32_delta(sample_tick, previous_sample_tick)
            sample_deltas_ms.append(delta_ticks * 1000 // tick_hz)
        previous_sample_tick = sample_tick
        samples.append(
            DesignFeatureSample(
                segment_id=segment_id,
                sample_class=manifest_row["class"],
                time_ms=time_ms,
                sample_tick=sample_tick,
                imu_seq=imu_seq,
                ax_ug=ax_ug,
                ay_ug=ay_ug,
                az_ug=az_ug,
                a_mag_ug=a_mag_ug,
                dynamic_ug=dynamic_ug,
            )
        )

    expected_rows = (eval_end_ms - eval_start_ms) * tick_hz // 20_000
    if len(samples) != expected_rows:
        raise ValueError(
            f"{segment_id}: 评价窗口应为{expected_rows}行，实际为{len(samples)}行"
        )
    return samples, sample_deltas_ms


def load_design_feature_samples(
    dataset_root: Path, manifest_row: dict[str, str]
) -> tuple[list[DesignFeatureSample], list[int]]:
    return load_feature_samples(dataset_root, manifest_row, "design")


def load_validation_feature_samples(
    dataset_root: Path, manifest_row: dict[str, str]
) -> tuple[list[DesignFeatureSample], list[int]]:
    return load_feature_samples(dataset_root, manifest_row, "validation")


def summarize_design_segment(
    manifest_row: dict[str, str],
    samples: Sequence[DesignFeatureSample],
    sample_deltas_ms: Sequence[int],
) -> DesignFeatureSummary:
    dynamic = [sample.dynamic_ug for sample in samples]
    magnitudes = [sample.a_mag_ug for sample in samples]
    return DesignFeatureSummary(
        segment_id=manifest_row["segment_id"],
        sample_class=manifest_row["class"],
        reference_count=int(manifest_row["reference_count"], 10),
        eval_rows=len(samples),
        eval_duration_ms=int(manifest_row["eval_end_ms"], 10)
        - int(manifest_row["eval_start_ms"], 10),
        sample_dt_min_ms=min(sample_deltas_ms),
        sample_dt_mean_ms=round(statistics.fmean(sample_deltas_ms), 6),
        sample_dt_max_ms=max(sample_deltas_ms),
        a_mag_mean_ug=round(statistics.fmean(magnitudes), 3),
        dynamic_mean_ug=round(statistics.fmean(dynamic), 3),
        dynamic_p50_ug=percentile_nearest_rank(dynamic, 50),
        dynamic_p75_ug=percentile_nearest_rank(dynamic, 75),
        dynamic_p90_ug=percentile_nearest_rank(dynamic, 90),
        dynamic_p95_ug=percentile_nearest_rank(dynamic, 95),
        dynamic_p99_ug=percentile_nearest_rank(dynamic, 99),
        dynamic_max_ug=max(dynamic),
    )


def render_dynamic_svg(
    path: Path, segment_id: str, sample_class: str, samples: Sequence[DesignFeatureSample]
) -> None:
    """用标准库生成可审查SVG，避免依赖matplotlib和环境字体。"""

    width, height = 1200, 360
    left, right, top, bottom = 75, 25, 35, 55
    plot_width = width - left - right
    plot_height = height - top - bottom
    max_dynamic = max(sample.dynamic_ug for sample in samples)
    y_limit = max(100_000, ((max_dynamic + 99_999) // 100_000) * 100_000)
    start_ms = samples[0].time_ms
    end_ms = samples[-1].time_ms
    time_span = max(1, end_ms - start_ms)
    points = []
    for sample in samples:
        x = left + (sample.time_ms - start_ms) * plot_width / time_span
        y = top + plot_height - sample.dynamic_ug * plot_height / y_limit
        points.append(f"{x:.2f},{y:.2f}")

    grid = []
    for step in range(5):
        fraction = step / 4
        y = top + plot_height * (1 - fraction)
        value = int(y_limit * fraction)
        grid.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{width-right}" y2="{y:.2f}" '
            'stroke="#d8dde6" stroke-width="1"/>'
        )
        grid.append(
            f'<text x="{left-8}" y="{y+4:.2f}" text-anchor="end" '
            f'font-size="12" fill="#475569">{value}</text>'
        )

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{left}" y="22" font-size="16" font-weight="600" fill="#0f172a">'
        f'{segment_id} ({sample_class}) dynamic = |a_mag - 1g|</text>',
        *grid,
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_height}" '
        'stroke="#334155" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top+plot_height}" x2="{width-right}" '
        f'y2="{top+plot_height}" stroke="#334155" stroke-width="1"/>',
        f'<polyline points="{" ".join(points)}" fill="none" stroke="#2563eb" '
        'stroke-width="1.2" stroke-linejoin="round"/>',
        f'<text x="{left}" y="{height-18}" font-size="12" fill="#475569">'
        f'{start_ms} ms</text>',
        f'<text x="{width-right}" y="{height-18}" text-anchor="end" font-size="12" '
        f'fill="#475569">{end_ms} ms</text>',
        f'<text x="18" y="{top+plot_height/2:.2f}" transform="rotate(-90 18 '
        f'{top+plot_height/2:.2f})" text-anchor="middle" font-size="12" '
        'fill="#475569">dynamic (ug)</text>',
        '</svg>',
    ]
    svg = "\n".join(svg_parts) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(svg, encoding="utf-8")
    temporary.replace(path)


def write_analysis_hashes(output_root: Path, paths: Sequence[Path]) -> Path:
    hash_path = output_root / "design_analysis_hashes.sha256"
    lines = []
    for path in sorted(paths, key=lambda item: item.relative_to(output_root).as_posix()):
        relative = path.relative_to(output_root).as_posix()
        lines.append(f"{sha256_file(path)}  {relative}")
    temporary = hash_path.with_suffix(hash_path.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(hash_path)
    return hash_path


def analyze_design(dataset_root: Path, output_root: Path | None) -> int:
    dataset_root = dataset_root.resolve()
    checker = DatasetChecker(dataset_root)
    checker.run()
    if checker.errors:
        print_human_report(checker.report())
        print("\nanalyze-design已停止：数据集机械检查未通过。", file=sys.stderr)
        return 1

    output = (
        output_root.resolve()
        if output_root is not None
        else dataset_root / "processed" / "design_analysis"
    )
    output.mkdir(parents=True, exist_ok=True)
    with (dataset_root / "manifest.csv").open(
        "r", encoding="utf-8-sig", newline=""
    ) as manifest_handle:
        manifest_rows = {
            row["segment_id"]: row for row in csv.DictReader(manifest_handle)
        }

    all_samples: list[DesignFeatureSample] = []
    summaries: list[DesignFeatureSummary] = []
    generated_paths: list[Path] = []
    plot_paths: list[Path] = []
    for segment_id, (split, _, _) in EXPECTED_SEGMENTS.items():
        if split != "design":
            continue
        manifest_row = manifest_rows[segment_id]
        samples, sample_deltas = load_design_feature_samples(dataset_root, manifest_row)
        all_samples.extend(samples)
        summaries.append(summarize_design_segment(manifest_row, samples, sample_deltas))
        plot_path = output / "plots" / f"{segment_id}_dynamic.svg"
        render_dynamic_svg(plot_path, segment_id, manifest_row["class"], samples)
        plot_paths.append(plot_path)

    sample_path = output / "design_feature_samples.csv"
    sample_fields = tuple(asdict(all_samples[0]).keys())
    write_csv_atomic(sample_path, sample_fields, (asdict(item) for item in all_samples))
    generated_paths.append(sample_path)

    summary_path = output / "design_segment_summary.csv"
    summary_fields = tuple(asdict(summaries[0]).keys())
    write_csv_atomic(summary_path, summary_fields, (asdict(item) for item in summaries))
    generated_paths.append(summary_path)

    class_rows: list[dict[str, object]] = []
    for sample_class in ("still", "nonstep", "walk"):
        class_samples = [item for item in all_samples if item.sample_class == sample_class]
        dynamic = [item.dynamic_ug for item in class_samples]
        class_summaries = [item for item in summaries if item.sample_class == sample_class]
        class_rows.append(
            {
                "class": sample_class,
                "segment_count": len(class_summaries),
                "sample_count": len(class_samples),
                "reference_count_total": sum(item.reference_count for item in class_summaries),
                "dynamic_mean_ug": round(statistics.fmean(dynamic), 3),
                "dynamic_p50_ug": percentile_nearest_rank(dynamic, 50),
                "dynamic_p75_ug": percentile_nearest_rank(dynamic, 75),
                "dynamic_p90_ug": percentile_nearest_rank(dynamic, 90),
                "dynamic_p95_ug": percentile_nearest_rank(dynamic, 95),
                "dynamic_p99_ug": percentile_nearest_rank(dynamic, 99),
                "dynamic_max_ug": max(dynamic),
            }
        )
    class_path = output / "design_class_summary.csv"
    write_csv_atomic(class_path, tuple(class_rows[0].keys()), class_rows)
    generated_paths.append(class_path)

    metadata = {
        "tool_version": TOOL_VERSION,
        "analysis_scope": "design_only",
        "validation_sensor_values_read": False,
        "dataset_version": "task10_v1",
        "manifest_sha256": sha256_file(dataset_root / "manifest.csv"),
        "input_segments": [item.segment_id for item in summaries],
        "eval_window_ms": [6000, 42000],
        "feature_formula": {
            "a_mag_ug": "isqrt(ax_ug^2 + ay_ug^2 + az_ug^2)",
            "dynamic_ug": "abs(a_mag_ug - 1000000)",
        },
        "percentile_method": "nearest_rank",
        "thresholds_selected": False,
        "algorithms_executed": [],
        "manual_count_limitation": (
            "count_only; confidence=medium; possible_manual_count_error; no event timestamps"
        ),
    }
    metadata_path = output / "design_analysis_metadata.json"
    write_json_report(metadata_path, metadata)
    generated_paths.append(metadata_path)
    generated_paths.extend(plot_paths)
    hash_path = write_analysis_hashes(output, generated_paths)

    print("Design-only analysis: PASS")
    print(f"output: {output}")
    print("validation_sensor_values_read: false")
    print("thresholds_selected: false")
    print()
    print(
        "segment_id        class    rows dt_min dt_mean dt_max "
        "dyn_p50 dyn_p90 dyn_p95 dyn_p99 dyn_max"
    )
    for item in summaries:
        print(
            f"{item.segment_id:<17} {item.sample_class:<8} {item.eval_rows:>4} "
            f"{item.sample_dt_min_ms:>6} {item.sample_dt_mean_ms:>7.3f} "
            f"{item.sample_dt_max_ms:>6} {item.dynamic_p50_ug:>7} "
            f"{item.dynamic_p90_ug:>7} {item.dynamic_p95_ug:>7} "
            f"{item.dynamic_p99_ug:>7} {item.dynamic_max_ug:>7}"
        )
    print(f"\nartifacts: {len(generated_paths)} data/plot files + {hash_path.name}")
    return 0


def rounded_divide_signed(numerator: int, denominator: int) -> int:
    """有符号四舍五入除法，语义可直接移植到后续C实现。"""

    if denominator <= 0:
        raise ValueError("denominator必须大于0")
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def replay_algorithm(
    samples: Sequence[DesignFeatureSample], parameters: AlgorithmParameters
) -> list[AlgorithmEvent]:
    """在单段设计集上运行B0或B1，输出算法事件而非人工事件。"""

    if not samples:
        return []
    events: list[AlgorithmEvent] = []
    filtered = samples[0].dynamic_ug
    previous_time = samples[0].time_ms
    warmup_start = samples[0].time_ms
    last_event_time: int | None = None
    armed = True

    for sample in samples:
        dt_ms = sample.time_ms - previous_time
        if dt_ms < 0:
            raise ValueError(f"{sample.segment_id}: 样本时间倒退")
        if dt_ms > parameters.reset_gap_ms:
            # 长缺口后不能沿用旧滤波和滞回状态，也不能在缺口边界计步。
            filtered = sample.dynamic_ug
            warmup_start = sample.time_ms
            last_event_time = None
            armed = True
        elif parameters.algorithm == "B1" and dt_ms > 0:
            adjustment = rounded_divide_signed(
                (sample.dynamic_ug - filtered) * dt_ms,
                parameters.ewma_tau_ms + dt_ms,
            )
            filtered += adjustment
        else:
            filtered = sample.dynamic_ug
        previous_time = sample.time_ms

        if sample.time_ms - warmup_start < parameters.warmup_ms:
            continue
        refractory_ok = (
            last_event_time is None
            or sample.time_ms - last_event_time >= parameters.refractory_ms
        )

        if parameters.algorithm == "B0":
            # B0没有滞回；持续越阈可能每隔refractory再次计数，这是基线缺陷之一。
            if sample.dynamic_ug >= parameters.high_threshold_ug and refractory_ok:
                events.append(
                    AlgorithmEvent(
                        segment_id=sample.segment_id,
                        algorithm="B0",
                        event_index=len(events) + 1,
                        event_time_ms=sample.time_ms,
                        dynamic_ug=sample.dynamic_ug,
                        filtered_dynamic_ug=sample.dynamic_ug,
                    )
                )
                last_event_time = sample.time_ms
            continue

        if parameters.algorithm != "B1":
            raise ValueError(f"未知算法: {parameters.algorithm}")
        if armed:
            if filtered >= parameters.high_threshold_ug and refractory_ok:
                events.append(
                    AlgorithmEvent(
                        segment_id=sample.segment_id,
                        algorithm="B1",
                        event_index=len(events) + 1,
                        event_time_ms=sample.time_ms,
                        dynamic_ug=sample.dynamic_ug,
                        filtered_dynamic_ug=filtered,
                    )
                )
                last_event_time = sample.time_ms
                armed = False
        elif filtered <= parameters.low_threshold_ug:
            armed = True
    return events


def evaluate_candidate(
    design_samples: dict[str, list[DesignFeatureSample]],
    parameters: AlgorithmParameters,
) -> CandidateResult:
    counts = {
        segment_id: len(replay_algorithm(samples, parameters))
        for segment_id, samples in design_samples.items()
    }
    walk_errors: list[int] = []
    walk_relative_errors: list[float] = []
    still_false_count = 0
    nonstep_false_count = 0
    for segment_id, output_count in counts.items():
        _, sample_class, reference_count = EXPECTED_SEGMENTS[segment_id]
        if sample_class == "walk":
            absolute = abs(output_count - reference_count)
            walk_errors.append(absolute)
            walk_relative_errors.append(absolute / reference_count)
        elif sample_class == "still":
            still_false_count += output_count
        elif sample_class == "nonstep":
            nonstep_false_count += output_count

    walk_abs_total = sum(walk_errors)
    # 静止误计成本最高，非步行误计次之，同时保留步行计数误差。
    score = walk_abs_total + 4 * still_false_count + 2 * nonstep_false_count
    return CandidateResult(
        parameters=parameters,
        segment_counts=counts,
        walk_abs_error_total=walk_abs_total,
        walk_relative_error_mean=round(statistics.fmean(walk_relative_errors), 9),
        walk_max_abs_error=max(walk_errors),
        still_false_count=still_false_count,
        nonstep_false_count=nonstep_false_count,
        score=score,
    )


def candidate_sort_key(result: CandidateResult) -> tuple[object, ...]:
    parameters = result.parameters
    return (
        result.score,
        result.walk_abs_error_total,
        result.still_false_count,
        result.nonstep_false_count,
        result.walk_max_abs_error,
        parameters.high_threshold_ug,
        parameters.low_threshold_ug,
        parameters.refractory_ms,
        parameters.ewma_tau_ms,
    )


def candidate_to_row(
    index: int, result: CandidateResult, design_ids: Sequence[str]
) -> dict[str, object]:
    parameters = result.parameters
    row: dict[str, object] = {
        "candidate_index": index,
        "algorithm": parameters.algorithm,
        "high_threshold_ug": parameters.high_threshold_ug,
        "low_threshold_ug": parameters.low_threshold_ug,
        "refractory_ms": parameters.refractory_ms,
        "ewma_tau_ms": parameters.ewma_tau_ms,
        "warmup_ms": parameters.warmup_ms,
        "reset_gap_ms": parameters.reset_gap_ms,
        "score": result.score,
        "walk_abs_error_total": result.walk_abs_error_total,
        "walk_relative_error_mean": result.walk_relative_error_mean,
        "walk_max_abs_error": result.walk_max_abs_error,
        "still_false_count": result.still_false_count,
        "nonstep_false_count": result.nonstep_false_count,
    }
    for segment_id in design_ids:
        row[f"count_{segment_id}"] = result.segment_counts[segment_id]
    return row


def tune_design(dataset_root: Path, output_root: Path | None) -> int:
    """在design上有限搜索B0/B1；只推荐候选，不冻结也不运行validation。"""

    dataset_root = dataset_root.resolve()
    checker = DatasetChecker(dataset_root)
    checker.run()
    if checker.errors:
        print_human_report(checker.report())
        print("\ntune-design已停止：数据集机械检查未通过。", file=sys.stderr)
        return 1

    output = (
        output_root.resolve()
        if output_root is not None
        else dataset_root / "processed" / "design_tuning"
    )
    output.mkdir(parents=True, exist_ok=True)
    with (dataset_root / "manifest.csv").open(
        "r", encoding="utf-8-sig", newline=""
    ) as manifest_handle:
        manifest_rows = {
            row["segment_id"]: row for row in csv.DictReader(manifest_handle)
        }

    design_ids = [
        segment_id
        for segment_id, (split, _, _) in EXPECTED_SEGMENTS.items()
        if split == "design"
    ]
    design_samples: dict[str, list[DesignFeatureSample]] = {}
    for segment_id in design_ids:
        samples, _ = load_design_feature_samples(
            dataset_root, manifest_rows[segment_id]
        )
        design_samples[segment_id] = samples

    b0_thresholds = (60_000, 80_000, 100_000, 120_000, 150_000, 180_000, 220_000, 260_000, 300_000, 350_000)
    refractory_values = (300, 400, 500, 600, 700)
    b0_results = [
        evaluate_candidate(
            design_samples,
            AlgorithmParameters(
                algorithm="B0",
                high_threshold_ug=threshold,
                low_threshold_ug=0,
                refractory_ms=refractory,
                ewma_tau_ms=0,
            ),
        )
        for threshold, refractory in itertools.product(
            b0_thresholds, refractory_values
        )
    ]

    b1_tau_values = (40, 80, 120, 160, 240)
    b1_thresholds = (40_000, 60_000, 80_000, 100_000, 120_000, 150_000, 180_000, 220_000)
    b1_low_ratios = (40, 50, 60, 70)
    b1_results: list[CandidateResult] = []
    for tau_ms, high, low_ratio, refractory in itertools.product(
        b1_tau_values, b1_thresholds, b1_low_ratios, refractory_values
    ):
        low = high * low_ratio // 100
        b1_results.append(
            evaluate_candidate(
                design_samples,
                AlgorithmParameters(
                    algorithm="B1",
                    high_threshold_ug=high,
                    low_threshold_ug=low,
                    refractory_ms=refractory,
                    ewma_tau_ms=tau_ms,
                ),
            )
        )

    b0_ranked = sorted(b0_results, key=candidate_sort_key)
    b1_ranked = sorted(b1_results, key=candidate_sort_key)
    best_b0 = b0_ranked[0]
    best_b1 = b1_ranked[0]
    recommended = best_b1 if best_b1.score < best_b0.score else best_b0

    generated_paths: list[Path] = []
    for filename, ranked in (
        ("b0_candidates.csv", b0_ranked),
        ("b1_candidates.csv", b1_ranked),
    ):
        rows = [
            candidate_to_row(index, result, design_ids)
            for index, result in enumerate(ranked, start=1)
        ]
        path = output / filename
        write_csv_atomic(path, tuple(rows[0].keys()), rows)
        generated_paths.append(path)

    selected_metrics: list[dict[str, object]] = []
    selected_events: list[dict[str, object]] = []
    for result in (best_b0, best_b1):
        for segment_id in design_ids:
            _, sample_class, reference_count = EXPECTED_SEGMENTS[segment_id]
            output_count = result.segment_counts[segment_id]
            selected_metrics.append(
                {
                    "algorithm": result.parameters.algorithm,
                    "segment_id": segment_id,
                    "class": sample_class,
                    "reference_count": reference_count,
                    "output_count": output_count,
                    "abs_error": abs(output_count - reference_count),
                    "relative_error": (
                        round(abs(output_count - reference_count) / reference_count, 9)
                        if reference_count > 0
                        else "NA"
                    ),
                    "false_count": output_count if reference_count == 0 else "NA",
                }
            )
            for event in replay_algorithm(design_samples[segment_id], result.parameters):
                selected_events.append(asdict(event))

    metrics_path = output / "selected_design_metrics.csv"
    write_csv_atomic(metrics_path, tuple(selected_metrics[0].keys()), selected_metrics)
    generated_paths.append(metrics_path)
    events_path = output / "selected_algorithm_events.csv"
    event_fields = (
        "segment_id",
        "algorithm",
        "event_index",
        "event_time_ms",
        "dynamic_ug",
        "filtered_dynamic_ug",
    )
    write_csv_atomic(events_path, event_fields, selected_events)
    generated_paths.append(events_path)

    selection = {
        "tool_version": TOOL_VERSION,
        "status": "recommended_not_frozen",
        "analysis_scope": "design_only",
        "validation_sensor_values_read": False,
        "validation_algorithms_executed": False,
        "manifest_sha256": sha256_file(dataset_root / "manifest.csv"),
        "feature_formula": {
            "a_mag_ug": "isqrt(ax_ug^2 + ay_ug^2 + az_ug^2)",
            "dynamic_ug": "abs(a_mag_ug - 1000000)",
        },
        "ewma_formula": (
            "filtered += round_signed((dynamic-filtered)*dt_ms/(tau_ms+dt_ms))"
        ),
        "selection_score": (
            "walk_abs_error_total + 4*still_false_count + 2*nonstep_false_count"
        ),
        "tie_break_order": [
            "score",
            "walk_abs_error_total",
            "still_false_count",
            "nonstep_false_count",
            "walk_max_abs_error",
            "lower_parameter_tuple",
        ],
        "fixed_parameters": {"warmup_ms": 200, "reset_gap_ms": 100},
        "candidate_grid": {
            "b0_high_threshold_ug": list(b0_thresholds),
            "b1_high_threshold_ug": list(b1_thresholds),
            "b1_low_ratio_percent": list(b1_low_ratios),
            "b1_ewma_tau_ms": list(b1_tau_values),
            "refractory_ms": list(refractory_values),
        },
        "candidate_counts": {"B0": len(b0_results), "B1": len(b1_results)},
        "best_B0": {
            "parameters": asdict(best_b0.parameters),
            "metrics": {
                "score": best_b0.score,
                "walk_abs_error_total": best_b0.walk_abs_error_total,
                "walk_relative_error_mean": best_b0.walk_relative_error_mean,
                "walk_max_abs_error": best_b0.walk_max_abs_error,
                "still_false_count": best_b0.still_false_count,
                "nonstep_false_count": best_b0.nonstep_false_count,
            },
            "segment_counts": best_b0.segment_counts,
        },
        "best_B1": {
            "parameters": asdict(best_b1.parameters),
            "metrics": {
                "score": best_b1.score,
                "walk_abs_error_total": best_b1.walk_abs_error_total,
                "walk_relative_error_mean": best_b1.walk_relative_error_mean,
                "walk_max_abs_error": best_b1.walk_max_abs_error,
                "still_false_count": best_b1.still_false_count,
                "nonstep_false_count": best_b1.nonstep_false_count,
            },
            "segment_counts": best_b1.segment_counts,
        },
        "recommendation": {
            "algorithm": recommended.parameters.algorithm,
            "reason": (
                "lower_design_score"
                if best_b1.score != best_b0.score
                else "tie_prefers_simpler_B0"
            ),
            "parameters": asdict(recommended.parameters),
            "score": recommended.score,
        },
        "limitations": [
            "count_only labels cannot measure TP, FP, FN, or event latency",
            "manual walk counts have medium confidence and possible count error",
            "one missed event and one false event may cancel in total count",
            "recommendation is not frozen and validation has not been executed",
        ],
    }
    selection_path = output / "design_candidate_selection.json"
    write_json_report(selection_path, selection)
    generated_paths.append(selection_path)
    hash_path = write_analysis_hashes(output, generated_paths)

    print("Design-only tuning: PASS")
    print(f"output: {output}")
    print("validation_sensor_values_read: false")
    print("parameters_frozen: false")
    print(f"B0 candidates: {len(b0_results)}")
    print(f"B1 candidates: {len(b1_results)}")
    for label, result in (("best_B0", best_b0), ("best_B1", best_b1)):
        print(
            f"{label}: score={result.score}, walk_abs={result.walk_abs_error_total}, "
            f"still_false={result.still_false_count}, "
            f"nonstep_false={result.nonstep_false_count}, "
            f"parameters={asdict(result.parameters)}"
        )
        print(f"  counts={result.segment_counts}")
    print(
        f"recommendation: {recommended.parameters.algorithm}, score={recommended.score}, "
        "status=recommended_not_frozen"
    )
    print(f"artifacts: {len(generated_paths)} files + {hash_path.name}")
    return 0


def china_time_iso() -> str:
    """使用项目所在地固定UTC+8偏移，避免依赖主机时区配置。"""

    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")


def write_single_hash_sidecar(path: Path, target: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = f"{sha256_file(target)}  {target.name}\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8", newline="\n")
    temporary.replace(path)


def read_single_hash_sidecar(path: Path, target: Path) -> str:
    if not path.is_file():
        raise ValueError(f"缺少哈希侧车文件: {path}")
    line = path.read_text(encoding="utf-8").strip()
    match = re.fullmatch(r"([0-9A-Fa-f]{64})  (.+)", line)
    if match is None or match.group(2) != target.name:
        raise ValueError(f"哈希侧车格式错误: {path}")
    expected = match.group(1).upper()
    actual = sha256_file(target)
    if actual != expected:
        raise ValueError(f"冻结参数哈希不匹配: expected={expected}, actual={actual}")
    return actual


def freeze_parameters(dataset_root: Path) -> int:
    """把设计集选出的B0/B1一次性冻结；已存在时拒绝覆盖。"""

    dataset_root = dataset_root.resolve()
    checker = DatasetChecker(dataset_root)
    checker.run()
    if checker.errors:
        print_human_report(checker.report())
        print("\nfreeze已停止：数据集机械检查未通过。", file=sys.stderr)
        return 1

    selection_path = (
        dataset_root
        / "processed"
        / "design_tuning"
        / "design_candidate_selection.json"
    )
    seal_path = dataset_root / "validation_seal.txt"
    manifest_path = dataset_root / "manifest.csv"
    firmware_path = dataset_root / "firmware_identity.txt"
    parameter_path = dataset_root / "parameters" / "frozen_parameters.json"
    sidecar_path = dataset_root / "parameters" / "frozen_parameters.sha256"

    for required in (selection_path, seal_path, manifest_path, firmware_path):
        if not required.is_file():
            raise ValueError(f"冻结前缺少文件: {required}")
    if parameter_path.exists() or sidecar_path.exists():
        raise ValueError("冻结参数或侧车哈希已存在；拒绝覆盖")

    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    if selection.get("status") != "recommended_not_frozen":
        raise ValueError("设计集选择文件状态不是recommended_not_frozen")
    if selection.get("analysis_scope") != "design_only":
        raise ValueError("设计集选择文件作用域异常")
    if selection.get("validation_sensor_values_read") is not False:
        raise ValueError("设计集选择文件不能证明validation未读")
    if selection.get("validation_algorithms_executed") is not False:
        raise ValueError("设计集选择文件表明validation算法已运行")
    if selection.get("tool_version") != TOOL_VERSION:
        raise ValueError(
            "设计集选择文件工具版本与当前工具不一致；先用当前工具重跑tune-design"
        )
    if selection.get("manifest_sha256") != sha256_file(manifest_path):
        raise ValueError("设计集选择文件中的manifest哈希已失效")

    with manifest_path.open("r", encoding="utf-8-sig", newline="") as handle:
        manifest_rows = list(csv.DictReader(handle))
    dataset_versions = {row["dataset_version"] for row in manifest_rows}
    if len(dataset_versions) != 1:
        raise ValueError("manifest包含多个dataset_version")

    frozen = {
        "freeze_schema_version": 1,
        "algorithm_version": "task10_step_detector_v1",
        "freeze_status": "frozen_before_first_validation",
        "freeze_time": china_time_iso(),
        "dataset_version": next(iter(dataset_versions)),
        "input_unit": "ug",
        "nominal_period_ms": 20,
        "evaluation_window": "manifest.eval_start_ms <= time_ms < manifest.eval_end_ms",
        "feature_formula": selection["feature_formula"],
        "ewma_formula": selection["ewma_formula"],
        "algorithms": {
            "B0": selection["best_B0"]["parameters"],
            "B1": selection["best_B1"]["parameters"],
        },
        "selection_score": selection["selection_score"],
        "tie_break_order": selection["tie_break_order"],
        "design_metrics": {
            "B0": selection["best_B0"]["metrics"],
            "B1": selection["best_B1"]["metrics"],
        },
        "parameter_source": [
            "physical_step_frequency_constraints",
            "design_set_feature_distribution",
            "finite_candidate_comparison",
        ],
        "manual_label_mode": "count_only",
        "manual_walk_count_confidence": "medium_possible_count_error",
        "design_result_hash": sha256_file(selection_path),
        "manifest_sha256": sha256_file(manifest_path),
        "firmware_identity_sha256": sha256_file(firmware_path),
        "validation_seal_sha256": sha256_file(seal_path),
        "replay_tool_version": TOOL_VERSION,
        "replay_tool_sha256": sha256_file(Path(__file__).resolve()),
        "parameter_file_sha256": "RECORDED_IN_frozen_parameters.sha256",
        "first_validation_executed": False,
    }
    write_json_report(parameter_path, frozen)
    write_single_hash_sidecar(sidecar_path, parameter_path)
    frozen_hash = read_single_hash_sidecar(sidecar_path, parameter_path)
    print("Parameter freeze: PASS")
    print(f"parameters: {parameter_path}")
    print(f"parameter_sha256: {frozen_hash}")
    print("validation_executed: false")
    return 0


def algorithm_parameters_from_dict(
    algorithm: str, payload: dict[str, object]
) -> AlgorithmParameters:
    required = {
        "algorithm",
        "high_threshold_ug",
        "low_threshold_ug",
        "refractory_ms",
        "ewma_tau_ms",
        "warmup_ms",
        "reset_gap_ms",
    }
    if set(payload) != required or payload.get("algorithm") != algorithm:
        raise ValueError(f"冻结参数中的{algorithm}字段不完整或算法名不一致")
    return AlgorithmParameters(
        algorithm=algorithm,
        high_threshold_ug=int(payload["high_threshold_ug"]),
        low_threshold_ug=int(payload["low_threshold_ug"]),
        refractory_ms=int(payload["refractory_ms"]),
        ewma_tau_ms=int(payload["ewma_tau_ms"]),
        warmup_ms=int(payload["warmup_ms"]),
        reset_gap_ms=int(payload["reset_gap_ms"]),
    )


def validate_first(dataset_root: Path) -> int:
    """使用冻结参数首次读取validation；结果和运行标记均拒绝覆盖。"""

    dataset_root = dataset_root.resolve()
    checker = DatasetChecker(dataset_root)
    checker.run()
    if checker.errors:
        print_human_report(checker.report())
        print("\nvalidate-first已停止：数据集机械检查未通过。", file=sys.stderr)
        return 1

    parameter_path = dataset_root / "parameters" / "frozen_parameters.json"
    sidecar_path = dataset_root / "parameters" / "frozen_parameters.sha256"
    selection_path = (
        dataset_root
        / "processed"
        / "design_tuning"
        / "design_candidate_selection.json"
    )
    manifest_path = dataset_root / "manifest.csv"
    firmware_path = dataset_root / "firmware_identity.txt"
    seal_path = dataset_root / "validation_seal.txt"
    results_dir = dataset_root / "results"
    metrics_path = results_dir / "validation_first_run.csv"
    events_path = results_dir / "validation_first_run_events.csv"
    summary_path = results_dir / "validation_first_run_summary.json"
    hashes_path = results_dir / "validation_first_run.sha256"
    attempt_path = results_dir / "validation_first_run_attempt.json"
    protected_outputs = (
        metrics_path,
        events_path,
        summary_path,
        hashes_path,
        attempt_path,
    )

    if not parameter_path.is_file():
        raise ValueError("冻结参数不存在，拒绝运行validation")
    for path in protected_outputs:
        if path.exists():
            raise ValueError(f"首次验证证据已存在，拒绝覆盖: {path}")

    parameter_hash = read_single_hash_sidecar(sidecar_path, parameter_path)
    frozen = json.loads(parameter_path.read_text(encoding="utf-8"))
    current_hashes = {
        "design_result_hash": sha256_file(selection_path),
        "manifest_sha256": sha256_file(manifest_path),
        "firmware_identity_sha256": sha256_file(firmware_path),
        "validation_seal_sha256": sha256_file(seal_path),
        "replay_tool_sha256": sha256_file(Path(__file__).resolve()),
    }
    for field, actual in current_hashes.items():
        if frozen.get(field) != actual:
            raise ValueError(f"冻结后{field}发生变化，拒绝运行validation")
    if frozen.get("freeze_status") != "frozen_before_first_validation":
        raise ValueError("冻结状态错误")
    if frozen.get("first_validation_executed") is not False:
        raise ValueError("冻结文件表明首次validation已执行")

    algorithms_payload = frozen.get("algorithms")
    if not isinstance(algorithms_payload, dict):
        raise ValueError("冻结参数缺少algorithms")
    parameters = {
        name: algorithm_parameters_from_dict(name, algorithms_payload[name])
        for name in ("B0", "B1")
    }
    with manifest_path.open("r", encoding="utf-8-sig", newline="") as handle:
        manifest_rows = {
            row["segment_id"]: row for row in csv.DictReader(handle)
        }
    validation_ids = [
        segment_id
        for segment_id, (split, _, _) in EXPECTED_SEGMENTS.items()
        if split == "validation"
    ]
    if set(validation_ids) != {
        segment_id
        for segment_id, row in manifest_rows.items()
        if row["split"] == "validation"
    }:
        raise ValueError("validation片段集合与冻结定义不一致")

    # 该标记必须先于任何validation传感器数值读取落盘。
    attempt = {
        "status": "started",
        "started_time": china_time_iso(),
        "dataset_version": frozen["dataset_version"],
        "parameter_sha256": parameter_hash,
        "validation_seal_sha256": current_hashes["validation_seal_sha256"],
        "replay_tool_sha256": current_hashes["replay_tool_sha256"],
        "validation_segments": validation_ids,
        "note": "created before first validation sensor value was read",
    }
    write_json_report(attempt_path, attempt)

    try:
        validation_samples: dict[str, list[DesignFeatureSample]] = {}
        for segment_id in validation_ids:
            samples, _ = load_validation_feature_samples(
                dataset_root, manifest_rows[segment_id]
            )
            validation_samples[segment_id] = samples

        metric_rows: list[dict[str, object]] = []
        event_rows: list[dict[str, object]] = []
        algorithm_summaries: dict[str, dict[str, object]] = {}
        for algorithm in ("B0", "B1"):
            walk_abs_error_total = 0
            still_false_count = 0
            nonstep_false_count = 0
            segment_counts: dict[str, int] = {}
            for segment_id in validation_ids:
                events = replay_algorithm(
                    validation_samples[segment_id], parameters[algorithm]
                )
                _, sample_class, reference_count = EXPECTED_SEGMENTS[segment_id]
                output_count = len(events)
                segment_counts[segment_id] = output_count
                absolute_error = abs(output_count - reference_count)
                relative_error: object = (
                    round(absolute_error / reference_count, 9)
                    if reference_count > 0
                    else "NA"
                )
                false_count: object = output_count if reference_count == 0 else "NA"
                if sample_class == "walk":
                    walk_abs_error_total += absolute_error
                elif sample_class == "still":
                    still_false_count += output_count
                elif sample_class == "nonstep":
                    nonstep_false_count += output_count
                metric_rows.append(
                    {
                        "segment_id": segment_id,
                        "algorithm": algorithm,
                        "class": sample_class,
                        "reference_count": reference_count,
                        "output_count": output_count,
                        "abs_error": absolute_error,
                        "relative_error": relative_error,
                        "false_count": false_count,
                    }
                )
                event_rows.extend(asdict(event) for event in events)

            score = (
                walk_abs_error_total
                + 4 * still_false_count
                + 2 * nonstep_false_count
            )
            algorithm_summaries[algorithm] = {
                "parameters": asdict(parameters[algorithm]),
                "score": score,
                "walk_abs_error_total": walk_abs_error_total,
                "still_false_count": still_false_count,
                "nonstep_false_count": nonstep_false_count,
                "segment_counts": segment_counts,
            }

        metric_fields = (
            "segment_id",
            "algorithm",
            "class",
            "reference_count",
            "output_count",
            "abs_error",
            "relative_error",
            "false_count",
        )
        event_fields = (
            "segment_id",
            "algorithm",
            "event_index",
            "event_time_ms",
            "dynamic_ug",
            "filtered_dynamic_ug",
        )
        write_csv_atomic(metrics_path, metric_fields, metric_rows)
        write_csv_atomic(events_path, event_fields, event_rows)
        b0_score = int(algorithm_summaries["B0"]["score"])
        b1_score = int(algorithm_summaries["B1"]["score"])
        comparison = (
            "B1_improved"
            if b1_score < b0_score
            else "B1_tied"
            if b1_score == b0_score
            else "B1_regressed"
        )
        summary = {
            "validation_schema_version": 1,
            "status": "first_independent_validation_complete",
            "completed_time": china_time_iso(),
            "dataset_version": frozen["dataset_version"],
            "parameter_sha256": parameter_hash,
            "validation_seal_sha256": current_hashes["validation_seal_sha256"],
            "replay_tool_version": TOOL_VERSION,
            "replay_tool_sha256": current_hashes["replay_tool_sha256"],
            "selection_score": frozen["selection_score"],
            "algorithms": algorithm_summaries,
            "comparison": comparison,
            "limitations": [
                "count_only labels cannot measure TP, FP, FN, or event latency",
                "manual walk count has medium confidence and possible count error",
                "validation contains one segment per class",
                "offline validation does not prove embedded real-time behavior",
            ],
        }
        write_json_report(summary_path, summary)
        result_paths = (metrics_path, events_path, summary_path)
        hash_lines = [
            f"{sha256_file(path)}  {path.name}" for path in sorted(result_paths)
        ]
        hashes_path.write_text(
            "\n".join(hash_lines) + "\n", encoding="utf-8", newline="\n"
        )

        attempt.update(
            {
                "status": "completed",
                "completed_time": summary["completed_time"],
                "result_inventory_sha256": sha256_file(hashes_path),
                "comparison": comparison,
            }
        )
        write_json_report(attempt_path, attempt)
    except Exception as error:
        attempt.update(
            {
                "status": "failed_after_validation_access_started",
                "failed_time": china_time_iso(),
                "error_type": type(error).__name__,
                "error": str(error),
            }
        )
        write_json_report(attempt_path, attempt)
        raise

    print("First independent validation: PASS (execution completed)")
    print(f"parameters_sha256: {parameter_hash}")
    for algorithm in ("B0", "B1"):
        result = algorithm_summaries[algorithm]
        print(
            f"{algorithm}: score={result['score']}, "
            f"walk_abs={result['walk_abs_error_total']}, "
            f"still_false={result['still_false_count']}, "
            f"nonstep_false={result['nonstep_false_count']}, "
            f"counts={result['segment_counts']}"
        )
    print(f"comparison: {comparison}")
    print(f"results: {results_dir}")
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SF32LB52 SmartWatch任务10 IMU数据集离线工具"
    )
    parser.add_argument("--version", action="version", version=TOOL_VERSION)
    subparsers = parser.add_subparsers(dest="command", required=True)

    check_parser = subparsers.add_parser(
        "check", help="机械检查数据身份、格式、tick、序号和哈希"
    )
    check_parser.add_argument(
        "--dataset", required=True, type=Path, help="dataset目录的绝对或相对路径"
    )
    check_parser.add_argument(
        "--json-out",
        type=Path,
        help="可选：把机械检查结果写入JSON；不会包含传感器数值统计",
    )

    analyze_parser = subparsers.add_parser(
        "analyze-design",
        help="只分析9段设计集，输出特征摘要和SVG，不运行计步算法",
    )
    analyze_parser.add_argument(
        "--dataset", required=True, type=Path, help="dataset目录的绝对或相对路径"
    )
    analyze_parser.add_argument(
        "--output",
        type=Path,
        help="可选输出目录；默认dataset/processed/design_analysis",
    )

    tune_parser = subparsers.add_parser(
        "tune-design",
        help="只用9段设计集比较B0/B1有限候选，给出未冻结推荐",
    )
    tune_parser.add_argument(
        "--dataset", required=True, type=Path, help="dataset目录的绝对或相对路径"
    )
    tune_parser.add_argument(
        "--output",
        type=Path,
        help="可选输出目录；默认dataset/processed/design_tuning",
    )

    freeze_parser = subparsers.add_parser(
        "freeze", help="冻结设计集选出的B0/B1参数并生成独立哈希"
    )
    freeze_parser.add_argument(
        "--dataset", required=True, type=Path, help="dataset目录的绝对或相对路径"
    )

    validate_parser = subparsers.add_parser(
        "validate-first", help="使用冻结参数首次运行独立验证集，拒绝覆盖"
    )
    validate_parser.add_argument(
        "--dataset", required=True, type=Path, help="dataset目录的绝对或相对路径"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.command == "check":
        checker = DatasetChecker(args.dataset)
        checker.run()
        report = checker.report()
        print_human_report(report)
        if args.json_out is not None:
            write_json_report(args.json_out, report)
            print(f"json_report: {args.json_out.resolve()}")
        return 0 if not checker.errors else 1
    if args.command == "analyze-design":
        return analyze_design(args.dataset, args.output)
    if args.command == "tune-design":
        return tune_design(args.dataset, args.output)
    if args.command == "freeze":
        return freeze_parameters(args.dataset)
    if args.command == "validate-first":
        return validate_first(args.dataset)
    raise AssertionError(f"未处理的命令: {args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中止。", file=sys.stderr)
        raise SystemExit(130)
