#!/usr/bin/env python3
"""从任务10设计集导出目标板计步算法黄金向量。"""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path

TOOL_VERSION = "task10_golden_export_v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def require_int(row: dict[str, str], field: str) -> int:
    try:
        return int(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"字段{field}缺失或不是整数：{row!r}") from error


def main() -> int:
    parser = argparse.ArgumentParser(description="导出任务10目标板计步黄金向量")
    parser.add_argument("--samples", required=True, type=Path)
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--segment", default="D_WALK_02")
    parser.add_argument("--start-ms", type=int, default=6000)
    parser.add_argument("--end-ms", type=int, default=10600)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    samples_path = args.samples.resolve()
    events_path = args.events.resolve()
    output_path = args.output.resolve()

    if not args.segment.startswith("D_"):
        raise ValueError("黄金向量只能从设计集D_*生成，禁止读取validation")
    if args.start_ms > args.end_ms:
        raise ValueError("start-ms不能大于end-ms")
    if (args.end_ms - args.start_ms) % 20 != 0:
        raise ValueError("黄金窗口长度必须是20 ms的整数倍")

    all_samples = read_csv(samples_path)
    samples = [
        row
        for row in all_samples
        if row.get("segment_id") == args.segment
        and args.start_ms <= require_int(row, "time_ms") <= args.end_ms
    ]
    samples.sort(key=lambda row: require_int(row, "time_ms"))

    expected_count = ((args.end_ms - args.start_ms) // 20) + 1
    if len(samples) != expected_count:
        raise ValueError(f"样本数错误：实际{len(samples)}，预期{expected_count}")

    for index, row in enumerate(samples):
        expected_time = args.start_ms + index * 20
        actual_time = require_int(row, "time_ms")
        if actual_time != expected_time:
            raise ValueError(
                f"第{index}行时间错误：实际{actual_time}，预期{expected_time}"
            )
        if index > 0:
            previous = samples[index - 1]
            tick_delta = (
                require_int(row, "sample_tick")
                - require_int(previous, "sample_tick")
            ) & 0xFFFFFFFF
            seq_delta = (
                require_int(row, "imu_seq")
                - require_int(previous, "imu_seq")
            ) & 0xFFFFFFFF
            if tick_delta != 20:
                raise ValueError(
                    f"第{index}行sample_tick差值为{tick_delta}，预期20"
                )
            if seq_delta != 1:
                raise ValueError(f"第{index}行imu_seq差值为{seq_delta}，预期1")

    all_events = read_csv(events_path)
    selected_events: dict[str, list[dict[str, str]]] = {"B0": [], "B1": []}
    for row in all_events:
        if row.get("segment_id") != args.segment:
            continue
        algorithm = row.get("algorithm")
        if algorithm not in selected_events:
            continue
        event_time = require_int(row, "event_time_ms")
        if args.start_ms <= event_time <= args.end_ms:
            selected_events[algorithm].append(row)

    for algorithm in ("B0", "B1"):
        selected_events[algorithm].sort(
            key=lambda row: require_int(row, "event_time_ms")
        )
        if len(selected_events[algorithm]) != 3:
            raise ValueError(
                f"当前冻结窗口的{algorithm}事件数为"
                f"{len(selected_events[algorithm])}，预期3"
            )

    samples_sha256 = sha256_file(samples_path)
    events_sha256 = sha256_file(events_path)
    lines: list[str] = [
        "#ifndef STEP_DETECTOR_GOLDEN_VECTORS_H",
        "#define STEP_DETECTOR_GOLDEN_VECTORS_H",
        "",
        "#include <stdint.h>",
        "",
        f'#define STEP_GOLDEN_TOOL_VERSION "{TOOL_VERSION}"',
        f'#define STEP_GOLDEN_SEGMENT_ID "{args.segment}"',
        f"#define STEP_GOLDEN_START_MS {args.start_ms}U",
        f"#define STEP_GOLDEN_END_MS {args.end_ms}U",
        f"#define STEP_GOLDEN_SAMPLE_COUNT {len(samples)}U",
        f'#define STEP_GOLDEN_SAMPLES_SHA256 "{samples_sha256}"',
        f'#define STEP_GOLDEN_EVENTS_SHA256 "{events_sha256}"',
        "",
        "typedef struct",
        "{",
        "    uint32_t time_ms;",
        "    uint32_t sample_tick;",
        "    uint32_t imu_seq;",
        "    int32_t ax_ug;",
        "    int32_t ay_ug;",
        "    int32_t az_ug;",
        "    uint32_t expected_magnitude_ug;",
        "    uint32_t expected_dynamic_ug;",
        "} step_detector_golden_sample_t;",
        "",
        "typedef struct",
        "{",
        "    uint32_t time_ms;",
        "    uint32_t dynamic_ug;",
        "    uint32_t filtered_dynamic_ug;",
        "} step_detector_golden_event_t;",
        "",
        "static const step_detector_golden_sample_t",
        "    g_step_detector_golden_samples[] =",
        "{",
    ]

    for row in samples:
        lines.append(
            "    {"
            f"{require_int(row, 'time_ms')}U,"
            f"{require_int(row, 'sample_tick')}U,"
            f"{require_int(row, 'imu_seq')}U,"
            f"{require_int(row, 'ax_ug')},"
            f"{require_int(row, 'ay_ug')},"
            f"{require_int(row, 'az_ug')},"
            f"{require_int(row, 'a_mag_ug')}U,"
            f"{require_int(row, 'dynamic_ug')}U"
            "},"
        )

    lines.extend(["};", ""])
    for algorithm in ("B0", "B1"):
        array_name = algorithm.lower()
        lines.extend(
            [
                "static const step_detector_golden_event_t",
                f"    g_step_detector_golden_{array_name}_events[] =",
                "{",
            ]
        )
        for row in selected_events[algorithm]:
            lines.append(
                "    {"
                f"{require_int(row, 'event_time_ms')}U,"
                f"{require_int(row, 'dynamic_ug')}U,"
                f"{require_int(row, 'filtered_dynamic_ug')}U"
                "},"
            )
        lines.extend(
            [
                "};",
                "",
                f"#define STEP_GOLDEN_{algorithm}_EVENT_COUNT "
                f"{len(selected_events[algorithm])}U",
                "",
            ]
        )

    lines.extend(["#endif /* STEP_DETECTOR_GOLDEN_VECTORS_H */", ""])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text("\n".join(lines), encoding="utf-8", newline="\n")
    temporary.replace(output_path)

    print(
        "golden export complete:",
        f"segment={args.segment}",
        f"samples={len(samples)}",
        f"B0_events={len(selected_events['B0'])}",
        f"B1_events={len(selected_events['B1'])}",
        f"output={output_path}",
    )
    print(f"samples_sha256={samples_sha256}")
    print(f"events_sha256={events_sha256}")
    print(f"output_sha256={sha256_file(output_path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
