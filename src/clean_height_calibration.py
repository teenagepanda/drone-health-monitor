from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable

PRIMARY_METRICS = (
    "avg_marker_width_px",
    "avg_marker_height_px",
    "avg_marker_area_px2",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect outliers in height_calibration_summary.csv and create a "
            "clean calibration file with one aggregated row per height."
        )
    )
    parser.add_argument("--input", default="reports/height_calibration_summary.csv")
    parser.add_argument("--output", default="reports/height_calibration_clean.csv")
    parser.add_argument(
        "--outliers-output",
        default="reports/height_calibration_outliers.csv",
    )
    parser.add_argument("--test-type", default="center")
    parser.add_argument("--mad-z", type=float, default=3.5)
    parser.add_argument("--min-confidence", type=float, default=70.0)
    parser.add_argument("--min-samples", type=int, default=20)
    return parser.parse_args()


def to_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        number = float(value.strip())
    except (ValueError, AttributeError):
        return None
    return number if math.isfinite(number) else None


def modified_z_scores(values: list[float]) -> list[float]:
    if len(values) < 3:
        return [0.0] * len(values)
    median = statistics.median(values)
    deviations = [abs(value - median) for value in values]
    mad = statistics.median(deviations)
    if mad == 0:
        nonzero = [d for d in deviations if d > 0]
        if not nonzero:
            return [0.0] * len(values)
        scale = statistics.median(nonzero)
        return [(value - median) / scale for value in values]
    return [0.6745 * (value - median) / mad for value in values]


def weighted_mean(rows: list[dict[str, str]], field: str) -> float:
    total = 0.0
    weight_sum = 0.0
    for row in rows:
        value = to_float(row.get(field))
        if value is None:
            continue
        weight = to_float(row.get("samples")) or 1.0
        total += value * weight
        weight_sum += weight
    return total / weight_sum if weight_sum else 0.0


def pooled_stdev(
    rows: list[dict[str, str]], mean_field: str, stdev_field: str
) -> float:
    items: list[tuple[float, int, float]] = []
    for row in rows:
        mean_value = to_float(row.get(mean_field))
        if mean_value is None:
            continue
        count = max(int(to_float(row.get("samples")) or 1), 1)
        stdev = max(to_float(row.get(stdev_field)) or 0.0, 0.0)
        items.append((mean_value, count, stdev * stdev))
    if not items:
        return 0.0
    total_n = sum(count for _, count, _ in items)
    combined_mean = sum(mean * count for mean, count, _ in items) / total_n
    numerator = sum(
        max(count - 1, 0) * variance + count * (mean - combined_mean) ** 2
        for mean, count, variance in items
    )
    return math.sqrt(numerator / max(total_n - 1, 1))


def load_rows(path: Path, test_type: str) -> tuple[list[str], list[dict[str, str]]]:
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {path}")
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("CSV has no header")
        required = {
            "real_height_m",
            "test_type",
            "samples",
            "avg_marker_width_px",
            "avg_marker_height_px",
            "avg_marker_area_px2",
            "avg_confidence_percent",
        }
        missing = sorted(required - set(reader.fieldnames))
        if missing:
            raise ValueError("Missing required columns: " + ", ".join(missing))
        rows = [
            dict(row)
            for row in reader
            if row.get("test_type", "").strip().lower() == test_type.lower()
        ]
        return list(reader.fieldnames), rows


def basic_rejection_reason(
    row: dict[str, str], min_confidence: float, min_samples: int
) -> str | None:
    checks = {
        "real_height_m": to_float(row.get("real_height_m")),
        "avg_marker_width_px": to_float(row.get("avg_marker_width_px")),
        "avg_marker_height_px": to_float(row.get("avg_marker_height_px")),
        "avg_marker_area_px2": to_float(row.get("avg_marker_area_px2")),
    }
    for name, value in checks.items():
        if value is None or value <= 0:
            return f"invalid {name}"
    confidence = to_float(row.get("avg_confidence_percent"))
    if confidence is None or confidence < min_confidence:
        return f"confidence below {min_confidence:.1f}%"
    samples = to_float(row.get("samples"))
    if samples is None or samples < min_samples:
        return f"samples below {min_samples}"
    return None


def aggregate_group(
    rows: list[dict[str, str]], fieldnames: list[str]
) -> dict[str, str]:
    base = dict(rows[-1])
    base["samples"] = str(sum(int(to_float(r.get("samples")) or 0) for r in rows))
    pairs = (
        ("avg_marker_width_px", "stdev_marker_width_px"),
        ("avg_marker_height_px", "stdev_marker_height_px"),
        ("avg_marker_area_px2", "stdev_marker_area_px2"),
        ("avg_confidence_percent", "stdev_confidence_percent"),
    )
    for mean_field, stdev_field in pairs:
        if mean_field in fieldnames:
            base[mean_field] = f"{weighted_mean(rows, mean_field):.10g}"
        if stdev_field in fieldnames:
            base[stdev_field] = f"{pooled_stdev(rows, mean_field, stdev_field):.10g}"
    for field in ("avg_fps", "avg_processing_time_ms"):
        if field in fieldnames:
            base[field] = f"{weighted_mean(rows, field):.10g}"
    return {field: base.get(field, "") for field in fieldnames}


def write_csv(
    path: Path, fieldnames: list[str], rows: Iterable[dict[str, str]]
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    fieldnames, rows = load_rows(Path(args.input), args.test_type)
    if not rows:
        print(f"No rows found for test_type={args.test_type!r}")
        return 1

    accepted: list[dict[str, str]] = []
    rejected: list[dict[str, str]] = []
    for row in rows:
        reason = basic_rejection_reason(row, args.min_confidence, args.min_samples)
        if reason:
            rejected.append({**row, "rejection_reason": reason})
        else:
            accepted.append(row)

    groups: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in accepted:
        groups[round(float(row["real_height_m"]), 6)].append(row)

    final_groups: dict[float, list[dict[str, str]]] = {}
    for height, group in sorted(groups.items()):
        keep = [True] * len(group)
        reasons: list[list[str]] = [[] for _ in group]
        if len(group) >= 3:
            for metric in PRIMARY_METRICS:
                values = [float(row[metric]) for row in group]
                scores = modified_z_scores(values)
                for index, score in enumerate(scores):
                    if abs(score) > args.mad_z:
                        keep[index] = False
                        reasons[index].append(f"{metric} modified-z={score:.2f}")
        kept_rows: list[dict[str, str]] = []
        for index, row in enumerate(group):
            if keep[index]:
                kept_rows.append(row)
            else:
                rejected.append(
                    {**row, "rejection_reason": "; ".join(reasons[index])}
                )
        final_groups[height] = kept_rows

    clean_rows = [
        aggregate_group(group, fieldnames)
        for _, group in sorted(final_groups.items())
        if group
    ]
    write_csv(Path(args.output), fieldnames, clean_rows)
    write_csv(
        Path(args.outliers_output),
        fieldnames + ["rejection_reason"],
        rejected,
    )

    print("=" * 76)
    print("HEIGHT CALIBRATION CLEANING COMPLETE")
    print(f"Input rows:          {len(rows)}")
    print(f"Rejected rows:       {len(rejected)}")
    print(f"Clean height points: {len(clean_rows)}")
    print(f"Clean file:          {args.output}")
    print(f"Outlier audit:       {args.outliers_output}")
    print("-" * 76)
    print(f"{'Height(m)':>10} {'Runs':>6} {'Width(px)':>12} {'Height(px)':>12} {'Area(px^2)':>13}")
    for height, group in sorted(final_groups.items()):
        if not group:
            continue
        row = aggregate_group(group, fieldnames)
        print(
            f"{height:10.2f} {len(group):6d} "
            f"{float(row['avg_marker_width_px']):12.1f} "
            f"{float(row['avg_marker_height_px']):12.1f} "
            f"{float(row['avg_marker_area_px2']):13.0f}"
        )
    if rejected:
        print("-" * 76)
        print("Review the outlier audit before using the clean file for flight.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
