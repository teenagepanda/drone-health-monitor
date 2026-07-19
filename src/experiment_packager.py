
from __future__ import annotations

import csv
import shutil
import statistics
import time
import zipfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class ExperimentPackageResult:
    experiment_dir: Path
    zip_path: Path
    copied_images: int
    plots_created: int


def _safe_float(value):
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _read_csv_rows(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8-sig") as csv_file:
        return list(csv.DictReader(csv_file))


def _copy_file_if_exists(source: Path, destination: Path) -> bool:
    if not source.exists() or not source.is_file():
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return True


def _copy_recent_images(
    source_dir: Path,
    destination_dir: Path,
    started_at_epoch: float,
) -> list[Path]:
    copied = []
    if not source_dir.exists():
        return copied

    destination_dir.mkdir(parents=True, exist_ok=True)
    for source in sorted(source_dir.rglob("*.jpg")):
        try:
            if source.stat().st_mtime + 1 < started_at_epoch:
                continue
        except OSError:
            continue

        relative = source.relative_to(source_dir)
        target = destination_dir / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied.append(target)
    return copied


def _select_representative_images(
    rows: list[dict],
    destination_dir: Path,
) -> list[Path]:
    destination_dir.mkdir(parents=True, exist_ok=True)

    valid = []
    for index, row in enumerate(rows):
        image_path = row.get("image_path", "").strip()
        if not image_path:
            continue
        source = Path(image_path)
        if not source.exists():
            continue
        valid.append(
            {
                "index": index,
                "row": row,
                "source": source,
                "confidence": _safe_float(row.get("confidence_percent")),
                "error": _safe_float(row.get("error_distance_cm")),
            }
        )

    if not valid:
        return []

    selected = {
        "first": valid[0],
        "last": valid[-1],
    }

    confidence_valid = [item for item in valid if item["confidence"] is not None]
    if confidence_valid:
        selected["best_confidence"] = max(
            confidence_valid, key=lambda item: item["confidence"]
        )

    error_valid = [item for item in valid if item["error"] is not None]
    if error_valid:
        selected["best_centering"] = min(error_valid, key=lambda item: item["error"])
        selected["worst_centering"] = max(error_valid, key=lambda item: item["error"])

    copied = []
    seen = set()
    for label, item in selected.items():
        source = item["source"]
        if source.resolve() in seen:
            continue
        seen.add(source.resolve())
        target = destination_dir / f"{label}_{source.name}"
        shutil.copy2(source, target)
        copied.append(target)

    return copied


def _plot_series(
    rows: list[dict],
    column: str,
    title: str,
    ylabel: str,
    output: Path,
) -> bool:
    values = []
    sample_numbers = []
    for index, row in enumerate(rows, start=1):
        value = _safe_float(row.get(column))
        if value is None:
            continue
        sample_numbers.append(index)
        values.append(value)

    if not values:
        return False

    output.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111)
    ax.plot(sample_numbers, values, marker="o", markersize=3)
    ax.set_title(title)
    ax.set_xlabel("Sample")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return True


def _write_summary(
    rows: list[dict],
    summary_path: Path,
    run_metadata: dict,
) -> dict:
    def values(column: str) -> list[float]:
        result = []
        for row in rows:
            value = _safe_float(row.get(column))
            if value is not None:
                result.append(value)
        return result

    metrics = {
        "confidence_percent": values("confidence_percent"),
        "offset_x_px": values("offset_x_px"),
        "offset_y_px": values("offset_y_px"),
        "error_distance_px": values("error_distance_px"),
        "offset_x_cm": values("offset_x_cm"),
        "offset_y_cm": values("offset_y_cm"),
        "error_distance_cm": values("error_distance_cm"),
        "fps": values("fps"),
        "processing_time_ms": values("processing_time_ms"),
    }

    summary = dict(run_metadata)
    summary["samples"] = len(rows)

    for name, series in metrics.items():
        if not series:
            continue
        summary[f"avg_{name}"] = statistics.mean(series)
        summary[f"min_{name}"] = min(series)
        summary[f"max_{name}"] = max(series)
        summary[f"stdev_{name}"] = (
            statistics.stdev(series) if len(series) > 1 else 0.0
        )

    lines = [
        f"Experiment ID: {summary.get('experiment_id', '')}",
        f"Software version: {summary.get('software_version', '')}",
        f"Created: {summary.get('created_at', '')}",
        f"Test type: {summary.get('test_type', '')}",
        f"Real height: {summary.get('real_height_m', '')} m",
        f"Marker size: {summary.get('marker_size_cm', '')} cm",
        f"Samples: {summary.get('samples', 0)}",
        "",
    ]

    label_map = {
        "avg_confidence_percent": "Average confidence (%)",
        "avg_offset_x_px": "Average X offset (px)",
        "avg_offset_y_px": "Average Y offset (px)",
        "avg_error_distance_px": "Average radial error (px)",
        "avg_offset_x_cm": "Average X offset (cm)",
        "avg_offset_y_cm": "Average Y offset (cm)",
        "avg_error_distance_cm": "Average radial error (cm)",
        "max_error_distance_cm": "Maximum radial error (cm)",
        "avg_fps": "Average FPS",
        "avg_processing_time_ms": "Average processing time (ms)",
    }

    for key, label in label_map.items():
        if key in summary:
            lines.append(f"{label}: {summary[key]:.3f}")

    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def _write_one_row_csv(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def package_experiment(
    reports_dir: str | Path,
    started_at_epoch: float,
    software_version: str,
    test_type: str,
    real_height_m: Optional[float],
    marker_size_cm: float,
    raw_csv: str | Path,
    summary_csv: str | Path,
    landing_csv: str | Path,
    calibration_image_run_dir: Optional[str | Path],
    marker_image_dir: str | Path,
    detected_frame_path: str | Path,
) -> ExperimentPackageResult:
    reports_dir = Path(reports_dir)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    experiment_id = f"experiment_{stamp}"
    experiment_dir = reports_dir / experiment_id

    data_dir = experiment_dir / "data"
    all_images_dir = experiment_dir / "images" / "all"
    representative_dir = experiment_dir / "images" / "representative"
    plots_dir = experiment_dir / "plots"

    for directory in [data_dir, all_images_dir, representative_dir, plots_dir]:
        directory.mkdir(parents=True, exist_ok=True)

    raw_csv = Path(raw_csv)
    summary_csv = Path(summary_csv)
    landing_csv = Path(landing_csv)

    _copy_file_if_exists(raw_csv, data_dir / raw_csv.name)
    _copy_file_if_exists(summary_csv, data_dir / summary_csv.name)
    _copy_file_if_exists(landing_csv, data_dir / landing_csv.name)
    _copy_file_if_exists(
        Path(detected_frame_path),
        experiment_dir / "images" / "marker_detected.jpg",
    )

    copied_images = []

    if calibration_image_run_dir:
        calibration_dir = Path(calibration_image_run_dir)
        if calibration_dir.exists():
            for source in sorted(calibration_dir.glob("*.jpg")):
                target = all_images_dir / "calibration" / source.name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
                copied_images.append(target)

    copied_images.extend(
        _copy_recent_images(
            Path(marker_image_dir),
            all_images_dir / "marker_detections",
            started_at_epoch,
        )
    )

    raw_rows = _read_csv_rows(raw_csv)
    run_rows = []
    for row in raw_rows:
        timestamp_text = row.get("timestamp", "")
        try:
            row_time = datetime.fromisoformat(timestamp_text).timestamp()
        except Exception:
            row_time = None
        if row_time is None or row_time + 1 >= started_at_epoch:
            run_rows.append(row)

    representative_images = _select_representative_images(
        run_rows, representative_dir
    )
    copied_images.extend(representative_images)

    plots_created = 0
    plot_specs = [
        ("offset_x_cm", "Marker X Offset", "Offset X (cm)", "offset_x_cm.png"),
        ("offset_y_cm", "Marker Y Offset", "Offset Y (cm)", "offset_y_cm.png"),
        (
            "error_distance_cm",
            "Radial Centering Error",
            "Error distance (cm)",
            "error_distance_cm.png",
        ),
        (
            "confidence_percent",
            "Detection Confidence",
            "Confidence (%)",
            "confidence.png",
        ),
        ("fps", "Processing Frame Rate", "FPS", "fps.png"),
        (
            "processing_time_ms",
            "Detection Processing Time",
            "Processing time (ms)",
            "processing_time_ms.png",
        ),
    ]

    for column, title, ylabel, filename in plot_specs:
        if _plot_series(run_rows, column, title, ylabel, plots_dir / filename):
            plots_created += 1

    run_metadata = {
        "experiment_id": experiment_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "software_version": software_version,
        "test_type": test_type,
        "real_height_m": "" if real_height_m is None else real_height_m,
        "marker_size_cm": marker_size_cm,
    }

    summary = _write_summary(
        run_rows,
        experiment_dir / "summary.txt",
        run_metadata,
    )
    _write_one_row_csv(
        experiment_dir / "experiment_summary.csv",
        summary,
    )

    manifest_lines = [
        f"Experiment directory: {experiment_dir}",
        f"Raw samples included: {len(run_rows)}",
        f"Images copied: {len(copied_images)}",
        f"Plots created: {plots_created}",
        "",
        "Contents:",
    ]
    for path in sorted(experiment_dir.rglob("*")):
        if path.is_file():
            manifest_lines.append(str(path.relative_to(experiment_dir)))
    (experiment_dir / "manifest.txt").write_text(
        "\n".join(manifest_lines) + "\n",
        encoding="utf-8",
    )

    zip_path = reports_dir / f"{experiment_id}.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(experiment_dir.rglob("*")):
            if path.is_file():
                archive.write(path, arcname=str(path.relative_to(reports_dir)))

    return ExperimentPackageResult(
        experiment_dir=experiment_dir,
        zip_path=zip_path,
        copied_images=len(copied_images),
        plots_created=plots_created,
    )
