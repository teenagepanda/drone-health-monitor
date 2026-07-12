from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CalibrationPoint:
    timestamp: str
    height_m: float
    px_per_meter_x: float
    px_per_meter_y: float
    marker_width_px: float
    marker_height_px: float


@dataclass(frozen=True)
class InterpolatedCalibration:
    requested_height_m: float
    applied_height_m: float
    px_per_meter_x: float
    px_per_meter_y: float
    clamped: bool


class SummaryCalibrationTable:
    REQUIRED_COLUMNS = {
        "timestamp",
        "real_height_m",
        "test_type",
        "avg_marker_width_px",
        "avg_marker_height_px",
    }

    def __init__(self, points: Iterable[CalibrationPoint]) -> None:
        ordered = sorted(points, key=lambda point: point.height_m)
        if not ordered:
            raise ValueError(
                "No usable center calibration rows were found in the summary CSV."
            )
        if any(point.height_m <= 0 for point in ordered):
            raise ValueError("All real_height_m values must be positive.")
        if any(
            point.px_per_meter_x <= 0 or point.px_per_meter_y <= 0
            for point in ordered
        ):
            raise ValueError("Calculated pixels-per-meter values must be positive.")

        heights = [point.height_m for point in ordered]
        if len(heights) != len(set(heights)):
            raise ValueError("Duplicate heights remained after selecting newest rows.")

        self.points = ordered

    @classmethod
    def from_summary_csv(
        cls,
        path: str | Path,
        marker_size_m: float,
    ) -> "SummaryCalibrationTable":
        if marker_size_m <= 0:
            raise ValueError("marker_size_m must be positive.")

        csv_path = Path(path)
        if not csv_path.exists():
            raise FileNotFoundError(f"Calibration summary not found: {csv_path}")

        newest_by_height: dict[float, CalibrationPoint] = {}
        newest_sort_key: dict[float, tuple[int, str]] = {}

        with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            headers = set(reader.fieldnames or [])
            missing = cls.REQUIRED_COLUMNS - headers
            if missing:
                raise ValueError(
                    "Calibration summary is missing columns: "
                    + ", ".join(sorted(missing))
                )

            for line_number, row in enumerate(reader, start=2):
                test_type = (row.get("test_type") or "").strip().lower()
                if test_type != "center":
                    continue

                try:
                    height_m = float(row["real_height_m"])
                    marker_width_px = float(row["avg_marker_width_px"])
                    marker_height_px = float(row["avg_marker_height_px"])
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Invalid numeric value on summary CSV line {line_number}: {exc}"
                    ) from exc

                if height_m <= 0 or marker_width_px <= 0 or marker_height_px <= 0:
                    continue

                timestamp = (row.get("timestamp") or "").strip()
                sort_key = _timestamp_sort_key(timestamp)

                point = CalibrationPoint(
                    timestamp=timestamp,
                    height_m=height_m,
                    px_per_meter_x=marker_width_px / marker_size_m,
                    px_per_meter_y=marker_height_px / marker_size_m,
                    marker_width_px=marker_width_px,
                    marker_height_px=marker_height_px,
                )

                previous_key = newest_sort_key.get(height_m)
                if previous_key is None or sort_key >= previous_key:
                    newest_by_height[height_m] = point
                    newest_sort_key[height_m] = sort_key

        return cls(newest_by_height.values())

    @property
    def min_height_m(self) -> float:
        return self.points[0].height_m

    @property
    def max_height_m(self) -> float:
        return self.points[-1].height_m

    def interpolate(self, height_m: float) -> InterpolatedCalibration:
        if height_m <= 0:
            raise ValueError("Requested height must be positive.")

        heights = np.array([point.height_m for point in self.points], dtype=float)
        x_values = np.array(
            [point.px_per_meter_x for point in self.points], dtype=float
        )
        y_values = np.array(
            [point.px_per_meter_y for point in self.points], dtype=float
        )

        applied_height = float(np.clip(height_m, heights[0], heights[-1]))
        clamped = not np.isclose(height_m, applied_height)

        return InterpolatedCalibration(
            requested_height_m=height_m,
            applied_height_m=applied_height,
            px_per_meter_x=float(np.interp(applied_height, heights, x_values)),
            px_per_meter_y=float(np.interp(applied_height, heights, y_values)),
            clamped=clamped,
        )

    def pixel_offset_to_meters(
        self,
        marker_center_x_px: float,
        marker_center_y_px: float,
        frame_width_px: int,
        frame_height_px: int,
        height_m: float,
    ) -> tuple[float, float, InterpolatedCalibration]:
        calibration = self.interpolate(height_m)

        frame_center_x = frame_width_px / 2.0
        frame_center_y = frame_height_px / 2.0

        offset_x_px = marker_center_x_px - frame_center_x
        offset_y_px = marker_center_y_px - frame_center_y

        offset_x_m = offset_x_px / calibration.px_per_meter_x
        offset_y_m = offset_y_px / calibration.px_per_meter_y

        return offset_x_m, offset_y_m, calibration


def _timestamp_sort_key(value: str) -> tuple[int, str]:
    if not value:
        return (0, "")
    normalized = value.replace("Z", "+00:00")
    try:
        parsed = datetime.fromisoformat(normalized)
        return (1, parsed.isoformat())
    except ValueError:
        return (0, value)
