from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


@dataclass(frozen=True)
class CalibrationPoint:
    height_m: float
    meters_per_pixel: float
    samples: int = 0


class CameraCalibration:
    """Load height calibration data and convert camera pixels to real-world meters.

    The calibration summary contains the detected marker size in pixels at each
    measured camera height. Knowing the marker's real side length allows:

        meters_per_pixel = marker_size_m / average_marker_side_px

    Values between measured heights are linearly interpolated. Values outside
    the measured range are clamped to the nearest valid calibration point.
    """

    def __init__(
        self,
        summary_csv: str | Path,
        marker_size_m: float,
        accepted_test_types: Optional[Iterable[str]] = ("center",),
    ) -> None:
        if marker_size_m <= 0:
            raise ValueError("marker_size_m must be greater than zero")

        self.summary_csv = Path(summary_csv)
        self.marker_size_m = float(marker_size_m)
        self.accepted_test_types = (
            None if accepted_test_types is None else {str(value).strip().lower() for value in accepted_test_types}
        )
        self.points = self._load_points()

    @property
    def available(self) -> bool:
        return bool(self.points)

    @property
    def min_height_m(self) -> Optional[float]:
        return self.points[0].height_m if self.points else None

    @property
    def max_height_m(self) -> Optional[float]:
        return self.points[-1].height_m if self.points else None

    def meters_per_pixel(self, height_m: float) -> float:
        if not self.points:
            raise RuntimeError("No valid camera calibration points are loaded")
        if height_m is None or height_m <= 0:
            raise ValueError("height_m must be a positive value")

        height = float(height_m)
        if height <= self.points[0].height_m:
            return self.points[0].meters_per_pixel
        if height >= self.points[-1].height_m:
            return self.points[-1].meters_per_pixel

        for lower, upper in zip(self.points, self.points[1:]):
            if lower.height_m <= height <= upper.height_m:
                span = upper.height_m - lower.height_m
                if span <= 0:
                    return lower.meters_per_pixel
                ratio = (height - lower.height_m) / span
                return lower.meters_per_pixel + ratio * (
                    upper.meters_per_pixel - lower.meters_per_pixel
                )

        return self.points[-1].meters_per_pixel

    def describe(self) -> str:
        if not self.points:
            return f"no valid points loaded from {self.summary_csv}"
        return (
            f"{len(self.points)} height points from {self.min_height_m:.2f} m "
            f"to {self.max_height_m:.2f} m, marker size={self.marker_size_m:.3f} m"
        )

    def _load_points(self) -> list[CalibrationPoint]:
        if not self.summary_csv.exists():
            return []

        grouped: dict[float, list[tuple[float, int]]] = {}
        with self.summary_csv.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            required = {"real_height_m", "avg_marker_width_px", "avg_marker_height_px"}
            if not required.issubset(reader.fieldnames or []):
                return []

            for row in reader:
                try:
                    test_type = str(row.get("test_type", "")).strip().lower()
                    if self.accepted_test_types is not None and test_type not in self.accepted_test_types:
                        continue

                    height_m = float(row["real_height_m"])
                    width_px = float(row["avg_marker_width_px"])
                    height_px = float(row["avg_marker_height_px"])
                    samples = int(float(row.get("samples", 0) or 0))
                    average_side_px = (width_px + height_px) / 2.0
                    if height_m <= 0 or average_side_px <= 0:
                        continue

                    meters_per_pixel = self.marker_size_m / average_side_px
                    weight = max(samples, 1)
                    grouped.setdefault(height_m, []).append((meters_per_pixel, weight))
                except (TypeError, ValueError, KeyError):
                    continue

        points: list[CalibrationPoint] = []
        for height_m, values in grouped.items():
            total_weight = sum(weight for _, weight in values)
            weighted_mpp = sum(mpp * weight for mpp, weight in values) / total_weight
            points.append(
                CalibrationPoint(
                    height_m=height_m,
                    meters_per_pixel=weighted_mpp,
                    samples=total_weight,
                )
            )

        points.sort(key=lambda point: point.height_m)
        return points
