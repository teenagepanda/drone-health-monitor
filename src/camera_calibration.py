from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


@dataclass(frozen=True)
class CalibrationPoint:
    height_m: float
    meters_per_pixel: float
    average_side_px: float
    samples: int = 0


class CameraCalibration:
    """Load and validate height-calibration data for visual landing.

    V16 accepts only center-series rows that:
    - use the ``original`` reference image;
    - have average confidence of at least 90%;
    - contain positive height and marker-size values.

    The detected marker size must decrease as camera height increases. If the
    table is not monotonic, loading fails and the landing controller falls back
    to the original pixel-based gain instead of using unsafe calibration data.
    """

    def __init__(
        self,
        summary_csv: str | Path,
        marker_size_m: float,
        accepted_test_types: Optional[Iterable[str]] = ("center",),
        required_reference: str = "original",
        min_confidence_percent: float = 90.0,
        require_monotonic: bool = True,
    ) -> None:
        if marker_size_m <= 0:
            raise ValueError("marker_size_m must be greater than zero")
        if not 0 <= min_confidence_percent <= 100:
            raise ValueError("min_confidence_percent must be between 0 and 100")

        self.summary_csv = Path(summary_csv)
        self.marker_size_m = float(marker_size_m)
        self.accepted_test_types = (
            None
            if accepted_test_types is None
            else {str(value).strip().lower() for value in accepted_test_types}
        )
        self.required_reference = required_reference.strip().lower()
        self.min_confidence_percent = float(min_confidence_percent)
        self.require_monotonic = bool(require_monotonic)
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
                ratio = (height - lower.height_m) / span
                return lower.meters_per_pixel + ratio * (
                    upper.meters_per_pixel - lower.meters_per_pixel
                )

        return self.points[-1].meters_per_pixel

    def describe(self) -> str:
        if not self.points:
            return f"no valid points loaded from {self.summary_csv}"
        return (
            f"{len(self.points)} validated height points from "
            f"{self.min_height_m:.2f} m to {self.max_height_m:.2f} m, "
            f"reference={self.required_reference}, "
            f"minimum confidence={self.min_confidence_percent:.1f}%, "
            f"marker size={self.marker_size_m:.3f} m"
        )

    def _load_points(self) -> list[CalibrationPoint]:
        if not self.summary_csv.exists():
            return []

        grouped: dict[float, list[tuple[float, int]]] = {}
        with self.summary_csv.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            required = {
                "real_height_m",
                "test_type",
                "reference",
                "samples",
                "avg_marker_width_px",
                "avg_marker_height_px",
                "avg_confidence_percent",
            }
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise ValueError(
                    "Calibration CSV is missing columns: " + ", ".join(sorted(missing))
                )

            for row in reader:
                try:
                    test_type = str(row.get("test_type", "")).strip().lower()
                    if (
                        self.accepted_test_types is not None
                        and test_type not in self.accepted_test_types
                    ):
                        continue

                    reference = str(row.get("reference", "")).strip().lower()
                    if reference != self.required_reference:
                        continue

                    confidence = float(row["avg_confidence_percent"])
                    if confidence < self.min_confidence_percent:
                        continue

                    height_m = float(row["real_height_m"])
                    width_px = float(row["avg_marker_width_px"])
                    height_px = float(row["avg_marker_height_px"])
                    samples = int(float(row.get("samples", 0) or 0))
                    average_side_px = (width_px + height_px) / 2.0
                    if height_m <= 0 or average_side_px <= 0:
                        continue

                    weight = max(samples, 1)
                    grouped.setdefault(height_m, []).append((average_side_px, weight))
                except (TypeError, ValueError, KeyError):
                    continue

        points: list[CalibrationPoint] = []
        for height_m, values in grouped.items():
            total_weight = sum(weight for _, weight in values)
            average_side_px = (
                sum(side_px * weight for side_px, weight in values) / total_weight
            )
            points.append(
                CalibrationPoint(
                    height_m=height_m,
                    meters_per_pixel=self.marker_size_m / average_side_px,
                    average_side_px=average_side_px,
                    samples=total_weight,
                )
            )

        points.sort(key=lambda point: point.height_m)
        if self.require_monotonic:
            self._validate_monotonic(points)
        return points

    @staticmethod
    def _validate_monotonic(points: list[CalibrationPoint]) -> None:
        problems: list[str] = []
        for lower, upper in zip(points, points[1:]):
            if upper.average_side_px >= lower.average_side_px:
                problems.append(
                    f"{lower.height_m:.2f} m={lower.average_side_px:.1f}px, "
                    f"{upper.height_m:.2f} m={upper.average_side_px:.1f}px"
                )
        if problems:
            raise ValueError(
                "Calibration is not monotonic; repeat the listed measurements: "
                + "; ".join(problems)
            )
