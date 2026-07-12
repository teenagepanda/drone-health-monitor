from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


@dataclass(frozen=True)
class CalibrationPoint:
    height_m: float
    average_side_px: float
    meters_per_pixel: float
    samples: int = 0


class CameraCalibration:
    """Load a clean or summary height-calibration CSV.

    Supported inputs:
    - reports/height_calibration_clean.csv
    - reports/height_calibration_summary.csv

    Optional columns such as ``reference`` and ``avg_confidence_percent`` are
    validated when present. This keeps the clean aggregated file compatible
    even when it does not contain a reference column.
    """

    def __init__(
        self,
        csv_path: str | Path,
        marker_size_m: float,
        accepted_test_types: Optional[Iterable[str]] = ("center",),
        required_reference: Optional[str] = "original",
        min_confidence_percent: float = 85.0,
        require_monotonic: bool = True,
    ) -> None:
        if marker_size_m <= 0:
            raise ValueError("marker_size_m must be greater than zero")
        if not 0 <= min_confidence_percent <= 100:
            raise ValueError("min_confidence_percent must be between 0 and 100")

        self.csv_path = Path(csv_path)
        self.marker_size_m = float(marker_size_m)
        self.accepted_test_types = (
            None if accepted_test_types is None
            else {str(v).strip().lower() for v in accepted_test_types}
        )
        self.required_reference = (
            None if required_reference is None else required_reference.strip().lower()
        )
        self.min_confidence_percent = float(min_confidence_percent)
        self.require_monotonic = bool(require_monotonic)
        self.points = self._load_points()

    @property
    def available(self) -> bool:
        return len(self.points) >= 2

    @property
    def min_height_m(self) -> Optional[float]:
        return self.points[0].height_m if self.points else None

    @property
    def max_height_m(self) -> Optional[float]:
        return self.points[-1].height_m if self.points else None

    @property
    def max_side_px(self) -> Optional[float]:
        return self.points[0].average_side_px if self.points else None

    @property
    def min_side_px(self) -> Optional[float]:
        return self.points[-1].average_side_px if self.points else None

    def meters_per_pixel(self, height_m: float) -> float:
        """Interpolate image scale by measured camera height."""
        point = self._interpolate_by_height(float(height_m))
        return point.meters_per_pixel

    def expected_marker_side_px(self, height_m: float) -> float:
        """Interpolate expected marker side length in pixels."""
        point = self._interpolate_by_height(float(height_m))
        return point.average_side_px

    def estimate_height_m(self, marker_side_px: float) -> float:
        """Estimate camera-to-marker height from the detected marker size."""
        side = float(marker_side_px)
        if side <= 0:
            raise ValueError("marker_side_px must be positive")
        if not self.points:
            raise RuntimeError("No calibration points loaded")

        # Larger marker in the image means a lower camera height.
        if side >= self.points[0].average_side_px:
            return self.points[0].height_m
        if side <= self.points[-1].average_side_px:
            return self.points[-1].height_m

        for lower, upper in zip(self.points, self.points[1:]):
            high_px = lower.average_side_px
            low_px = upper.average_side_px
            if high_px >= side >= low_px:
                px_span = high_px - low_px
                ratio = 0.0 if px_span == 0 else (high_px - side) / px_span
                return lower.height_m + ratio * (upper.height_m - lower.height_m)

        return self.points[-1].height_m

    def describe(self) -> str:
        if not self.points:
            return f"no valid points loaded from {self.csv_path}"
        return (
            f"{len(self.points)} height points, "
            f"{self.min_height_m:.2f}-{self.max_height_m:.2f} m, "
            f"marker side {self.min_side_px:.1f}-{self.max_side_px:.1f} px, "
            f"marker size={self.marker_size_m:.3f} m"
        )

    def _interpolate_by_height(self, height_m: float) -> CalibrationPoint:
        if not self.points:
            raise RuntimeError("No valid camera calibration points are loaded")
        if height_m <= 0:
            raise ValueError("height_m must be positive")

        if height_m <= self.points[0].height_m:
            return self.points[0]
        if height_m >= self.points[-1].height_m:
            return self.points[-1]

        for lower, upper in zip(self.points, self.points[1:]):
            if lower.height_m <= height_m <= upper.height_m:
                span = upper.height_m - lower.height_m
                ratio = (height_m - lower.height_m) / span
                side = lower.average_side_px + ratio * (
                    upper.average_side_px - lower.average_side_px
                )
                return CalibrationPoint(
                    height_m=height_m,
                    average_side_px=side,
                    meters_per_pixel=self.marker_size_m / side,
                    samples=0,
                )
        return self.points[-1]

    def _load_points(self) -> list[CalibrationPoint]:
        if not self.csv_path.exists():
            return []

        grouped: dict[float, list[tuple[float, int]]] = {}
        with self.csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or [])
            required = {
                "real_height_m",
                "avg_marker_width_px",
                "avg_marker_height_px",
            }
            missing = required - fields
            if missing:
                raise ValueError(
                    "Calibration CSV is missing columns: " + ", ".join(sorted(missing))
                )

            for row in reader:
                try:
                    if "test_type" in fields and self.accepted_test_types is not None:
                        test_type = str(row.get("test_type", "")).strip().lower()
                        if test_type not in self.accepted_test_types:
                            continue

                    if (
                        "reference" in fields
                        and self.required_reference is not None
                        and str(row.get("reference", "")).strip().lower()
                        != self.required_reference
                    ):
                        continue

                    if "avg_confidence_percent" in fields:
                        confidence = float(row["avg_confidence_percent"])
                        # 0.25 m may intentionally use 85%; the clean file has
                        # already passed the operator's calibration filtering.
                        if confidence < self.min_confidence_percent:
                            continue

                    height = float(row["real_height_m"])
                    width = float(row["avg_marker_width_px"])
                    marker_h = float(row["avg_marker_height_px"])
                    samples = int(float(row.get("samples", 1) or 1))
                    side = (width + marker_h) / 2.0
                    if height <= 0 or side <= 0:
                        continue
                    grouped.setdefault(height, []).append((side, max(samples, 1)))
                except (ValueError, TypeError, KeyError):
                    continue

        points: list[CalibrationPoint] = []
        for height, values in grouped.items():
            weight_sum = sum(weight for _, weight in values)
            side = sum(value * weight for value, weight in values) / weight_sum
            points.append(
                CalibrationPoint(
                    height_m=height,
                    average_side_px=side,
                    meters_per_pixel=self.marker_size_m / side,
                    samples=weight_sum,
                )
            )

        points.sort(key=lambda p: p.height_m)
        if self.require_monotonic:
            self._validate_monotonic(points)
        return points

    @staticmethod
    def _validate_monotonic(points: list[CalibrationPoint]) -> None:
        problems = []
        for lower, upper in zip(points, points[1:]):
            if upper.average_side_px >= lower.average_side_px:
                problems.append(
                    f"{lower.height_m:.2f}m={lower.average_side_px:.1f}px -> "
                    f"{upper.height_m:.2f}m={upper.average_side_px:.1f}px"
                )
        if problems:
            raise ValueError(
                "Calibration marker size must decrease with height: "
                + "; ".join(problems)
            )
