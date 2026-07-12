from __future__ import annotations

import argparse

from summary_calibration import SummaryCalibrationTable


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate V14 summary CSV calibration."
    )
    parser.add_argument("summary_csv")
    parser.add_argument("--marker-size", type=float, required=True)
    parser.add_argument(
        "--heights",
        type=float,
        nargs="+",
        default=[0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00],
    )
    args = parser.parse_args()

    table = SummaryCalibrationTable.from_summary_csv(
        args.summary_csv,
        marker_size_m=args.marker_size,
    )

    print(
        f"Loaded {len(table.points)} center heights "
        f"({table.min_height_m:.2f}-{table.max_height_m:.2f} m)"
    )

    print("\nSelected center rows:")
    for point in table.points:
        print(
            f"{point.height_m:.2f} m | "
            f"{point.marker_width_px:.2f} x {point.marker_height_px:.2f} px | "
            f"{point.px_per_meter_x:.2f} / "
            f"{point.px_per_meter_y:.2f} px/m | "
            f"{point.timestamp}"
        )

    print("\nInterpolation check:")
    for height in args.heights:
        value = table.interpolate(height)
        print(
            f"{height:.2f} m -> "
            f"X={value.px_per_meter_x:.2f} px/m, "
            f"Y={value.px_per_meter_y:.2f} px/m, "
            f"applied={value.applied_height_m:.2f} m, "
            f"clamped={value.clamped}"
        )


if __name__ == "__main__":
    main()
