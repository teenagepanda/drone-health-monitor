import argparse
import csv
import math
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2

from camera_capture import CameraCapture
from landing_controller import LandingCommand, VisualLandingController
from experiment_packager import package_experiment

VERSION = "V24"


def _marker_geometry(corners, frame_shape):
    pts = corners.reshape(-1, 2)
    side_lengths = [
        float(((pts[(i + 1) % 4] - pts[i]) ** 2).sum() ** 0.5)
        for i in range(4)
    ]
    marker_width_px = (side_lengths[0] + side_lengths[2]) / 2.0
    marker_height_px = (side_lengths[1] + side_lengths[3]) / 2.0
    marker_area_px2 = float(cv2.contourArea(corners.astype("float32")))
    frame_h, frame_w = frame_shape[:2]
    center_x = int(pts[:, 0].mean())
    center_y = int(pts[:, 1].mean())
    frame_center_x = frame_w // 2
    frame_center_y = frame_h // 2
    offset_x_px = center_x - frame_center_x
    offset_y_px = center_y - frame_center_y
    return {
        "marker_width_px": marker_width_px,
        "marker_height_px": marker_height_px,
        "marker_area_px2": marker_area_px2,
        "marker_center_x_px": center_x,
        "marker_center_y_px": center_y,
        "frame_center_x_px": frame_center_x,
        "frame_center_y_px": frame_center_y,
        "offset_x_px": offset_x_px,
        "offset_y_px": offset_y_px,
        "error_distance_px": math.hypot(offset_x_px, offset_y_px),
    }


def _draw_centering_overlay(
    frame,
    detection,
    marker_size_cm: float,
    visual_alt_m: float | None = None,
    centered_threshold_cm: float = 2.0,
):
    """Draw a compact live overlay with centering status and direction arrow."""
    shown = frame.copy()
    frame_h, frame_w = shown.shape[:2]
    frame_center = (frame_w // 2, frame_h // 2)

    # Subtle frame-center guides.
    cv2.line(
        shown,
        (frame_center[0], 0),
        (frame_center[0], frame_h - 1),
        (145, 145, 145),
        1,
    )
    cv2.line(
        shown,
        (0, frame_center[1]),
        (frame_w - 1, frame_center[1]),
        (145, 145, 145),
        1,
    )
    cv2.circle(shown, frame_center, 7, (255, 255, 255), 2)

    status_text = "SEARCHING"
    status_color = (0, 165, 255)
    confidence_text = "Confidence: N/A"
    offset_x_text = "X: N/A"
    offset_y_text = "Y: N/A"
    center_error_text = "Error: N/A"
    direction_text = "Direction: N/A"
    direction_vector = None

    if detection.detected and detection.corners is not None:
        geometry = _marker_geometry(detection.corners, shown.shape)
        marker_center = (
            int(geometry["marker_center_x_px"]),
            int(geometry["marker_center_y_px"]),
        )

        cm_per_px = (
            marker_size_cm / geometry["marker_width_px"]
            if geometry["marker_width_px"] > 0
            else None
        )
        offset_x_cm = (
            geometry["offset_x_px"] * cm_per_px if cm_per_px is not None else None
        )
        offset_y_cm = (
            geometry["offset_y_px"] * cm_per_px if cm_per_px is not None else None
        )
        error_cm = (
            geometry["error_distance_px"] * cm_per_px if cm_per_px is not None else None
        )

        centered = error_cm is not None and error_cm <= centered_threshold_cm
        if centered:
            status_text = "CENTERED"
            status_color = (0, 200, 0)
        elif error_cm is not None and error_cm <= centered_threshold_cm * 2.5:
            status_text = "ALMOST CENTERED"
            status_color = (0, 165, 255)
        else:
            status_text = "NOT CENTERED"
            status_color = (0, 0, 255)

        cv2.circle(shown, marker_center, 7, status_color, 2)
        cv2.line(shown, frame_center, marker_center, status_color, 2)

        if offset_x_cm is not None:
            offset_x_text = f"X: {offset_x_cm:+.2f} cm"
        if offset_y_cm is not None:
            offset_y_text = f"Y: {offset_y_cm:+.2f} cm"
        if error_cm is not None:
            center_error_text = f"Error: {error_cm:.2f} cm"

        confidence = float(getattr(detection, "score", 0.0)) * 100.0
        confidence_text = f"Confidence: {confidence:.1f}%"

        # The arrow indicates the correction needed to move the camera/frame
        # center toward the detected marker center.
        dx = geometry["offset_x_px"]
        dy = geometry["offset_y_px"]
        deadband_px = max(8, int(min(frame_w, frame_h) * 0.015))

        horizontal = ""
        vertical = ""
        if dx > deadband_px:
            horizontal = "RIGHT"
        elif dx < -deadband_px:
            horizontal = "LEFT"

        if dy > deadband_px:
            vertical = "DOWN"
        elif dy < -deadband_px:
            vertical = "UP"

        if horizontal and vertical:
            direction_text = f"Move: {vertical}-{horizontal}"
        elif horizontal:
            direction_text = f"Move: {horizontal}"
        elif vertical:
            direction_text = f"Move: {vertical}"
        else:
            direction_text = "Move: HOLD"

        direction_vector = (-dx, -dy)

    # Compact status panel at the top-left.
    panel_lines = []
    if visual_alt_m is not None:
        panel_lines.append(f"Altitude: {visual_alt_m:.2f} m")
    panel_lines.extend(
        [
            confidence_text,
            offset_x_text,
            offset_y_text,
            center_error_text,
            direction_text,
        ]
    )

    x0, y0 = 12, 12
    line_h = 24
    panel_w = min(frame_w - 24, 320)
    panel_h = 18 + line_h * len(panel_lines)

    panel = shown.copy()
    cv2.rectangle(panel, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(panel, 0.58, shown, 0.42, 0, shown)

    for index, line in enumerate(panel_lines):
        y = y0 + 22 + index * line_h
        cv2.putText(
            shown,
            line,
            (x0 + 10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # Large status badge in a separate area to avoid text overlap.
    badge_text = status_text
    badge_scale = 0.62
    badge_thickness = 2
    (badge_w, badge_h), _ = cv2.getTextSize(
        badge_text,
        cv2.FONT_HERSHEY_SIMPLEX,
        badge_scale,
        badge_thickness,
    )
    badge_x = 12
    badge_y = min(frame_h - 16, y0 + panel_h + 34)
    cv2.rectangle(
        shown,
        (badge_x, badge_y - badge_h - 10),
        (badge_x + badge_w + 20, badge_y + 8),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        shown,
        badge_text,
        (badge_x + 10, badge_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        badge_scale,
        status_color,
        badge_thickness,
        cv2.LINE_AA,
    )

    # Direction arrow at the frame center.
    if direction_vector is not None and status_text != "CENTERED":
        dx, dy = direction_vector
        magnitude = max(1.0, (dx * dx + dy * dy) ** 0.5)
        arrow_length = min(95.0, max(38.0, magnitude * 0.45))
        end_x = int(frame_center[0] + arrow_length * dx / magnitude)
        end_y = int(frame_center[1] + arrow_length * dy / magnitude)
        cv2.arrowedLine(
            shown,
            frame_center,
            (end_x, end_y),
            status_color,
            4,
            tipLength=0.28,
        )

    return shown


def _read_csv_header(path: Path) -> list[str]:
    """Return the first CSV row, or an empty list for a missing/empty file."""
    if not path.exists() or path.stat().st_size == 0:
        return []

    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.reader(csv_file)
        try:
            return [cell.strip() for cell in next(reader)]
        except StopIteration:
            return []


def _rotate_incompatible_csv(path: Path, expected_fields: list[str], label: str) -> Path | None:
    """Rename an existing CSV when its header does not match the current schema.

    The old file is preserved with a timestamp suffix. A new CSV will be created
    automatically by the normal append function.
    """
    header = _read_csv_header(path)
    if not header or header == expected_fields:
        return None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rotated = path.with_name(f"{path.stem}_incompatible_{timestamp}{path.suffix}")

    counter = 1
    while rotated.exists():
        rotated = path.with_name(
            f"{path.stem}_incompatible_{timestamp}_{counter}{path.suffix}"
        )
        counter += 1

    path.rename(rotated)
    print(
        f"CSV SCHEMA MISMATCH | {label} | expected {len(expected_fields)} fields, "
        f"found {len(header)}. Old file moved to: {rotated}"
    )
    return rotated


def _ensure_calibration_csv_compatibility(
    raw_path: Path,
    summary_path: Path,
) -> None:
    """Protect calibration output from mixed schemas across software versions."""
    _rotate_incompatible_csv(raw_path, CALIBRATION_RAW_FIELDS, "raw calibration")

    # The summary schema is generated from the current V21.1 summary dictionary.
    # We only validate its core prefix here because older valid runs may contain
    # extra summary metrics added by a compatible release.
    summary_header = _read_csv_header(summary_path)
    required_summary_prefix = [
        "timestamp",
        "real_height_m",
        "test_type",
        "planned_offset_x_cm",
        "planned_offset_y_cm",
        "samples",
    ]
    if summary_header and summary_header[: len(required_summary_prefix)] != required_summary_prefix:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        rotated = summary_path.with_name(
            f"{summary_path.stem}_incompatible_{timestamp}{summary_path.suffix}"
        )
        counter = 1
        while rotated.exists():
            rotated = summary_path.with_name(
                f"{summary_path.stem}_incompatible_{timestamp}_{counter}{summary_path.suffix}"
            )
            counter += 1
        summary_path.rename(rotated)
        print(
            "CSV SCHEMA MISMATCH | calibration summary | "
            f"old file moved to: {rotated}"
        )



def _resolve_calibration_reference(source: str | Path, reference_name: str) -> Path:
    """Return the single reference image used for height calibration.

    Height calibration must not mix detections from fallback references such as
    A, B, C, D or E. If *source* is a directory, locate exactly one image whose
    filename stem matches *reference_name* (case-insensitive).
    """
    source_path = Path(source)
    if source_path.is_file():
        if source_path.stem.lower() != reference_name.lower():
            raise ValueError(
                f"Calibration reference must be '{reference_name}', got: {source_path.name}"
            )
        return source_path

    if not source_path.is_dir():
        raise FileNotFoundError(f"Reference path not found: {source_path}")

    allowed = {".png", ".jpg", ".jpeg", ".bmp"}
    matches = [
        path for path in source_path.iterdir()
        if path.is_file()
        and path.suffix.lower() in allowed
        and path.stem.lower() == reference_name.lower()
    ]
    if not matches:
        raise FileNotFoundError(
            f"Calibration reference '{reference_name}' was not found in: {source_path}"
        )
    if len(matches) > 1:
        names = ", ".join(sorted(path.name for path in matches))
        raise ValueError(
            f"Multiple calibration references named '{reference_name}' found: {names}"
        )
    return matches[0]

def _append_calibration_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = CALIBRATION_RAW_FIELDS
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


def _append_summary_csv(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(row.keys())
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerow(row)



def _read_center_summary_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        return [
            row for row in reader
            if row.get("test_type", "").strip().lower() == "center"
        ]


def _validate_monotonic_center_calibration(path: Path) -> list[str]:
    rows = _read_center_summary_rows(path)
    points = []
    for row in rows:
        try:
            height = float(row["real_height_m"])
            width = float(row["avg_marker_width_px"])
        except (KeyError, TypeError, ValueError):
            continue
        points.append((height, width))

    points.sort(key=lambda item: item[0])
    problems = []
    for (h1, w1), (h2, w2) in zip(points, points[1:]):
        if h2 > h1 and w2 >= w1:
            problems.append(
                f"Non-monotonic calibration: {h1:.2f} m -> {w1:.1f} px, "
                f"but {h2:.2f} m -> {w2:.1f} px"
            )
    return problems


@dataclass
class LandingTelemetry:
    armed: bool | None = None
    mode: str = "UNKNOWN"
    relative_alt_m: float | None = None
    landed_state: str = "UNKNOWN"


def _update_landing_telemetry(master, telemetry: LandingTelemetry) -> LandingTelemetry:
    """Consume available MAVLink messages without blocking the camera loop."""
    if master is None:
        return telemetry
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue
        if msg_type == "HEARTBEAT":
            telemetry.armed = bool(msg.base_mode & 128)
            try:
                telemetry.mode = str(master.flightmode or "UNKNOWN")
            except Exception:
                telemetry.mode = "UNKNOWN"
        elif msg_type == "GLOBAL_POSITION_INT":
            telemetry.relative_alt_m = float(msg.relative_alt) / 1000.0
        elif msg_type == "EXTENDED_SYS_STATE":
            telemetry.landed_state = str(msg.landed_state)
    return telemetry


def _request_landing_telemetry(master) -> None:
    """Request the MAVLink messages needed for telemetry-only landing tests.

    This function only requests message rates. It does not arm the vehicle,
    change flight mode, or send movement commands.
    """
    if master is None:
        return

    requested_messages = {
        33: 5.0,   # GLOBAL_POSITION_INT
        245: 2.0,  # EXTENDED_SYS_STATE
    }
    for message_id, rate_hz in requested_messages.items():
        try:
            interval_us = int(1_000_000 / rate_hz)
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                511,  # MAV_CMD_SET_MESSAGE_INTERVAL
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0,
            )
        except Exception as exc:
            print(f"WARNING: could not request MAVLink message {message_id}: {exc}")


def _append_landing_log(path: Path, command: LandingCommand, elapsed: float, telemetry: LandingTelemetry) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    row = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "elapsed_s": round(elapsed, 3),
        "state": command.state,
        "detected": command.detected,
        "centered": command.centered,
        "confidence_percent": command.confidence_percent,
        "reference": command.reference_name,
        "stable_frames": command.stable_frames,
        "marker_side_px": command.marker_side_px,
        "telemetry_alt_m": command.telemetry_alt_m,
        "visual_alt_m": command.visual_alt_m,
        "control_alt_m": command.control_alt_m,
        "altitude_source": command.altitude_source,
        "error_x_px": command.error_x_px,
        "error_y_px": command.error_y_px,
        "error_x_m": command.error_x_m,
        "error_y_m": command.error_y_m,
        "vx_mps": command.vx_mps,
        "vy_mps": command.vy_mps,
        "vz_mps": command.vz_mps,
        "command_sent": command.command_sent,
        "block_reason": command.block_reason,
        "saved_image_path": command.saved_image_path,
        "vehicle_armed": telemetry.armed,
        "vehicle_mode": telemetry.mode,
        "landed_state": telemetry.landed_state,
    }
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=f"Run camera test for visual-marker detection ({VERSION})")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index, usually 0")
    parser.add_argument("--camera-backend", choices=["auto", "picamera2", "opencv"], default="auto")
    parser.add_argument("--marker-type", choices=["aruco", "template"], default="template")
    parser.add_argument("--aruco-id", type=int, default=23)
    parser.add_argument(
        "--aruco-dictionary",
        choices=["4x4_50", "4x4_100", "5x5_50", "5x5_100", "6x6_50", "6x6_100"],
        default="4x4_50",
    )
    parser.add_argument(
        "--reference",
        default="markers/references",
        help="Reference image or directory. Order: original, A, B, C ...",
    )
    parser.add_argument("--duration", type=int, default=60)
    parser.add_argument("--threshold", type=float, default=0.72)
    parser.add_argument("--send-email", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--save-detected-frame", default="reports/marker_detected.jpg")
    parser.add_argument(
        "--calibrate-height",
        action="store_true",
        help="Record marker measurements for camera-height calibration",
    )
    parser.add_argument(
        "--real-height",
        type=float,
        help="Measured camera-lens-to-marker height in meters",
    )
    parser.add_argument(
        "--calibration-output",
        default="reports/height_calibration_raw.csv",
        help="CSV output file for raw calibration samples",
    )
    parser.add_argument(
        "--calibration-summary-output",
        default="reports/height_calibration_summary.csv",
        help="CSV output file for one summary row per calibration run",
    )
    parser.add_argument(
        "--test-type",
        choices=["center", "right", "left", "forward", "backward", "custom"],
        default="center",
        help="Calibration series label saved in the CSV",
    )
    parser.add_argument(
        "--offset-x",
        type=float,
        default=0.0,
        help="Planned physical X offset in cm; right is positive and left is negative",
    )
    parser.add_argument(
        "--offset-y",
        type=float,
        default=0.0,
        help="Planned physical Y offset in cm; forward is positive and backward is negative",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=30,
        help="Number of accepted detection samples to record",
    )
    parser.add_argument(
        "--calibration-interval",
        type=float,
        default=0.20,
        help="Minimum seconds between recorded calibration samples",
    )
    parser.add_argument(
        "--calibration-reference-name",
        default="original",
        help="Reference filename stem accepted during height calibration. Default: original",
    )
    parser.add_argument(
        "--calibration-min-confidence",
        type=float,
        default=90.0,
        help="Minimum confidence percentage required for an accepted calibration sample. Default: 90",
    )
    parser.add_argument(
        "--calibration-max-width-cv",
        type=float,
        default=5.0,
        help="Maximum allowed width coefficient of variation in percent before warning. Default: 5",
    )
    parser.add_argument(
        "--calibration-image-dir",
        default="reports/calibration_images",
        help="Directory for annotated calibration sample images",
    )
    parser.add_argument(
        "--save-calibration-images",
        action="store_true",
        help="Save one annotated image for every accepted calibration sample",
    )
    parser.add_argument(
        "--marker-size-cm",
        type=float,
        default=20.0,
        help="Printed marker side length in centimeters, used for pixel-to-cm conversion. Default: 20",
    )
    parser.add_argument(
        "--centering-threshold-cm",
        type=float,
        default=2.0,
        help="Maximum radial offset considered centered in the live overlay. Default: 2 cm",
    )
    parser.add_argument("--landing-controller", action="store_true", help="Run visual landing diagnostics and CSV logging")
    parser.add_argument(
        "--connect-flight-controller",
        action="store_true",
        help="Connect to the flight controller for telemetry only; movement commands remain disabled unless --enable-landing-commands is also supplied",
    )
    parser.add_argument("--connection", default="/dev/ttyACM0", help="MAVLink connection used for telemetry or landing control")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--enable-landing-commands", action="store_true", help="DANGER: send real GUIDED/LAND commands; default is dry-run")
    parser.add_argument(
        "--hover-test",
        action="store_true",
        help=(
            "Limited horizontal alignment test: real GUIDED XY commands are allowed, "
            "but vertical descent and final LAND are forcibly disabled"
        ),
    )
    parser.add_argument(
        "--max-xy-speed",
        type=float,
        default=0.10,
        help="Maximum horizontal correction speed in m/s. Default: 0.10",
    )
    parser.add_argument(
        "--descent-speed",
        type=float,
        default=0.20,
        help="Vertical descent speed in m/s outside hover-test mode. Default: 0.20",
    )
    parser.add_argument("--final-land", action="store_true", help="Allow MAV_CMD_NAV_LAND after minimum altitude")
    parser.add_argument("--landing-calibration", default="reports/height_calibration_clean.csv")
    parser.add_argument("--landing-log", default="reports/landing_runs/landing_log.csv")
    parser.add_argument("--landing-log-interval", type=float, default=0.20)
    parser.add_argument("--marker-image-dir", default="reports/marker_detections")
    parser.add_argument("--marker-image-interval", type=float, default=5.0)
    parser.add_argument("--marker-image-max-count", type=int, default=100)
    parser.add_argument("--landing-altitude-source", choices=["visual", "telemetry", "auto"], default="visual")
    parser.add_argument(
        "--experiment-output-dir",
        default="reports",
        help="Parent directory for the experiment folder and ZIP package.",
    )
    parser.add_argument(
        "--no-package-experiment",
        action="store_true",
        help="Disable automatic experiment folder and ZIP creation at the end.",
    )
    args = parser.parse_args()

    if args.marker_size_cm <= 0:
        parser.error("--marker-size-cm must be greater than zero")
    if args.centering_threshold_cm <= 0:
        parser.error("--centering-threshold-cm must be greater than zero")

    if args.calibrate_height:
        if args.marker_type != "template":
            parser.error("--calibrate-height currently requires --marker-type template")
        if args.real_height is None or args.real_height <= 0:
            parser.error("--calibrate-height requires --real-height with a positive value in meters")
        if args.calibration_samples <= 0:
            parser.error("--calibration-samples must be greater than zero")
        if args.calibration_interval < 0:
            parser.error("--calibration-interval cannot be negative")
        if not 0.0 <= args.calibration_min_confidence <= 100.0:
            parser.error("--calibration-min-confidence must be between 0 and 100")
        if args.calibration_max_width_cv < 0:
            parser.error("--calibration-max-width-cv cannot be negative")
        _ensure_calibration_csv_compatibility(
            Path(args.calibration_output),
            Path(args.calibration_summary_output),
        )

        expected_offsets = {
            "center": (0.0, 0.0),
            "right": (5.0, 0.0),
            "left": (-5.0, 0.0),
            "forward": (0.0, 5.0),
            "backward": (0.0, -5.0),
        }
        # Apply useful defaults only when the operator did not explicitly enter offsets.
        if args.test_type in expected_offsets and args.offset_x == 0.0 and args.offset_y == 0.0:
            args.offset_x, args.offset_y = expected_offsets[args.test_type]

    detector_reference = args.reference
    if args.calibrate_height:
        detector_reference = str(
            _resolve_calibration_reference(
                args.reference,
                args.calibration_reference_name,
            )
        )

    if args.marker_type == "aruco":
        from aruco_marker_detector import ArucoMarkerDetector, draw_aruco_detection

        detector = ArucoMarkerDetector(marker_id=args.aruco_id, dictionary_name=args.aruco_dictionary)
        base_draw = draw_aruco_detection
        detector_description = f"ArUco {args.aruco_dictionary} ID {args.aruco_id}"
    else:
        from visual_marker_detector import VisualMarkerDetector, draw_detection

        detector = VisualMarkerDetector(detector_reference, threshold=args.threshold)
        base_draw = draw_detection
        detector_description = f"Template reference: {', '.join(detector.reference_names)}"

    master = None
    landing_controller = None
    landing_telemetry = LandingTelemetry()
    if args.connect_flight_controller and not args.landing_controller:
        parser.error("--connect-flight-controller requires --landing-controller")
    if args.enable_landing_commands and not args.landing_controller:
        parser.error("--enable-landing-commands requires --landing-controller")
    if args.hover_test and not args.enable_landing_commands:
        parser.error("--hover-test requires --enable-landing-commands")
    if args.hover_test and args.final_land:
        parser.error("--hover-test cannot be combined with --final-land")
    if args.max_xy_speed <= 0 or args.max_xy_speed > 0.25:
        parser.error("--max-xy-speed must be greater than 0 and no more than 0.25 m/s")
    if args.descent_speed <= 0 or args.descent_speed > 0.30:
        parser.error("--descent-speed must be greater than 0 and no more than 0.30 m/s")
    if args.final_land and not args.enable_landing_commands:
        parser.error("--final-land requires --enable-landing-commands")
    if args.landing_controller:
        from pymavlink import mavutil

        connect_mavlink = args.connect_flight_controller or args.enable_landing_commands
        if connect_mavlink:
            master = mavutil.mavlink_connection(args.connection, baud=args.baud)
            print(f"Waiting for MAVLink heartbeat on {args.connection}...")
            try:
                master.wait_heartbeat(timeout=15)
            except TimeoutError as exc:
                raise RuntimeError(
                    f"No MAVLink heartbeat received from {args.connection} within 15 seconds"
                ) from exc
            print(
                f"MAVLink connected: system={master.target_system} "
                f"component={master.target_component}"
            )
            _request_landing_telemetry(master)
        landing_controller = VisualLandingController(
            master=master,
            enable_commands=args.enable_landing_commands,
            require_guided=True,
            calibration_csv=args.landing_calibration,
            min_confidence=max(args.threshold, 0.90),
            required_reference="original",
            stable_frames_required=5,
            min_landing_alt_m=0.30,
            max_xy_speed_mps=args.max_xy_speed,
            descent_speed_mps=args.descent_speed,
            allow_descent=not args.hover_test,
            altitude_source=args.landing_altitude_source,
            final_land=args.final_land,
            save_marker_images=True,
            marker_image_dir=args.marker_image_dir,
            marker_image_interval_s=args.marker_image_interval,
            marker_image_max_count=args.marker_image_max_count,
            save_state_transition_images=True,
        )
        Path(args.marker_image_dir).mkdir(parents=True, exist_ok=True)
        Path(args.landing_log).parent.mkdir(parents=True, exist_ok=True)
        if args.hover_test:
            mode = (
                f"HOVER TEST (XY only, max={args.max_xy_speed:.2f} m/s; "
                "descent and LAND disabled)"
            )
        elif args.enable_landing_commands:
            mode = "REAL COMMANDS"
        elif args.connect_flight_controller:
            mode = "TELEMETRY ONLY (commands disabled)"
        else:
            mode = "DRY-RUN (camera only)"
        print(f"Landing controller enabled: {mode} | log={args.landing_log}")

    cap = CameraCapture(camera_index=args.camera_index, backend=args.camera_backend)
    cap.open()
    print(f"Camera backend: {cap.active_backend}")
    print(f"Marker detector: {detector_description}")
    print(f"Visual marker test {VERSION} started for {args.duration} seconds.")
    print("Show the marker to the camera. Press q to stop if --show is used.")
    if args.calibrate_height:
        print(
            f"HEIGHT CALIBRATION MODE | real height={args.real_height:.3f} m | "
            f"series={args.test_type} | planned offset=({args.offset_x:+.1f}, {args.offset_y:+.1f}) cm | "
            f"target samples={args.calibration_samples} | raw output={args.calibration_output} | "
            f"minimum confidence={args.calibration_min_confidence:.1f}%"
        )
        print(
            f"Calibration reference lock: '{args.calibration_reference_name}' only | "
            f"file={detector_reference}"
        )

    calibration_image_run_dir = None
    if args.calibrate_height and args.save_calibration_images:
        height_label = f"{args.real_height:.2f}m".replace(".", "p")
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        calibration_image_run_dir = (
            Path(args.calibration_image_dir) / height_label / args.test_type / run_stamp
        )
        calibration_image_run_dir.mkdir(parents=True, exist_ok=True)
        print(f"Calibration images will be saved to: {calibration_image_run_dir}")

    start = time.time()
    experiment_started_at = start
    first_detection_elapsed = None
    best_detection = None
    last_print = 0.0
    email_sent = False
    detected_once = False
    fps_frame_count = 0
    fps = 0.0
    fps_window_start = time.perf_counter()
    calibration_rows: list[dict] = []
    last_calibration_record = 0.0
    rejected_reference_count = 0
    rejected_confidence_count = 0
    last_rejection_print = 0.0
    last_landing_log_time = 0.0
    last_landing_state = ""

    try:
        while time.time() - start < args.duration:
            camera_frame = cap.read()
            if not camera_frame.ok:
                print(f"Camera frame not received. {camera_frame.message}")
                time.sleep(0.2)
                continue

            fps_frame_count += 1
            frame = camera_frame.frame
            detection = detector.detect(frame)
            now = time.time()
            elapsed = now - start

            fps_elapsed = time.perf_counter() - fps_window_start
            if fps_elapsed >= 1.0:
                fps = fps_frame_count / fps_elapsed
                fps_frame_count = 0
                fps_window_start = time.perf_counter()

            if best_detection is None or detection.score > best_detection.score:
                best_detection = detection

            landing_command = None
            if landing_controller is not None:
                landing_telemetry = _update_landing_telemetry(master, landing_telemetry)
                landing_command = landing_controller.update(
                    frame.shape, detection, landing_telemetry, frame=frame
                )
                if (
                    now - last_landing_log_time >= args.landing_log_interval
                    or landing_command.state != last_landing_state
                ):
                    _append_landing_log(
                        Path(args.landing_log), landing_command, elapsed, landing_telemetry
                    )
                    last_landing_log_time = now
                if landing_command.state != last_landing_state:
                    print(f"LANDING STATE: {last_landing_state or 'NONE'} -> {landing_command.state}")
                    if landing_command.saved_image_path:
                        print(f"Landing evidence saved: {landing_command.saved_image_path}")
                    last_landing_state = landing_command.state

            if now - last_print >= 1.0:
                last_print = now
                if detection.detected:
                    print(
                        f"Marker detected | reference={getattr(detection, 'reference_name', None) or 'N/A'} | "
                        f"confidence={detection.score * 100:.1f}% | elapsed={elapsed:.2f}s | "
                        f"processing={getattr(detection, 'processing_time_s', 0.0) * 1000:.1f}ms | FPS={fps:.1f}"
                        + (f" | landing={landing_command.state}" if landing_command else "")
                    )
                else:
                    print(f"Marker not detected | {detection.message} | elapsed={elapsed:.2f}s | FPS={fps:.1f}")

            if args.calibrate_height and detection.detected and detection.corners is not None:
                detected_reference = (getattr(detection, "reference_name", None) or "").lower()
                required_reference = args.calibration_reference_name.lower()
                confidence_percent = detection.score * 100.0

                if detected_reference != required_reference:
                    rejected_reference_count += 1
                    if now - last_rejection_print >= 1.0:
                        last_rejection_print = now
                        print(
                            f"CALIBRATION SAMPLE REJECTED | reason=reference | "
                            f"detected={detected_reference or 'none'} | required={required_reference}"
                        )
                    continue

                if confidence_percent < args.calibration_min_confidence:
                    rejected_confidence_count += 1
                    if now - last_rejection_print >= 1.0:
                        last_rejection_print = now
                        print(
                            f"CALIBRATION SAMPLE REJECTED | reason=confidence | "
                            f"confidence={confidence_percent:.1f}% | "
                            f"required>={args.calibration_min_confidence:.1f}%"
                        )
                    continue

                if now - last_calibration_record >= args.calibration_interval:
                    geometry = _marker_geometry(detection.corners, frame.shape)
                    # Convert image-plane pixel error to centimeters using the observed marker width.
                    # This is a local scale approximation: printed_marker_width_cm / detected_marker_width_px.
                    cm_per_px = (args.marker_size_cm / geometry["marker_width_px"]) if geometry["marker_width_px"] > 0 else None
                    offset_x_cm = geometry["offset_x_px"] * cm_per_px if cm_per_px is not None else None
                    offset_y_cm = geometry["offset_y_px"] * cm_per_px if cm_per_px is not None else None
                    error_distance_cm = geometry["error_distance_px"] * cm_per_px if cm_per_px is not None else None

                    sample_number = len(calibration_rows) + 1
                    image_path = ""
                    if calibration_image_run_dir is not None:
                        image_file = calibration_image_run_dir / f"sample_{sample_number:03d}.jpg"
                        annotated = base_draw(
                            frame,
                            detection,
                            elapsed_s=elapsed,
                            fps=fps,
                            system_state="CALIBRATING",
                        )
                        annotated = _draw_centering_overlay(
                            annotated,
                            detection,
                            marker_size_cm=args.marker_size_cm,
                            visual_alt_m=args.real_height,
                            centered_threshold_cm=args.centering_threshold_cm,
                        )
                        cv2.imwrite(str(image_file), annotated)
                        image_path = str(image_file)

                    row = {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "real_height_m": args.real_height,
                        "test_type": args.test_type,
                        "planned_offset_x_cm": args.offset_x,
                        "planned_offset_y_cm": args.offset_y,
                        "reference": getattr(detection, "reference_name", None) or "N/A",
                        "confidence_percent": confidence_percent,
                        **geometry,
                        "offset_x_cm": offset_x_cm,
                        "offset_y_cm": offset_y_cm,
                        "error_distance_cm": error_distance_cm,
                        "image_path": image_path,
                        "processing_time_ms": getattr(detection, "processing_time_s", 0.0) * 1000.0,
                        "fps": fps,
                        "elapsed_s": elapsed,
                    }
                    calibration_rows.append(row)
                    last_calibration_record = now

                    accepted_widths = [item["marker_width_px"] for item in calibration_rows]
                    running_mean_width = statistics.mean(accepted_widths)
                    running_stdev_width = statistics.stdev(accepted_widths) if len(accepted_widths) > 1 else 0.0
                    running_width_cv = (running_stdev_width / running_mean_width * 100.0) if running_mean_width > 0 else 0.0

                    print(
                        f"CALIBRATION {len(calibration_rows)}/{args.calibration_samples} | "
                        f"height={args.real_height:.2f}m | series={args.test_type} | "
                        f"planned offset=({args.offset_x:+.1f},{args.offset_y:+.1f})cm | "
                        f"width={geometry['marker_width_px']:.1f}px | "
                        f"height_px={geometry['marker_height_px']:.1f}px | "
                        f"area={geometry['marker_area_px2']:.0f}px^2 | confidence={confidence_percent:.1f}% | "
                        f"running_avg_width={running_mean_width:.1f}px | width_CV={running_width_cv:.2f}%"
                    )
                    if len(calibration_rows) >= 5 and running_width_cv > args.calibration_max_width_cv:
                        print(
                            f"WARNING: calibration width is unstable | CV={running_width_cv:.2f}% | "
                            f"limit={args.calibration_max_width_cv:.2f}% | hold camera and marker steady"
                        )
                    if len(calibration_rows) >= args.calibration_samples:
                        print("Calibration sample target reached.")
                        break

            if detection.detected and not detected_once:
                detected_once = True
                first_detection_elapsed = elapsed
                print(
                    f"FIRST DETECTION | reference={getattr(detection, 'reference_name', None) or 'N/A'} | "
                    f"confidence={detection.score * 100:.1f}% | detection time={first_detection_elapsed:.2f}s"
                )
                save_path = Path(args.save_detected_frame)
                save_path.parent.mkdir(parents=True, exist_ok=True)
                if args.marker_type == "template":
                    annotated = base_draw(frame, detection, elapsed_s=elapsed, fps=fps, system_state="DETECTED")
                else:
                    annotated = base_draw(frame, detection)
                detected_visual_alt = (
                    landing_command.visual_alt_m
                    if landing_command is not None
                    else (args.real_height if args.calibrate_height else None)
                )
                annotated = _draw_centering_overlay(
                    annotated,
                    detection,
                    marker_size_cm=args.marker_size_cm,
                    visual_alt_m=detected_visual_alt,
                    centered_threshold_cm=args.centering_threshold_cm,
                )
                cv2.imwrite(str(save_path), annotated)
                print(f"Detected frame saved: {save_path}")

                if args.send_email and not email_sent:
                    try:
                        from email_sender import send_email_report

                        body = (
                            f"The drone camera detected the visual marker.\n"
                            f"{detection.message}\n"
                            f"Detection time: {first_detection_elapsed:.2f} seconds\n"
                            f"Confidence: {detection.score * 100:.1f}%\n"
                            f"Reference: {getattr(detection, 'reference_name', None) or 'N/A'}"
                        )
                        send_email_report(
                            "Drone Visual Marker Detected",
                            body,
                            [str(save_path)],
                        )
                        email_sent = True
                        print("Email notification sent successfully.")
                    except ImportError as exc:
                        print(f"Email support unavailable: {exc}")

            if args.show:
                if args.marker_type == "template":
                    state = "CALIBRATING" if args.calibrate_height else ("DETECTED" if detection.detected else "SEARCHING")
                    shown = base_draw(frame, detection, elapsed_s=elapsed, fps=fps, system_state=state)
                else:
                    shown = base_draw(frame, detection)

                visual_altitude = None
                if landing_command is not None:
                    visual_altitude = landing_command.visual_alt_m
                elif args.calibrate_height:
                    visual_altitude = args.real_height

                shown = _draw_centering_overlay(
                    shown,
                    detection,
                    marker_size_cm=args.marker_size_cm,
                    visual_alt_m=visual_altitude,
                    centered_threshold_cm=args.centering_threshold_cm,
                )
                cv2.imshow("Visual Marker Test", shown)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.02)
    finally:
        if landing_controller is not None:
            landing_controller.stop()
        cap.close()
        if args.show:
            cv2.destroyAllWindows()

    if args.calibrate_height:
        output_path = Path(args.calibration_output)
        if calibration_rows:
            _append_calibration_csv(output_path, calibration_rows)
            widths = [row["marker_width_px"] for row in calibration_rows]
            heights = [row["marker_height_px"] for row in calibration_rows]
            areas = [row["marker_area_px2"] for row in calibration_rows]
            confidences = [row["confidence_percent"] for row in calibration_rows]
            fps_values = [row["fps"] for row in calibration_rows]
            processing_values = [row["processing_time_ms"] for row in calibration_rows]
            elapsed_values = [row["elapsed_s"] for row in calibration_rows]
            offset_x_values = [row["offset_x_px"] for row in calibration_rows]
            offset_y_values = [row["offset_y_px"] for row in calibration_rows]
            distance_values = [row["error_distance_px"] for row in calibration_rows]
            offset_x_cm_values = [row["offset_x_cm"] for row in calibration_rows if row["offset_x_cm"] is not None]
            offset_y_cm_values = [row["offset_y_cm"] for row in calibration_rows if row["offset_y_cm"] is not None]
            distance_cm_values = [row["error_distance_cm"] for row in calibration_rows if row["error_distance_cm"] is not None]

            def sample_stdev(values):
                return statistics.stdev(values) if len(values) > 1 else 0.0

            summary_row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "real_height_m": args.real_height,
                "test_type": args.test_type,
                "planned_offset_x_cm": args.offset_x,
                "planned_offset_y_cm": args.offset_y,
                "samples": len(calibration_rows),
                "avg_marker_width_px": statistics.mean(widths),
                "stdev_marker_width_px": sample_stdev(widths),
                "avg_marker_height_px": statistics.mean(heights),
                "stdev_marker_height_px": sample_stdev(heights),
                "avg_marker_area_px2": statistics.mean(areas),
                "stdev_marker_area_px2": sample_stdev(areas),
                "avg_confidence_percent": statistics.mean(confidences),
                "stdev_confidence_percent": sample_stdev(confidences),
                "avg_fps": statistics.mean(fps_values),
                "avg_processing_time_ms": statistics.mean(processing_values),
                "avg_offset_x_px": statistics.mean(offset_x_values),
                "stdev_offset_x_px": sample_stdev(offset_x_values),
                "avg_offset_y_px": statistics.mean(offset_y_values),
                "stdev_offset_y_px": sample_stdev(offset_y_values),
                "avg_error_distance_px": statistics.mean(distance_values),
                "stdev_error_distance_px": sample_stdev(distance_values),
                "min_error_distance_px": min(distance_values),
                "max_error_distance_px": max(distance_values),
                "avg_offset_x_cm": statistics.mean(offset_x_cm_values) if offset_x_cm_values else "",
                "stdev_offset_x_cm": sample_stdev(offset_x_cm_values) if offset_x_cm_values else "",
                "avg_offset_y_cm": statistics.mean(offset_y_cm_values) if offset_y_cm_values else "",
                "stdev_offset_y_cm": sample_stdev(offset_y_cm_values) if offset_y_cm_values else "",
                "avg_error_distance_cm": statistics.mean(distance_cm_values) if distance_cm_values else "",
                "stdev_error_distance_cm": sample_stdev(distance_cm_values) if distance_cm_values else "",
                "min_error_distance_cm": min(distance_cm_values) if distance_cm_values else "",
                "max_error_distance_cm": max(distance_cm_values) if distance_cm_values else "",
                "saved_images": sum(1 for row in calibration_rows if row.get("image_path")),
                "first_accepted_sample_elapsed_s": min(elapsed_values),
                "last_accepted_sample_elapsed_s": max(elapsed_values),
            }
            summary_path = Path(args.calibration_summary_output)
            _append_summary_csv(summary_path, summary_row)

            mean_width = statistics.mean(widths)
            stdev_width = sample_stdev(widths)
            width_cv = (stdev_width / mean_width * 100.0) if mean_width > 0 else 0.0
            quality_status = "PASS" if width_cv <= args.calibration_max_width_cv else "WARNING"

            print(
                f"CALIBRATION SUMMARY | samples={len(calibration_rows)} | real height={args.real_height:.3f}m | "
                f"series={args.test_type} | offset=({args.offset_x:+.1f},{args.offset_y:+.1f})cm | "
                f"avg width={statistics.mean(widths):.1f}px (SD={sample_stdev(widths):.1f}) | "
                f"avg height={statistics.mean(heights):.1f}px | avg area={statistics.mean(areas):.0f}px^2 | "
                f"avg confidence={statistics.mean(confidences):.1f}% | avg FPS={statistics.mean(fps_values):.1f}"
            )
            print(
                f"CENTERING SUMMARY | avg X={statistics.mean(offset_x_values):+.1f}px | "
                f"avg Y={statistics.mean(offset_y_values):+.1f}px | "
                f"avg radial error={statistics.mean(distance_values):.1f}px | "
                f"max radial error={max(distance_values):.1f}px"
            )
            if distance_cm_values:
                print(
                    f"CENTERING SUMMARY CM | avg radial error={statistics.mean(distance_cm_values):.2f}cm | "
                    f"max radial error={max(distance_cm_values):.2f}cm | marker size={args.marker_size_cm:.1f}cm"
                )

            print(
                f"CALIBRATION QUALITY {quality_status} | accepted={len(calibration_rows)} | "
                f"rejected_reference={rejected_reference_count} | "
                f"rejected_confidence={rejected_confidence_count} | "
                f"width_CV={width_cv:.2f}% | limit={args.calibration_max_width_cv:.2f}%"
            )
            print(f"Raw calibration data appended to: {output_path}")
            print(f"Run summary appended to: {summary_path}")

            monotonic_problems = _validate_monotonic_center_calibration(summary_path)
            if monotonic_problems:
                print("CALIBRATION CONSISTENCY ERROR:")
                for problem in monotonic_problems:
                    print(f" - {problem}")
                print("Repeat the listed height measurement(s) before using this calibration for landing.")
            else:
                print("Calibration consistency check: PASS (marker width decreases with height).")
        else:
            print("CALIBRATION SUMMARY | no valid detections recorded; CSV was not changed.")

    total = time.time() - start

    if not args.no_package_experiment:
        try:
            package_result = package_experiment(
                reports_dir=args.experiment_output_dir,
                started_at_epoch=experiment_started_at,
                software_version=VERSION,
                test_type=args.test_type if args.calibrate_height else "marker_detection",
                real_height_m=args.real_height if args.calibrate_height else None,
                marker_size_cm=args.marker_size_cm,
                raw_csv=args.calibration_output,
                summary_csv=args.calibration_summary_output,
                landing_csv=args.landing_log,
                calibration_image_run_dir=calibration_image_run_dir,
                marker_image_dir=args.marker_image_dir,
                detected_frame_path=args.save_detected_frame,
            )
            print(f"EXPERIMENT PACKAGE: {package_result.experiment_dir}")
            print(f"EXPERIMENT ZIP: {package_result.zip_path}")
            print(
                f"Package contents: images={package_result.copied_images}, "
                f"plots={package_result.plots_created}"
            )
        except Exception as exc:
            print(f"WARNING: experiment package could not be created: {exc}")

    if landing_controller is not None:
        print(f"Landing CSV log: {args.landing_log}")
        print(f"Landing evidence directory: {args.marker_image_dir}")
    if detected_once:
        print(f"FINAL RESULT: DETECTED | detection time={first_detection_elapsed:.2f}s")
    elif best_detection is not None:
        print(
            f"FINAL RESULT: NOT DETECTED | best reference={getattr(best_detection, 'reference_name', None) or 'none'} | "
            f"best confidence={best_detection.score * 100:.1f}% | test duration={total:.2f}s"
        )
    else:
        print(f"FINAL RESULT: NOT DETECTED | test duration={total:.2f}s")


if __name__ == "__main__":
    main()
