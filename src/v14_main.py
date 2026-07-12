from __future__ import annotations

import argparse
import csv
import logging
import time
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from height_source import HeightProvider
from summary_calibration import SummaryCalibrationTable


class CameraBackend(Protocol):
    def read(self) -> tuple[bool, np.ndarray | None]:
        ...

    def close(self) -> None:
        ...


class Picamera2Backend:
    def __init__(self, width: int, height: int) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 is not installed. Install it with: "
                "sudo apt install -y python3-picamera2"
            ) from exc

        self.camera = Picamera2()
        configuration = self.camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": 30},
            buffer_count=4,
        )
        self.camera.configure(configuration)
        self.camera.start()
        time.sleep(1.0)

    def read(self) -> tuple[bool, np.ndarray | None]:
        try:
            frame_rgb = self.camera.capture_array()
            if frame_rgb is None:
                return False, None

            # Picamera2 RGB888 -> OpenCV BGR
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            return True, frame_bgr
        except Exception as exc:
            logging.warning("Picamera2 frame capture failed: %s", exc)
            return False, None

    def close(self) -> None:
        try:
            self.camera.stop()
        finally:
            self.camera.close()


class OpenCVBackend:
    def __init__(self, index: int, width: int, height: int) -> None:
        self.camera = cv2.VideoCapture(index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.camera.isOpened():
            raise RuntimeError(f"Could not open OpenCV camera index {index}.")

    def read(self) -> tuple[bool, np.ndarray | None]:
        success, frame = self.camera.read()
        return bool(success), frame if success else None

    def close(self) -> None:
        self.camera.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="V14 ArUco detection with direct summary CSV calibration"
    )
    parser.add_argument(
        "--calibration-summary",
        default="reports/height_calibration_summary.csv",
    )
    parser.add_argument("--marker-size", type=float, required=True)
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--connection")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--camera-backend",
        choices=["picamera2", "opencv", "auto"],
        default="picamera2",
        help="Use picamera2 for Raspberry Pi CSI cameras.",
    )
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--manual-height", type=float, default=0.25)
    parser.add_argument("--stable-frames", type=int, default=8)
    parser.add_argument("--deadband", type=float, default=0.03)
    parser.add_argument("--kp", type=float, default=0.45)
    parser.add_argument("--max-velocity", type=float, default=0.20)
    parser.add_argument("--show", action="store_true")
    parser.add_argument(
        "--log-file",
        default="logs/v14_detection_log.csv",
    )
    return parser.parse_args()


def connect_vehicle(connection: str | None, baud: int):
    if not connection:
        return None

    from pymavlink import mavutil

    vehicle = mavutil.mavlink_connection(connection, baud=baud)
    vehicle.wait_heartbeat(timeout=10)
    logging.info(
        "MAVLink heartbeat received: system=%s component=%s",
        vehicle.target_system,
        vehicle.target_component,
    )
    return vehicle


def create_camera(args: argparse.Namespace) -> CameraBackend:
    if args.camera_backend == "picamera2":
        logging.info("Opening Raspberry Pi CSI camera with Picamera2.")
        return Picamera2Backend(args.width, args.height)

    if args.camera_backend == "opencv":
        logging.info("Opening camera index %d with OpenCV.", args.camera_index)
        return OpenCVBackend(args.camera_index, args.width, args.height)

    # auto: prefer Picamera2, then fall back to OpenCV
    try:
        logging.info("Trying Picamera2 camera backend.")
        return Picamera2Backend(args.width, args.height)
    except Exception as exc:
        logging.warning("Picamera2 unavailable: %s", exc)
        logging.info("Falling back to OpenCV camera index %d.", args.camera_index)
        return OpenCVBackend(args.camera_index, args.width, args.height)


def marker_center(corners: np.ndarray) -> tuple[float, float]:
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)
    return float(center[0]), float(center[1])


def proposed_velocity(
    offset_x_m: float,
    offset_y_m: float,
    kp: float,
    max_velocity: float,
    deadband: float,
) -> tuple[float, float, bool]:
    centered_x = abs(offset_x_m) <= deadband
    centered_y = abs(offset_y_m) <= deadband

    if centered_x and centered_y:
        return 0.0, 0.0, True

    right = 0.0 if centered_x else _clamp(kp * offset_x_m, max_velocity)
    forward = 0.0 if centered_y else _clamp(-kp * offset_y_m, max_velocity)
    return right, forward, False


def _clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


def ensure_log(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return

    with path.open("w", newline="", encoding="utf-8") as handle:
        csv.writer(handle).writerow(
            [
                "timestamp",
                "marker_id",
                "height_m",
                "height_source",
                "calibration_height_m",
                "calibration_clamped",
                "offset_x_m",
                "offset_y_m",
                "proposed_right_m_s",
                "proposed_forward_m_s",
                "centered",
            ]
        )


def append_log(path: Path, values: list[object]) -> None:
    with path.open("a", newline="", encoding="utf-8") as handle:
        csv.writer(handle).writerow(values)


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    calibration = SummaryCalibrationTable.from_summary_csv(
        args.calibration_summary,
        marker_size_m=args.marker_size,
    )

    logging.info(
        "Loaded %d center calibration heights from %.2f to %.2f m",
        len(calibration.points),
        calibration.min_height_m,
        calibration.max_height_m,
    )
    for point in calibration.points:
        logging.info(
            "Calibration %.2f m | width %.2f px | height %.2f px | "
            "X %.2f px/m | Y %.2f px/m",
            point.height_m,
            point.marker_width_px,
            point.marker_height_px,
            point.px_per_meter_x,
            point.px_per_meter_y,
        )

    vehicle = connect_vehicle(args.connection, args.baud)
    height_provider = HeightProvider(
        vehicle=vehicle,
        fallback_height_m=args.manual_height,
    )

    aruco_dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    aruco_parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(
        aruco_dictionary,
        aruco_parameters,
    )

    camera = create_camera(args)

    log_path = Path(args.log_file)
    ensure_log(log_path)

    stable_frames = 0
    consecutive_read_failures = 0

    logging.info(
        "V14 started. Movement transmission remains disabled. "
        "Press q in the camera window or Ctrl+C to stop."
    )

    try:
        while True:
            success, frame = camera.read()
            if not success or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures == 1 or consecutive_read_failures % 20 == 0:
                    logging.warning(
                        "Camera frame read failed (%d consecutive failures).",
                        consecutive_read_failures,
                    )
                time.sleep(0.05)
                continue

            consecutive_read_failures = 0

            corners, ids, _ = detector.detectMarkers(frame)
            selected_index = None

            if ids is not None:
                matches = np.where(ids.flatten() == args.marker_id)[0]
                if len(matches):
                    selected_index = int(matches[0])

            if selected_index is None:
                stable_frames = 0
                cv2.putText(
                    frame,
                    "Marker not detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )
            else:
                stable_frames += 1
                selected_corners = corners[selected_index]
                center_x, center_y = marker_center(selected_corners)
                reading = height_provider.read()

                offset_x_m, offset_y_m, applied = (
                    calibration.pixel_offset_to_meters(
                        marker_center_x_px=center_x,
                        marker_center_y_px=center_y,
                        frame_width_px=frame.shape[1],
                        frame_height_px=frame.shape[0],
                        height_m=reading.height_m,
                    )
                )

                right_m_s, forward_m_s, centered = proposed_velocity(
                    offset_x_m=offset_x_m,
                    offset_y_m=offset_y_m,
                    kp=args.kp,
                    max_velocity=args.max_velocity,
                    deadband=args.deadband,
                )

                cv2.aruco.drawDetectedMarkers(
                    frame,
                    [selected_corners],
                    np.array([[args.marker_id]]),
                )
                cv2.circle(
                    frame,
                    (round(center_x), round(center_y)),
                    5,
                    (0, 255, 0),
                    -1,
                )
                cv2.line(
                    frame,
                    (frame.shape[1] // 2, 0),
                    (frame.shape[1] // 2, frame.shape[0]),
                    (255, 255, 0),
                    1,
                )
                cv2.line(
                    frame,
                    (0, frame.shape[0] // 2),
                    (frame.shape[1], frame.shape[0] // 2),
                    (255, 255, 0),
                    1,
                )

                text = (
                    f"h={reading.height_m:.2f}m "
                    f"X={offset_x_m:+.3f}m "
                    f"Y={offset_y_m:+.3f}m "
                    f"stable={stable_frames}/{args.stable_frames}"
                )
                cv2.putText(
                    frame,
                    text,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 0),
                    2,
                )

                if stable_frames >= args.stable_frames:
                    logging.info(
                        "Marker %d | height %.2f m (%s) | "
                        "X %+0.3f m | Y %+0.3f m | "
                        "proposed right %+0.3f m/s | "
                        "forward %+0.3f m/s | centered=%s%s",
                        args.marker_id,
                        reading.height_m,
                        reading.source,
                        offset_x_m,
                        offset_y_m,
                        right_m_s,
                        forward_m_s,
                        centered,
                        " | HEIGHT CLAMPED" if applied.clamped else "",
                    )

                    append_log(
                        log_path,
                        [
                            time.strftime("%Y-%m-%dT%H:%M:%S"),
                            args.marker_id,
                            f"{reading.height_m:.4f}",
                            reading.source,
                            f"{applied.applied_height_m:.4f}",
                            applied.clamped,
                            f"{offset_x_m:.5f}",
                            f"{offset_y_m:.5f}",
                            f"{right_m_s:.4f}",
                            f"{forward_m_s:.4f}",
                            centered,
                        ],
                    )

                    stable_frames = max(0, args.stable_frames - 2)

            if args.show:
                cv2.imshow("Drone Health Monitor V14", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        camera.close()
        cv2.destroyAllWindows()
        if vehicle is not None:
            vehicle.close()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        logging.exception("V14 stopped because of an error: %s", error)
        raise SystemExit(1)
