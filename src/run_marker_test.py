import argparse
import csv
import statistics
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera_capture import CameraCapture
from email_sender import send_email_report


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
    return {
        "marker_width_px": marker_width_px,
        "marker_height_px": marker_height_px,
        "marker_area_px2": marker_area_px2,
        "offset_x_px": center_x - frame_w // 2,
        "offset_y_px": center_y - frame_h // 2,
    }


def _append_calibration_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp",
        "real_height_m",
        "reference",
        "confidence_percent",
        "marker_width_px",
        "marker_height_px",
        "marker_area_px2",
        "offset_x_px",
        "offset_y_px",
        "processing_time_ms",
        "fps",
        "elapsed_s",
    ]
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="Run camera test for visual-marker detection")
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
        default="reports/height_calibration.csv",
        help="CSV output file for calibration samples",
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
    args = parser.parse_args()

    if args.calibrate_height:
        if args.marker_type != "template":
            parser.error("--calibrate-height currently requires --marker-type template")
        if args.real_height is None or args.real_height <= 0:
            parser.error("--calibrate-height requires --real-height with a positive value in meters")
        if args.calibration_samples <= 0:
            parser.error("--calibration-samples must be greater than zero")
        if args.calibration_interval < 0:
            parser.error("--calibration-interval cannot be negative")

    if args.marker_type == "aruco":
        from aruco_marker_detector import ArucoMarkerDetector, draw_aruco_detection

        detector = ArucoMarkerDetector(marker_id=args.aruco_id, dictionary_name=args.aruco_dictionary)
        base_draw = draw_aruco_detection
        detector_description = f"ArUco {args.aruco_dictionary} ID {args.aruco_id}"
    else:
        from visual_marker_detector import VisualMarkerDetector, draw_detection

        detector = VisualMarkerDetector(args.reference, threshold=args.threshold)
        base_draw = draw_detection
        detector_description = f"Multi-reference template: {', '.join(detector.reference_names)}"

    cap = CameraCapture(camera_index=args.camera_index, backend=args.camera_backend)
    cap.open()
    print(f"Camera backend: {cap.active_backend}")
    print(f"Marker detector: {detector_description}")
    print(f"Visual marker test started for {args.duration} seconds.")
    print("Show the marker to the camera. Press q to stop if --show is used.")
    if args.calibrate_height:
        print(
            f"HEIGHT CALIBRATION MODE | real height={args.real_height:.3f} m | "
            f"target samples={args.calibration_samples} | output={args.calibration_output}"
        )

    start = time.time()
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

            if now - last_print >= 1.0:
                last_print = now
                if detection.detected:
                    print(
                        f"Marker detected | reference={getattr(detection, 'reference_name', None) or 'N/A'} | "
                        f"confidence={detection.score * 100:.1f}% | elapsed={elapsed:.2f}s | "
                        f"processing={getattr(detection, 'processing_time_s', 0.0) * 1000:.1f}ms | FPS={fps:.1f}"
                    )
                else:
                    print(f"Marker not detected | {detection.message} | elapsed={elapsed:.2f}s | FPS={fps:.1f}")

            if args.calibrate_height and detection.detected and detection.corners is not None:
                if now - last_calibration_record >= args.calibration_interval:
                    geometry = _marker_geometry(detection.corners, frame.shape)
                    row = {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "real_height_m": args.real_height,
                        "reference": getattr(detection, "reference_name", None) or "N/A",
                        "confidence_percent": detection.score * 100.0,
                        **geometry,
                        "processing_time_ms": getattr(detection, "processing_time_s", 0.0) * 1000.0,
                        "fps": fps,
                        "elapsed_s": elapsed,
                    }
                    calibration_rows.append(row)
                    last_calibration_record = now
                    print(
                        f"CALIBRATION {len(calibration_rows)}/{args.calibration_samples} | "
                        f"height={args.real_height:.2f}m | width={geometry['marker_width_px']:.1f}px | "
                        f"height_px={geometry['marker_height_px']:.1f}px | "
                        f"area={geometry['marker_area_px2']:.0f}px^2 | confidence={detection.score * 100:.1f}%"
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
                cv2.imwrite(str(save_path), annotated)
                print(f"Detected frame saved: {save_path}")

                if args.send_email and not email_sent:
                    body = (
                        f"The drone camera detected the visual marker.\n{detection.message}\n"
                        f"Detection time: {first_detection_elapsed:.2f} seconds\n"
                        f"Confidence: {detection.score * 100:.1f}%\n"
                        f"Reference: {getattr(detection, 'reference_name', None) or 'N/A'}"
                    )
                    send_email_report("Drone Visual Marker Detected", body, [str(save_path)])
                    email_sent = True
                    print("Email notification sent successfully.")

            if args.show:
                if args.marker_type == "template":
                    state = "CALIBRATING" if args.calibrate_height else ("DETECTED" if detection.detected else "SEARCHING")
                    shown = base_draw(frame, detection, elapsed_s=elapsed, fps=fps, system_state=state)
                else:
                    shown = base_draw(frame, detection)
                cv2.imshow("Visual Marker Test", shown)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.02)
    finally:
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
            print(
                f"CALIBRATION SUMMARY | samples={len(calibration_rows)} | real height={args.real_height:.3f}m | "
                f"avg width={statistics.mean(widths):.1f}px | avg height={statistics.mean(heights):.1f}px | "
                f"avg area={statistics.mean(areas):.0f}px^2 | avg confidence={statistics.mean(confidences):.1f}%"
            )
            print(f"Calibration data appended to: {output_path}")
        else:
            print("CALIBRATION SUMMARY | no valid detections recorded; CSV was not changed.")

    total = time.time() - start
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
