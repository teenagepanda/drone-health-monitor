import argparse
import time
from pathlib import Path

import cv2

from mavlink_reader import MavlinkReader
from vibration import VibrationAnalyzer
from health_checks import check_drone_health
from logger import CsvLogger
from pi_health import get_cpu_temp_c, get_cpu_usage_percent, get_ram_usage_percent
from report_generator import generate_report, text_summary
from email_sender import send_email_report
import config



def run_camera_only(args) -> None:
    """Run marker detection without creating any MAVLink connection."""
    from camera_capture import CameraCapture

    if args.marker_type == "aruco":
        from aruco_marker_detector import ArucoMarkerDetector, draw_aruco_detection

        detector = ArucoMarkerDetector(
            marker_id=args.aruco_id,
            dictionary_name=args.aruco_dictionary,
        )
        draw_detection = draw_aruco_detection
        description = f"ArUco {args.aruco_dictionary} ID {args.aruco_id}"
    else:
        from visual_marker_detector import VisualMarkerDetector, draw_detection

        detector = VisualMarkerDetector(
            args.marker_reference,
            threshold=args.marker_threshold,
        )
        description = f"references {', '.join(detector.reference_names)}"

    camera = CameraCapture(
        camera_index=args.camera_index,
        backend=args.camera_backend,
    )
    camera.open()

    print(f"Camera-only test started for {args.duration} seconds.")
    print("Flight controller connection: DISABLED")
    print(f"Camera backend: {camera.active_backend}")
    print(f"Visual marker detection: {description}")
    if args.autoland_on_marker or args.enable_autolanding:
        print("Autonomous landing is disabled in camera-only mode.")

    start = time.time()
    last_print = 0.0
    detected_once = False
    best_detection = None
    detection_elapsed = None
    saved_frame_path = Path(args.report_dir) / "marker_detected_camera_only.jpg"

    try:
        while time.time() - start < args.duration:
            camera_frame = camera.read()
            if not camera_frame.ok:
                print(f"Camera frame not received. {camera_frame.message}")
                time.sleep(0.20)
                continue

            frame = camera_frame.frame
            detection = detector.detect(frame)
            elapsed = time.time() - start

            if best_detection is None or detection.score > best_detection.score:
                best_detection = detection

            if detection.detected and not detected_once:
                detected_once = True
                detection_elapsed = elapsed
                saved_frame_path.parent.mkdir(parents=True, exist_ok=True)
                annotated = draw_detection(frame, detection)
                cv2.imwrite(str(saved_frame_path), annotated)
                print(
                    f"Visual marker detected | {detection.message} | "
                    f"detection time={detection_elapsed:.2f}s"
                )
                print(f"Detected frame saved: {saved_frame_path}")

                if args.email_on_marker:
                    send_email_report(
                        subject="Drone Camera-Only Marker Test",
                        body=(
                            "The visual marker was detected in camera-only mode.\n"
                            f"{detection.message}\n"
                            f"Detection time: {detection_elapsed:.2f} seconds"
                        ),
                        attachments=[str(saved_frame_path)],
                    )
                    print("Marker detection email sent successfully.")

            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                print(f"Camera-only marker status: {detection.message}")

            if args.show:
                shown = draw_detection(frame, detection)
                cv2.imshow("Camera-only visual marker test", shown)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nCamera-only test stopped by user.")
    finally:
        camera.close()
        if args.show:
            cv2.destroyAllWindows()

    if detected_once:
        print(f"FINAL RESULT: DETECTED | detection time={detection_elapsed:.2f}s")
    elif best_detection is not None:
        print(
            "FINAL RESULT: NOT DETECTED | "
            f"best reference={getattr(best_detection, 'reference_name', None) or 'none'} | "
            f"best confidence={best_detection.score * 100:.1f}%"
        )
    else:
        print("FINAL RESULT: NOT DETECTED | no valid camera frames")

def main():
    parser = argparse.ArgumentParser(description="Run timed drone health test and optionally send email report")
    parser.add_argument("--connection", default=config.DEFAULT_CONNECTION)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--duration", type=int, default=180, help="Test duration in seconds")
    parser.add_argument("--no-flight-controller", action="store_true", help="Run camera marker test only, without opening a MAVLink connection")
    parser.add_argument("--show", action="store_true", help="Show the camera window during camera-only testing")
    parser.add_argument("--send-email", action="store_true", help="Send report to email after test")
    parser.add_argument("--detect-marker", action="store_true", help="Also test camera visual-marker detection during the health test")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index for visual-marker test")
    parser.add_argument("--camera-backend", choices=["auto", "picamera2", "opencv"], default="auto", help="Camera backend. Use picamera2 for Raspberry Pi CSI camera")
    parser.add_argument("--marker-type", choices=["aruco", "template"], default="template", help="Marker detection method")
    parser.add_argument("--aruco-id", type=int, default=23, help="Target ArUco marker ID")
    parser.add_argument("--aruco-dictionary", choices=["4x4_50", "4x4_100", "5x5_50", "5x5_100", "6x6_50", "6x6_100"], default="4x4_50", help="ArUco dictionary")
    parser.add_argument("--marker-reference", default="markers/references", help="Reference image or directory. Directory order: original, A, B, C ...")
    parser.add_argument("--marker-threshold", type=float, default=0.72, help="Template detection score threshold")
    parser.add_argument("--email-on-marker", action="store_true", help="Send email immediately when the visual marker is detected")
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--report-dir", default="reports")

    # Autonomous landing is OFF by default. Without --enable-autolanding it is dry-run only.
    parser.add_argument("--autoland-on-marker", action="store_true", help="Run the visual landing controller when the marker is detected")
    parser.add_argument("--enable-autolanding", action="store_true", help="Actually send MAVLink velocity commands. Use only after safe bench/SITL tests")
    parser.add_argument("--landing-require-guided", action="store_true", default=True, help="Send real landing commands only in GUIDED mode")
    parser.add_argument("--landing-kp-xy", type=float, default=0.0008, help="Fallback pixel-error velocity gain when calibrated conversion is unavailable")
    parser.add_argument("--landing-kp-position", type=float, default=0.80, help="Horizontal position-error gain in 1/s when using calibrated meters")
    parser.add_argument("--landing-calibration-csv", default="reports/height_calibration_summary.csv", help="Height calibration summary CSV")
    parser.add_argument("--landing-marker-size-m", type=float, default=0.20, help="Real printed marker side length in meters")
    parser.add_argument("--landing-disable-calibration", action="store_true", help="Disable height-aware pixel-to-meter conversion")
    parser.add_argument("--landing-max-xy-speed", type=float, default=0.25, help="Maximum horizontal correction speed in m/s")
    parser.add_argument("--landing-descent-speed", type=float, default=0.20, help="Descent speed in m/s when marker is centered")
    parser.add_argument("--landing-center-tolerance-px", type=int, default=70, help="Allowed pixel error before descending")
    parser.add_argument("--landing-min-alt", type=float, default=0.60, help="Stop visual descent below this relative altitude")
    parser.add_argument("--landing-final-land", action="store_true", help="Request ArduPilot LAND when centered below landing-min-alt")
    parser.add_argument("--landing-invert-x", action="store_true", help="Invert forward/back correction if camera orientation requires it")
    parser.add_argument("--landing-invert-y", action="store_true", help="Invert left/right correction if camera orientation requires it")
    args = parser.parse_args()

    if args.no_flight_controller:
        if args.enable_autolanding:
            parser.error("--enable-autolanding cannot be used with --no-flight-controller")
        run_camera_only(args)
        return

    reader = MavlinkReader(args.connection, args.baud)
    reader.connect()

    vibration = VibrationAnalyzer(window_size=100)
    logger = CsvLogger(args.log_dir)

    start = time.time()
    last_print = 0.0

    print(f"Health test started for {args.duration} seconds.")
    print("Safety: first tests should be done without propellers.")

    marker_detector = None
    draw_marker_detection = None
    camera = None
    marker_detected_once = False
    marker_detection_elapsed = None
    marker_best_detection = None
    marker_email_sent = False
    landing_controller = None
    last_landing_print = 0.0

    if args.autoland_on_marker:
        args.detect_marker = True
        from landing_controller import VisualLandingController
        landing_controller = VisualLandingController(
            master=reader.master,
            enable_commands=args.enable_autolanding,
            require_guided=args.landing_require_guided,
            kp_xy=args.landing_kp_xy,
            kp_position=args.landing_kp_position,
            calibration_csv=args.landing_calibration_csv,
            marker_size_m=args.landing_marker_size_m,
            use_calibration=not args.landing_disable_calibration,
            max_xy_speed_mps=args.landing_max_xy_speed,
            descent_speed_mps=args.landing_descent_speed,
            center_tolerance_px=args.landing_center_tolerance_px,
            min_landing_alt_m=args.landing_min_alt,
            final_land=args.landing_final_land,
            invert_x=args.landing_invert_x,
            invert_y=args.landing_invert_y,
        )
        mode = "REAL MAVLink commands" if args.enable_autolanding else "DRY-RUN only"
        print(f"Autonomous visual landing enabled: {mode}")
        if landing_controller.calibration is not None:
            print(f"Camera calibration loaded: {landing_controller.calibration.describe()}")
        elif args.landing_disable_calibration:
            print("Camera calibration disabled: using pixel-gain fallback.")
        else:
            print(f"Camera calibration unavailable: {landing_controller.calibration_error}. Using pixel-gain fallback.")
        print("Safety: test first in SITL or with propellers removed. The script does not arm or take off.")

    if args.detect_marker:
        from camera_capture import CameraCapture
        if args.marker_type == "aruco":
            from aruco_marker_detector import ArucoMarkerDetector, draw_aruco_detection
            marker_detector = ArucoMarkerDetector(marker_id=args.aruco_id, dictionary_name=args.aruco_dictionary)
            draw_marker_detection = draw_aruco_detection
            print(f"Visual marker detection enabled: ArUco {args.aruco_dictionary} ID {args.aruco_id}")
        else:
            from visual_marker_detector import VisualMarkerDetector, draw_detection
            marker_detector = VisualMarkerDetector(args.marker_reference, threshold=args.marker_threshold)
            draw_marker_detection = draw_detection
            print(f"Visual marker detection enabled: references {', '.join(marker_detector.reference_names)}")

        camera = CameraCapture(camera_index=args.camera_index, backend=args.camera_backend)
        camera.open()
        print(f"Camera backend: {camera.active_backend}")

    try:
        while time.time() - start < args.duration:
            tel = reader.read_messages()

            vibration.add_sample(tel.acc_x, tel.acc_y, tel.acc_z)
            vib_rms = vibration.rms()

            cpu_temp = get_cpu_temp_c()
            cpu_usage = get_cpu_usage_percent()
            ram_usage = get_ram_usage_percent()

            health = check_drone_health(tel, vib_rms, cpu_temp, cpu_usage, ram_usage)
            logger.write(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health)

            marker_status_text = None
            if args.detect_marker and camera is not None and marker_detector is not None:
                import cv2
                from pathlib import Path

                camera_frame = camera.read()
                if camera_frame.ok:
                    frame = camera_frame.frame
                    marker_detection = marker_detector.detect(frame)
                    if marker_best_detection is None or marker_detection.score > marker_best_detection.score:
                        marker_best_detection = marker_detection
                    marker_status_text = marker_detection.message
                    if marker_detection.detected and not marker_detected_once:
                        marker_detected_once = True
                        marker_detection_elapsed = time.time() - start
                        detected_frame_path = Path(args.report_dir) / "marker_detected.jpg"
                        detected_frame_path.parent.mkdir(parents=True, exist_ok=True)
                        cv2.imwrite(str(detected_frame_path), draw_marker_detection(frame, marker_detection))
                        print(f"✅ Visual marker detected | {marker_detection.message} | detection time={marker_detection_elapsed:.2f}s")
                        print(f"Detected marker frame saved: {detected_frame_path}")
                        if args.email_on_marker and not marker_email_sent:
                            send_email_report(
                                subject="Drone Visual Marker Detected",
                                body=f"The drone camera detected the visual marker.\n{marker_detection.message}\nDetection time from test start: {marker_detection_elapsed:.2f} seconds",
                                attachments=[str(detected_frame_path)],
                            )
                            marker_email_sent = True
                            print("Marker detection email sent successfully.")

                    if landing_controller is not None:
                        landing_cmd = landing_controller.update(frame.shape, marker_detection, tel)
                        now_landing = time.time()
                        if now_landing - last_landing_print >= 1.0:
                            last_landing_print = now_landing
                            print(f"Landing controller: {landing_cmd.message}")
                else:
                    marker_status_text = f"Camera frame not received. {camera_frame.message}"

            now = time.time()
            if now - last_print >= config.STATUS_PRINT_INTERVAL_SECONDS:
                last_print = now
                remaining = int(args.duration - (now - start))
                print_status(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health, remaining)
                if marker_status_text is not None:
                    print(f"Visual marker: {marker_status_text}")

            time.sleep(config.LOG_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        logger.close()
        if landing_controller is not None:
            landing_controller.stop()
        if camera is not None:
            camera.close()

    print(f"CSV log saved: {logger.path}")

    report_path = generate_report(str(logger.path), args.report_dir)
    print(f"HTML report saved: {report_path}")

    summary = text_summary(str(logger.path))
    if args.detect_marker:
        if marker_detected_once:
            summary += (f"\n\nVisual marker test: DETECTED"
                        f"\nReference: {getattr(marker_best_detection, 'reference_name', None) or 'N/A'}"
                        f"\nConfidence: {marker_best_detection.score * 100:.1f}%"
                        f"\nDetection time: {marker_detection_elapsed:.2f} s")
        else:
            best_ref = getattr(marker_best_detection, 'reference_name', None) if marker_best_detection else None
            best_score = marker_best_detection.score * 100 if marker_best_detection else 0.0
            summary += (f"\n\nVisual marker test: NOT DETECTED"
                        f"\nBest reference: {best_ref or 'none'}"
                        f"\nBest confidence: {best_score:.1f}%")
    if args.autoland_on_marker:
        summary += "\nAutonomous landing controller: " + ("REAL COMMANDS ENABLED" if args.enable_autolanding else "DRY-RUN ONLY")
    print(summary)

    if args.send_email:
        send_email_report(
            subject="Drone Health Monitoring Report",
            body=summary,
            attachments=[str(logger.path), str(report_path)]
        )
        print("Email report sent successfully.")


def fmt(value, suffix="", digits=2):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}{suffix}"
    return f"{value}{suffix}"


def print_status(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health, remaining):
    print("-" * 80)
    print(f"Remaining: {remaining}s | Status: {health.status}")
    print(f"Mode: {tel.mode} | Armed: {tel.armed}")
    print(f"Battery: {fmt(tel.battery_voltage_v, ' V')} | Current: {fmt(tel.battery_current_a, ' A')} | Remaining: {fmt(tel.battery_remaining_percent, '%', 0)}")
    print(f"GPS: fix_type={tel.gps_fix_type} | satellites={tel.gps_satellites} | HDOP={fmt(tel.gps_hdop)}")
    print(f"RAW_IMU vibration RMS: {fmt(vib_rms, ' m/s^2')}")
    print(f"ArduPilot VIBE: X={fmt(tel.vibe_x)} | Y={fmt(tel.vibe_y)} | Z={fmt(tel.vibe_z)}")
    print(f"IMU clipping: {tel.clipping_0}, {tel.clipping_1}, {tel.clipping_2}")
    print(f"Pi CPU: {cpu_usage:.1f}% | RAM: {ram_usage:.1f}% | Temp: {fmt(cpu_temp, ' C')}")
    print("Messages:")
    for msg in health.messages:
        print(f" - {msg}")


if __name__ == "__main__":
    main()
