import argparse
import time

from mavlink_reader import MavlinkReader
from vibration import VibrationAnalyzer
from health_checks import check_drone_health
from logger import CsvLogger
from pi_health import get_cpu_temp_c, get_cpu_usage_percent, get_ram_usage_percent
from report_generator import generate_report, text_summary
from email_config import load_email_config
from email_sender import send_email_report
import config


def main():
    email_config = load_email_config()

    if email_config is None:
        return
    parser = argparse.ArgumentParser(description="Run timed drone health test and optionally send email report")
    parser.add_argument("--connection", default=config.DEFAULT_CONNECTION)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--duration", type=int, default=180, help="Test duration in seconds")
    parser.add_argument("--send-email", action="store_true", help="Send report to email after test")
    parser.add_argument("--detect-marker", action="store_true", help="Also test camera visual-marker detection during the health test")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index for visual-marker test")
    parser.add_argument("--marker-reference", default="markers/reference_marker.png", help="Reference marker image path")
    parser.add_argument("--marker-threshold", type=float, default=0.72, help="Visual-marker detection score threshold")
    parser.add_argument("--email-on-marker", action="store_true", help="Send email immediately when the visual marker is detected")
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--report-dir", default="reports")
    args = parser.parse_args()

    reader = MavlinkReader(args.connection, args.baud)
    reader.connect()

    vibration = VibrationAnalyzer(window_size=100)
    logger = CsvLogger(args.log_dir)

    start = time.time()
    last_print = 0.0

    print(f"Health test started for {args.duration} seconds.")
    print("Safety: first tests should be done without propellers.")

    marker_detector = None
    camera = None
    marker_detected_once = False
    marker_email_sent = False

    if args.detect_marker:
        import cv2
        from visual_marker_detector import VisualMarkerDetector, draw_detection

        marker_detector = VisualMarkerDetector(args.marker_reference, threshold=args.marker_threshold)
        camera = cv2.VideoCapture(args.camera_index)
        if not camera.isOpened():
            raise RuntimeError(f"Could not open camera index {args.camera_index}")
        print("Visual marker detection enabled.")

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
                from visual_marker_detector import draw_detection

                ok, frame = camera.read()
                if ok:
                    marker_detection = marker_detector.detect(frame)
                    marker_status_text = f"Marker detected={marker_detection.detected} | score={marker_detection.score:.2f}"
                    if marker_detection.detected and not marker_detected_once:
                        marker_detected_once = True
                        detected_frame_path = Path(args.report_dir) / "marker_detected.jpg"
                        detected_frame_path.parent.mkdir(parents=True, exist_ok=True)
                        cv2.imwrite(str(detected_frame_path), draw_detection(frame, marker_detection))
                        print(f"✅ Visual marker detected | score={marker_detection.score:.2f}")
                        print(f"Detected marker frame saved: {detected_frame_path}")
                        if args.email_on_marker and not marker_email_sent:
                            send_email_report(
                                subject="Drone Visual Marker Detected",
                                body=f"The drone camera detected the visual marker. Detection score: {marker_detection.score:.2f}",
                                attachments=[str(detected_frame_path)],
                            )
                            marker_email_sent = True
                            print("Marker detection email sent successfully.")
                else:
                    marker_status_text = "Camera frame not received"

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
        if camera is not None:
            camera.release()

    print(f"CSV log saved: {logger.path}")

    report_path = generate_report(str(logger.path), args.report_dir)
    print(f"HTML report saved: {report_path}")

    summary = text_summary(str(logger.path))
    if args.detect_marker:
        summary += "\n\nVisual marker test: " + ("DETECTED" if marker_detected_once else "NOT DETECTED")
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
