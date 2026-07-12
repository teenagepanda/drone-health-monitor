import argparse
import time
from pathlib import Path

import cv2

from camera_capture import CameraCapture
from email_sender import send_email_report


def main():
    parser = argparse.ArgumentParser(description="Run camera test for visual-marker detection")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index, usually 0")
    parser.add_argument("--camera-backend", choices=["auto", "picamera2", "opencv"], default="auto")
    parser.add_argument("--marker-type", choices=["aruco", "template"], default="template")
    parser.add_argument("--aruco-id", type=int, default=23)
    parser.add_argument("--aruco-dictionary", choices=["4x4_50", "4x4_100", "5x5_50", "5x5_100", "6x6_50", "6x6_100"], default="4x4_50")
    parser.add_argument("--reference", default="markers/references", help="Reference image or directory. Order: original, A, B, C ...")
    parser.add_argument("--duration", type=int, default=60)
    parser.add_argument("--threshold", type=float, default=0.72)
    parser.add_argument("--send-email", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--save-detected-frame", default="reports/marker_detected.jpg")
    args = parser.parse_args()

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

    start = time.time()
    first_detection_elapsed = None
    best_detection = None
    last_print = 0.0
    email_sent = False
    detected_once = False
    frame_count = 0
    fps = 0.0
    fps_window_start = time.perf_counter()

    try:
        while time.time() - start < args.duration:
            camera_frame = cap.read()
            if not camera_frame.ok:
                print(f"Camera frame not received. {camera_frame.message}")
                time.sleep(0.2)
                continue

            frame_count += 1
            frame = camera_frame.frame
            detection = detector.detect(frame)
            now = time.time()
            elapsed = now - start

            fps_elapsed = time.perf_counter() - fps_window_start
            if fps_elapsed >= 1.0:
                fps = frame_count / fps_elapsed
                frame_count = 0
                fps_window_start = time.perf_counter()

            if best_detection is None or detection.score > best_detection.score:
                best_detection = detection

            if now - last_print >= 1.0:
                last_print = now
                if detection.detected:
                    print(
                        f"✅ Marker detected | reference={getattr(detection, 'reference_name', None) or 'N/A'} | "
                        f"confidence={detection.score * 100:.1f}% | elapsed={elapsed:.2f}s | "
                        f"processing={getattr(detection, 'processing_time_s', 0.0) * 1000:.1f}ms | FPS={fps:.1f}"
                    )
                else:
                    print(f"Marker not detected | {detection.message} | elapsed={elapsed:.2f}s | FPS={fps:.1f}")

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
                    state = "DETECTED" if detection.detected else "SEARCHING"
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
