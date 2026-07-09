import argparse
import time
from pathlib import Path

import cv2

from email_sender import send_email_report
from visual_marker_detector import VisualMarkerDetector, draw_detection


def main():
    parser = argparse.ArgumentParser(description="Run camera test for visual-marker detection")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index, usually 0")
    parser.add_argument("--reference", default="markers/reference_marker.png", help="Reference marker image path")
    parser.add_argument("--duration", type=int, default=60, help="Test duration in seconds")
    parser.add_argument("--threshold", type=float, default=0.72, help="Detection score threshold")
    parser.add_argument("--send-email", action="store_true", help="Send email when marker is detected")
    parser.add_argument("--show", action="store_true", help="Show camera window on desktop")
    parser.add_argument("--save-detected-frame", default="reports/marker_detected.jpg", help="Path to save detected frame")
    args = parser.parse_args()

    detector = VisualMarkerDetector(args.reference, threshold=args.threshold)
    cap = cv2.VideoCapture(args.camera_index)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera_index}")

    print(f"Visual marker test started for {args.duration} seconds.")
    print("Show the marker to the camera. Press q to stop if --show is used.")

    start = time.time()
    last_print = 0.0
    email_sent = False
    detected_once = False
    saved_frame_path = None

    try:
        while time.time() - start < args.duration:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame not received.")
                time.sleep(0.2)
                continue

            detection = detector.detect(frame)
            now = time.time()

            if now - last_print >= 1.0:
                last_print = now
                if detection.detected:
                    print(f"✅ Visual marker detected | score={detection.score:.2f}")
                else:
                    print(f"Marker not detected | best score={detection.score:.2f}")

            if detection.detected and not detected_once:
                detected_once = True
                save_path = Path(args.save_detected_frame)
                save_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(save_path), draw_detection(frame, detection))
                saved_frame_path = str(save_path)
                print(f"Detected frame saved: {saved_frame_path}")

                if args.send_email and not email_sent:
                    send_email_report(
                        subject="Drone Visual Marker Detected",
                        body=f"The drone camera detected the visual marker. Detection score: {detection.score:.2f}",
                        attachments=[saved_frame_path],
                    )
                    email_sent = True
                    print("Email notification sent successfully.")

            if args.show:
                cv2.imshow("Visual Marker Test", draw_detection(frame, detection))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.05)
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    if detected_once:
        print("FINAL RESULT: visual marker was detected.")
    else:
        print("FINAL RESULT: visual marker was not detected during the test.")


if __name__ == "__main__":
    main()
