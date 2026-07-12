from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class ArucoDetection:
    detected: bool
    marker_id: Optional[int] = None
    corners: Optional[np.ndarray] = None
    center: Optional[Tuple[int, int]] = None
    message: str = "ArUco marker not detected"


class ArucoMarkerDetector:
    """
    Detects an ArUco marker for the drone visual-marker test.

    Recommended for the project instead of template matching because ArUco is
    more stable under rotation, distance changes, and perspective distortion.
    """

    DICTIONARIES = {
        "4x4_50": cv2.aruco.DICT_4X4_50,
        "4x4_100": cv2.aruco.DICT_4X4_100,
        "5x5_50": cv2.aruco.DICT_5X5_50,
        "5x5_100": cv2.aruco.DICT_5X5_100,
        "6x6_50": cv2.aruco.DICT_6X6_50,
        "6x6_100": cv2.aruco.DICT_6X6_100,
    }

    def __init__(self, marker_id: int = 23, dictionary_name: str = "4x4_50"):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco module is missing. Install python3-opencv on Raspberry Pi "
                "and use a venv created with --system-site-packages."
            )
        if dictionary_name not in self.DICTIONARIES:
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")

        self.marker_id = int(marker_id)
        self.dictionary_name = dictionary_name
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.DICTIONARIES[dictionary_name])

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.parameters = cv2.aruco.DetectorParameters()
        else:
            self.parameters = cv2.aruco.DetectorParameters_create()

        self._detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, frame: np.ndarray) -> ArucoDetection:
        if frame is None or frame.size == 0:
            return ArucoDetection(False, message="Empty camera frame")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is None or len(ids) == 0:
            return ArucoDetection(False, message="No ArUco markers detected")

        ids_flat = ids.flatten().tolist()
        if self.marker_id not in ids_flat:
            return ArucoDetection(False, message=f"Detected ArUco IDs {ids_flat}, but not target ID {self.marker_id}")

        idx = ids_flat.index(self.marker_id)
        marker_corners = corners[idx][0].astype(np.float32)
        center_x = int(marker_corners[:, 0].mean())
        center_y = int(marker_corners[:, 1].mean())

        return ArucoDetection(
            True,
            marker_id=self.marker_id,
            corners=marker_corners,
            center=(center_x, center_y),
            message=f"ArUco marker ID {self.marker_id} detected",
        )


def draw_aruco_detection(frame: np.ndarray, detection: ArucoDetection) -> np.ndarray:
    output = frame.copy()
    if detection.corners is not None:
        pts = detection.corners.astype(int).reshape((-1, 1, 2))
        cv2.polylines(output, [pts], True, (0, 255, 0), 2)
        if detection.center is not None:
            cv2.circle(output, detection.center, 5, (0, 255, 0), -1)

    label = "ARUCO DETECTED" if detection.detected else "ARUCO NOT DETECTED"
    cv2.putText(output, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if detection.detected else (0, 165, 255), 2)
    if detection.marker_id is not None:
        cv2.putText(output, f"ID: {detection.marker_id}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return output
