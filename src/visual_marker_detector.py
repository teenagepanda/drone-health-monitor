from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class MarkerDetection:
    detected: bool
    score: float = 0.0
    corners: Optional[np.ndarray] = None
    message: str = "Marker not detected"


class VisualMarkerDetector:
    """
    Detects the project's black-square visual marker with a white internal pattern.

    Method:
    1. Look for a large dark square/rectangle in the camera frame.
    2. Warp that square to a fixed size.
    3. Compare its binary pattern to the saved reference marker image.

    The detector is intended for preflight/bench testing and for confirming that the
    camera can see the marker. Do not use it alone for autonomous landing control
    without additional distance/pose estimation and flight safety checks.
    """

    def __init__(self, reference_image_path: str | Path, marker_size: int = 240, min_area: int = 2500, threshold: float = 0.72):
        self.reference_image_path = Path(reference_image_path)
        self.marker_size = marker_size
        self.min_area = min_area
        self.threshold = threshold

        ref = cv2.imread(str(self.reference_image_path), cv2.IMREAD_GRAYSCALE)
        if ref is None:
            raise FileNotFoundError(f"Reference marker image not found: {self.reference_image_path}")

        ref = cv2.resize(ref, (self.marker_size, self.marker_size))
        self.reference_binary = self._to_binary(ref)
        self.reference_rotations = [np.rot90(self.reference_binary, k) for k in range(4)]

    def detect(self, frame: np.ndarray) -> MarkerDetection:
        if frame is None or frame.size == 0:
            return MarkerDetection(False, message="Empty camera frame")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Dark regions become white in this mask, making the black marker square easy to find.
        _, dark_mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue
            aspect = w / float(h)

            # The marker is almost square, but camera angle can distort it.
            if len(approx) >= 4 and 0.65 <= aspect <= 1.55:
                candidates.append((area, contour))

        best = MarkerDetection(False, message="Marker not detected")
        for _, contour in sorted(candidates, key=lambda item: item[0], reverse=True)[:5]:
            corners = self._contour_to_corners(contour)
            warped = self._warp(gray, corners)
            candidate_binary = self._to_binary(warped)
            score = self._score(candidate_binary)

            if score > best.score:
                best = MarkerDetection(score >= self.threshold, score=score, corners=corners, message=f"Marker score: {score:.2f}")

            if best.detected:
                best.message = f"Visual marker detected. Score: {score:.2f}"
                return best

        return best

    def _to_binary(self, gray: np.ndarray) -> np.ndarray:
        # White pattern = 1, black background = 0.
        _, binary = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary.astype(np.uint8)

    def _score(self, candidate_binary: np.ndarray) -> float:
        scores = []
        for ref in self.reference_rotations:
            same = (candidate_binary == ref).mean()
            scores.append(float(same))
        return max(scores)

    def _contour_to_corners(self, contour: np.ndarray) -> np.ndarray:
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        return self._order_points(box.astype("float32"))

    def _order_points(self, pts: np.ndarray) -> np.ndarray:
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]      # top-left
        rect[2] = pts[np.argmax(s)]      # bottom-right
        rect[1] = pts[np.argmin(diff)]   # top-right
        rect[3] = pts[np.argmax(diff)]   # bottom-left
        return rect

    def _warp(self, gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
        dst = np.array([
            [0, 0],
            [self.marker_size - 1, 0],
            [self.marker_size - 1, self.marker_size - 1],
            [0, self.marker_size - 1],
        ], dtype="float32")
        matrix = cv2.getPerspectiveTransform(corners, dst)
        return cv2.warpPerspective(gray, matrix, (self.marker_size, self.marker_size))


def draw_detection(frame: np.ndarray, detection: MarkerDetection) -> np.ndarray:
    output = frame.copy()
    if detection.corners is not None:
        pts = detection.corners.astype(int).reshape((-1, 1, 2))
        cv2.polylines(output, [pts], True, (0, 255, 0) if detection.detected else (0, 165, 255), 2)
    label = "DETECTED" if detection.detected else "NOT DETECTED"
    cv2.putText(output, f"Marker: {label} | score={detection.score:.2f}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if detection.detected else (0, 165, 255), 2)
    return output
