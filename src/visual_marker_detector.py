from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import time

import cv2
import numpy as np


@dataclass
class MarkerDetection:
    detected: bool
    score: float = 0.0
    corners: Optional[np.ndarray] = None
    message: str = "Marker not detected"
    reference_name: Optional[str] = None
    processing_time_s: float = 0.0

    @property
    def center(self) -> Optional[tuple[int, int]]:
        if self.corners is None:
            return None
        pts = self.corners.reshape(-1, 2)
        return int(pts[:, 0].mean()), int(pts[:, 1].mean())


@dataclass(frozen=True)
class _ReferenceTemplate:
    name: str
    path: Path
    rotations: tuple[np.ndarray, ...]


class VisualMarkerDetector:
    """Multi-reference detector for the original printed visual marker.

    References are checked in this deterministic fallback order:
    original -> A -> B -> C -> D -> E -> remaining names.
    All references are loaded once during program startup.
    """

    def __init__(
        self,
        reference_image_path: str | Path = "markers/references",
        marker_size: int = 240,
        min_area: int = 900,
        threshold: float = 0.72,
    ):
        self.reference_image_path = Path(reference_image_path)
        self.marker_size = marker_size
        self.min_area = min_area
        self.threshold = threshold
        self.references = self._load_references(self.reference_image_path)

    def _load_references(self, source: Path) -> list[_ReferenceTemplate]:
        if source.is_file():
            paths = [source]
        elif source.is_dir():
            allowed = {".png", ".jpg", ".jpeg", ".bmp"}
            paths = [
                p for p in source.iterdir()
                if p.is_file() and p.suffix.lower() in allowed and not p.name.startswith("_")
            ]
            paths.sort(key=self._reference_sort_key)
        else:
            raise FileNotFoundError(f"Reference path not found: {source}")

        if not paths:
            raise FileNotFoundError(f"No reference images found in: {source}")

        references: list[_ReferenceTemplate] = []
        for path in paths:
            ref = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if ref is None:
                raise ValueError(f"Could not read reference image: {path}")
            ref = cv2.resize(ref, (self.marker_size, self.marker_size), interpolation=cv2.INTER_AREA)
            binary = self._to_binary(ref)
            rotations = tuple(np.rot90(binary, k).copy() for k in range(4))
            references.append(_ReferenceTemplate(path.stem, path, rotations))
        return references

    @staticmethod
    def _reference_sort_key(path: Path) -> tuple[int, str]:
        stem = path.stem.strip()
        if stem.lower() == "original":
            return (0, "")
        if len(stem) == 1 and stem.isalpha():
            return (1, stem.upper())
        return (2, stem.lower())

    @property
    def reference_names(self) -> list[str]:
        return [ref.name for ref in self.references]

    def detect(self, frame: np.ndarray) -> MarkerDetection:
        process_start = time.perf_counter()
        if frame is None or frame.size == 0:
            return MarkerDetection(False, message="Empty camera frame")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, dark_mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[tuple[float, np.ndarray]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            _, _, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue
            aspect = w / float(h)
            if 0.62 <= aspect <= 1.62:
                candidates.append((area, contour))

        best = MarkerDetection(False, message="Marker not detected")
        for _, contour in sorted(candidates, key=lambda item: item[0], reverse=True)[:8]:
            corners = self._contour_to_corners(contour)
            warped = self._warp(gray, corners)
            candidate_binary = self._to_binary(warped)

            for reference in self.references:
                score = self._score_against_reference(candidate_binary, reference)
                if score > best.score:
                    best = MarkerDetection(
                        detected=score >= self.threshold,
                        score=score,
                        corners=corners,
                        reference_name=reference.name,
                    )
                if score >= self.threshold:
                    elapsed = time.perf_counter() - process_start
                    return MarkerDetection(
                        True,
                        score=score,
                        corners=corners,
                        reference_name=reference.name,
                        processing_time_s=elapsed,
                        message=(f"Visual marker detected | reference={reference.name} | "
                                 f"confidence={score * 100:.1f}% | frame processing={elapsed:.3f}s"),
                    )

        elapsed = time.perf_counter() - process_start
        best.processing_time_s = elapsed
        best.message = (
            f"Marker not detected | best reference={best.reference_name or 'none'} | "
            f"best confidence={best.score * 100:.1f}% | frame processing={elapsed:.3f}s"
        )
        return best

    def _to_binary(self, gray: np.ndarray) -> np.ndarray:
        _, binary = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary.astype(np.uint8)

    @staticmethod
    def _score_against_reference(candidate_binary: np.ndarray, reference: _ReferenceTemplate) -> float:
        return max(float((candidate_binary == rotation).mean()) for rotation in reference.rotations)

    def _contour_to_corners(self, contour: np.ndarray) -> np.ndarray:
        rect = cv2.minAreaRect(contour)
        return self._order_points(cv2.boxPoints(rect).astype("float32"))

    @staticmethod
    def _order_points(pts: np.ndarray) -> np.ndarray:
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
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


def draw_detection(
    frame: np.ndarray,
    detection: MarkerDetection,
    *,
    elapsed_s: Optional[float] = None,
    fps: Optional[float] = None,
    system_state: Optional[str] = None,
) -> np.ndarray:
    """Draw marker geometry and telemetry overlay on a video frame."""
    output = frame.copy()
    h, w = output.shape[:2]
    frame_center = (w // 2, h // 2)
    cv2.drawMarker(output, frame_center, (255, 255, 0), cv2.MARKER_CROSS, 26, 2)

    color = (0, 255, 0) if detection.detected else (0, 165, 255)
    marker_center = detection.center
    if detection.corners is not None:
        pts = detection.corners.astype(int).reshape((-1, 1, 2))
        cv2.polylines(output, [pts], True, color, 3)

    offset_x = None
    offset_y = None
    if marker_center is not None:
        offset_x = marker_center[0] - frame_center[0]
        offset_y = marker_center[1] - frame_center[1]
        cv2.circle(output, marker_center, 7, (0, 0, 255), -1)
        cv2.arrowedLine(output, frame_center, marker_center, (255, 0, 255), 2, tipLength=0.12)

    state = system_state or ("ALIGNING" if detection.detected else "SEARCHING")
    ref = detection.reference_name or "none"
    lines = [
        f"State: {state}",
        f"Reference: {ref}",
        f"Confidence: {detection.score * 100:.1f}%",
        f"Frame processing: {detection.processing_time_s * 1000:.1f} ms",
    ]
    if elapsed_s is not None:
        lines.append(f"Elapsed: {elapsed_s:.2f} s")
    if fps is not None:
        lines.append(f"FPS: {fps:.1f}")
    if offset_x is not None and offset_y is not None:
        lines.append(f"Offset: X={offset_x:+d}px Y={offset_y:+d}px")

    panel_h = 28 * len(lines) + 16
    overlay = output.copy()
    cv2.rectangle(overlay, (8, 8), (510, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, output, 0.42, 0, output)
    for i, text in enumerate(lines):
        cv2.putText(output, text, (20, 34 + i * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    return output
