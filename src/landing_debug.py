from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2


class LandingDebugLogger:
    FIELDS = [
        "timestamp", "elapsed_s", "state", "detected", "centered",
        "reference", "confidence_percent", "stable_frames", "marker_side_px",
        "telemetry_alt_m", "visual_alt_m", "control_alt_m", "altitude_source",
        "meters_per_pixel", "error_x_px", "error_y_px", "error_x_m",
        "error_y_m", "vx_mps", "vy_mps", "vz_mps", "command_sent",
        "block_reason",
    ]

    def __init__(self, log_dir: str = "logs") -> None:
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"landing_debug_{stamp}.csv"
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.FIELDS)
        self._writer.writeheader()

    def write(self, command, elapsed_s: float) -> None:
        row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "elapsed_s": f"{elapsed_s:.3f}",
            "state": command.state,
            "detected": command.detected,
            "centered": command.centered,
            "reference": command.reference_name,
            "confidence_percent": f"{command.confidence_percent:.3f}",
            "stable_frames": command.stable_frames,
            "marker_side_px": self._fmt(command.marker_side_px),
            "telemetry_alt_m": self._fmt(command.telemetry_alt_m),
            "visual_alt_m": self._fmt(command.visual_alt_m),
            "control_alt_m": self._fmt(command.control_alt_m),
            "altitude_source": command.altitude_source,
            "meters_per_pixel": self._fmt(command.meters_per_pixel, 8),
            "error_x_px": self._fmt(command.error_x_px),
            "error_y_px": self._fmt(command.error_y_px),
            "error_x_m": self._fmt(command.error_x_m, 6),
            "error_y_m": self._fmt(command.error_y_m, 6),
            "vx_mps": f"{command.vx_mps:.5f}",
            "vy_mps": f"{command.vy_mps:.5f}",
            "vz_mps": f"{command.vz_mps:.5f}",
            "command_sent": command.command_sent,
            "block_reason": command.block_reason,
        }
        self._writer.writerow(row)
        self._handle.flush()

    def close(self) -> None:
        if not self._handle.closed:
            self._handle.close()

    @staticmethod
    def _fmt(value: Optional[float], digits: int = 4) -> str:
        return "" if value is None else f"{value:.{digits}f}"


def draw_landing_debug(frame, detection, command):
    """Draw landing-controller diagnostics over a copy of the camera frame."""
    output = frame.copy()
    h, w = output.shape[:2]
    frame_center = (w // 2, h // 2)
    cv2.drawMarker(output, frame_center, (255, 255, 0), cv2.MARKER_CROSS, 28, 2)

    corners = getattr(detection, "corners", None)
    marker_center = getattr(detection, "center", None)
    if corners is not None:
        points = corners.astype(int).reshape((-1, 1, 2))
        cv2.polylines(output, [points], True, (0, 255, 0), 3)
    if marker_center is not None:
        marker_center = (int(marker_center[0]), int(marker_center[1]))
        cv2.circle(output, marker_center, 7, (0, 0, 255), -1)
        cv2.arrowedLine(
            output, frame_center, marker_center, (255, 0, 255), 2, tipLength=0.12
        )

    lines = [
        f"Landing: {command.state} | {'REAL' if command.command_sent else 'DRY/BLOCKED'}",
        f"Ref: {command.reference_name} | Confidence: {command.confidence_percent:.1f}%",
        f"Stable: {command.stable_frames} | Centered: {command.centered}",
        f"Altitude visual/FC/control: {_fmt(command.visual_alt_m)} / "
        f"{_fmt(command.telemetry_alt_m)} / {_fmt(command.control_alt_m)} m",
        f"Source: {command.altitude_source} | Marker: {_fmt(command.marker_side_px, 1)} px",
        f"Error px: ({_fmt(command.error_x_px, 0)}, {_fmt(command.error_y_px, 0)})",
        f"Error m: ({_fmt(command.error_x_m, 3)}, {_fmt(command.error_y_m, 3)})",
        f"Command m/s: vx={command.vx_mps:+.2f} vy={command.vy_mps:+.2f} "
        f"vz={command.vz_mps:+.2f}",
    ]
    if command.block_reason:
        lines.append(f"Block: {command.block_reason}")

    panel_h = 28 * len(lines) + 16
    overlay = output.copy()
    cv2.rectangle(overlay, (8, 8), (760, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.62, output, 0.38, 0, output)
    for index, text in enumerate(lines):
        cv2.putText(
            output, text, (20, 34 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2
        )
    return output


def _fmt(value: Optional[float], digits: int = 2) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"
