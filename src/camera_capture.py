from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2


@dataclass
class CameraFrame:
    ok: bool
    frame: Optional[object] = None
    message: str = ""


class CameraCapture:
    """
    Unified camera reader for the drone project.

    Raspberry Pi CSI cameras on Raspberry Pi OS Bookworm work best through
    Picamera2/libcamera. OpenCV VideoCapture is kept only as a fallback for
    USB cameras or older systems.
    """

    def __init__(self, camera_index: int = 0, backend: str = "auto", width: int = 1280, height: int = 720):
        self.camera_index = camera_index
        self.backend = backend
        self.width = width
        self.height = height
        self._picam2 = None
        self._cap = None
        self.active_backend = None

    def open(self) -> None:
        if self.backend in ("auto", "picamera2"):
            try:
                from picamera2 import Picamera2

                self._picam2 = Picamera2(camera_num=self.camera_index)
                config = self._picam2.create_preview_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"}
                )
                self._picam2.configure(config)
                self._picam2.start()
                self.active_backend = "picamera2"
                return
            except Exception as exc:
                self._picam2 = None
                if self.backend == "picamera2":
                    raise RuntimeError(f"Could not open camera with Picamera2: {exc}") from exc

        if self.backend in ("auto", "opencv"):
            self._cap = cv2.VideoCapture(self.camera_index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self._cap.isOpened():
                raise RuntimeError(f"Could not open camera index {self.camera_index} with OpenCV")
            self.active_backend = "opencv"
            return

        raise ValueError("backend must be: auto, picamera2, or opencv")

    def read(self) -> CameraFrame:
        if self.active_backend == "picamera2" and self._picam2 is not None:
            try:
                frame_rgb = self._picam2.capture_array()
                if frame_rgb is None:
                    return CameraFrame(False, message="Picamera2 returned empty frame")
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                return CameraFrame(True, frame=frame_bgr)
            except Exception as exc:
                return CameraFrame(False, message=f"Picamera2 frame error: {exc}")

        if self.active_backend == "opencv" and self._cap is not None:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                return CameraFrame(False, message="OpenCV camera frame not received")
            return CameraFrame(True, frame=frame)

        return CameraFrame(False, message="Camera is not open")

    def close(self) -> None:
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
            self._picam2 = None

        if self._cap is not None:
            self._cap.release()
            self._cap = None
