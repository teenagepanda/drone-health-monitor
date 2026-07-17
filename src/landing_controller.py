from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil

try:
    import cv2
except ImportError:  # Image saving is optional; flight control remains available.
    cv2 = None

from camera_calibration import CameraCalibration


@dataclass
class LandingCommand:
    active: bool
    detected: bool
    centered: bool
    state: str = "INACTIVE"
    confidence_percent: float = 0.0
    reference_name: str = "none"
    stable_frames: int = 0
    marker_side_px: Optional[float] = None
    telemetry_alt_m: Optional[float] = None
    visual_alt_m: Optional[float] = None
    control_alt_m: Optional[float] = None
    altitude_source: str = "none"
    meters_per_pixel: Optional[float] = None
    error_x_px: Optional[float] = None
    error_y_px: Optional[float] = None
    error_x_m: Optional[float] = None
    error_y_m: Optional[float] = None
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    command_sent: bool = False
    block_reason: str = ""
    message: str = "Landing controller inactive"
    saved_image_path: str = ""


class VisualLandingController:
    """Height-calibrated visual landing controller with safe dry-run diagnostics.

    Safety properties:
    - Commands are disabled unless ``enable_commands=True``.
    - The controller never arms or takes off.
    - Real movement is blocked unless the vehicle is armed and, by default,
      in GUIDED mode.
    - Marker loss commands a zero-velocity hold.
    - Descent starts only after consecutive high-confidence centered frames.
    """

    def __init__(
        self,
        master,
        enable_commands: bool = False,
        require_guided: bool = True,
        kp_xy: float = 0.0008,
        kp_position: float = 0.80,
        calibration_csv: str = "reports/height_calibration_clean.csv",
        marker_size_m: float = 0.20,
        use_calibration: bool = True,
        max_xy_speed_mps: float = 0.25,
        descent_speed_mps: float = 0.20,
        center_tolerance_px: int = 70,
        center_tolerance_m: float = 0.08,
        min_landing_alt_m: float = 0.30,
        command_interval_s: float = 0.20,
        final_land: bool = False,
        invert_x: bool = False,
        invert_y: bool = False,
        min_confidence: float = 0.90,
        required_reference: str = "original",
        stable_frames_required: int = 5,
        marker_lost_timeout_s: float = 0.75,
        altitude_source: str = "visual",
        save_marker_images: bool = True,
        marker_image_dir: str = "reports/marker_detections",
        marker_image_interval_s: float = 5.0,
        marker_image_max_count: int = 100,
        save_state_transition_images: bool = True,
    ):
        self.master = master
        self.enable_commands = bool(enable_commands)
        self.require_guided = bool(require_guided)
        self.kp_xy = float(kp_xy)
        self.kp_position = float(kp_position)
        self.marker_size_m = float(marker_size_m)
        self.max_xy_speed_mps = float(max_xy_speed_mps)
        self.descent_speed_mps = float(descent_speed_mps)
        self.center_tolerance_px = int(center_tolerance_px)
        self.center_tolerance_m = float(center_tolerance_m)
        self.min_landing_alt_m = float(min_landing_alt_m)
        self.command_interval_s = float(command_interval_s)
        self.final_land = bool(final_land)
        self.invert_x = bool(invert_x)
        self.invert_y = bool(invert_y)
        self.min_confidence = float(min_confidence)
        self.required_reference = required_reference.strip().lower()
        self.stable_frames_required = max(int(stable_frames_required), 1)
        self.marker_lost_timeout_s = max(float(marker_lost_timeout_s), 0.0)
        self.altitude_source = altitude_source.lower()
        self.save_marker_images = bool(save_marker_images)
        self.marker_image_dir = Path(marker_image_dir)
        self.marker_image_interval_s = max(float(marker_image_interval_s), 0.0)
        self.marker_image_max_count = max(int(marker_image_max_count), 0)
        self.save_state_transition_images = bool(save_state_transition_images)
        if self.altitude_source not in {"visual", "telemetry", "auto"}:
            raise ValueError("altitude_source must be visual, telemetry or auto")

        self.calibration = None
        self.calibration_error = None
        if use_calibration:
            try:
                calibration = CameraCalibration(
                    calibration_csv,
                    marker_size_m,
                    min_confidence_percent=85.0,
                )
                if calibration.available:
                    self.calibration = calibration
                else:
                    self.calibration_error = (
                        f"Fewer than two valid calibration points in {calibration_csv}"
                    )
            except Exception as exc:
                self.calibration_error = str(exc)

        self._last_command_time = 0.0
        self._last_seen_time = 0.0
        self._stable_frames = 0
        self._land_command_sent = False
        self._land_handoff_active = False
        self._last_marker_image_time = 0.0
        self._marker_image_count = 0
        self._last_capture_state = ""
        self.last_command = LandingCommand(False, False, False)

    def update(self, frame_shape, detection, telemetry, frame=None) -> LandingCommand:
        now = time.time()
        telemetry_alt = self._positive_float(
            getattr(telemetry, "relative_alt_m", None)
        )

        # Once LAND has been handed over to ArduPilot, do not send any more
        # velocity commands. ArduPilot must remain in full control of throttle,
        # touchdown detection and disarming.
        if self._land_handoff_active:
            armed = getattr(telemetry, "armed", None)
            landed = self._telemetry_reports_landed(telemetry)
            state = "LANDED" if (landed or armed is False) else "LAND_HANDOFF"
            cmd = LandingCommand(
                active=False,
                detected=bool(detection is not None and getattr(detection, "detected", False)),
                centered=False,
                state=state,
                telemetry_alt_m=telemetry_alt,
                command_sent=False,
                block_reason="ArduPilot owns landing and throttle control",
                message=(
                    f"REAL | state={state} | LAND command already sent; "
                    "visual velocity commands disabled until touchdown/disarm"
                ),
            )
            self.last_command = cmd
            return cmd
        confidence = float(getattr(detection, "score", 0.0) or 0.0)
        reference = str(
            getattr(detection, "reference_name", None) or "none"
        ).strip()

        valid_detection = bool(
            detection is not None and getattr(detection, "detected", False)
        )
        reject_reason = ""
        if not valid_detection:
            reject_reason = "marker not detected"
        elif confidence < self.min_confidence:
            reject_reason = (
                f"confidence {confidence * 100:.1f}% below "
                f"{self.min_confidence * 100:.1f}%"
            )
        elif (
            self.required_reference
            and reference.lower() != self.required_reference
        ):
            reject_reason = (
                f"reference {reference} is not {self.required_reference}"
            )

        if reject_reason:
            self._stable_frames = 0
            lost_for = now - self._last_seen_time if self._last_seen_time else 0.0
            sent, block = self._maybe_send_zero_hold(telemetry, now)
            state = "SEARCHING" if not self._last_seen_time else "MARKER_LOST"
            cmd = LandingCommand(
                active=False,
                detected=valid_detection,
                centered=False,
                state=state,
                confidence_percent=confidence * 100.0,
                reference_name=reference,
                stable_frames=0,
                telemetry_alt_m=telemetry_alt,
                command_sent=sent,
                block_reason=block or reject_reason,
                message=(
                    f"{self._mode_text()} | {state}: {reject_reason}; "
                    f"lost_for={lost_for:.2f}s; hold vx=0 vy=0 vz=0"
                ),
            )
            self.last_command = cmd
            return cmd

        center = self._get_detection_center(detection)
        marker_side_px = self._get_marker_side_px(detection)
        if center is None or marker_side_px is None or marker_side_px <= 0:
            self._stable_frames = 0
            cmd = LandingCommand(
                active=False,
                detected=True,
                centered=False,
                state="INVALID_GEOMETRY",
                confidence_percent=confidence * 100.0,
                reference_name=reference,
                block_reason="marker center or side length unavailable",
                message="Landing inactive: marker geometry unavailable",
            )
            self.last_command = cmd
            return cmd

        self._last_seen_time = now
        self._stable_frames += 1

        frame_h, frame_w = int(frame_shape[0]), int(frame_shape[1])
        center_x, center_y = center
        err_x_px = float(center_x - frame_w / 2.0)
        err_y_px = float(center_y - frame_h / 2.0)

        # The detected physical marker side gives a direct local image scale.
        meters_per_pixel = self.marker_size_m / marker_side_px
        err_x_m = err_x_px * meters_per_pixel
        err_y_m = err_y_px * meters_per_pixel

        visual_alt = None
        if self.calibration is not None:
            try:
                visual_alt = self.calibration.estimate_height_m(marker_side_px)
            except (ValueError, RuntimeError):
                visual_alt = None

        control_alt, altitude_source = self._select_altitude(
            telemetry_alt, visual_alt
        )

        centered = (
            abs(err_x_m) <= self.center_tolerance_m
            and abs(err_y_m) <= self.center_tolerance_m
        )
        stable = self._stable_frames >= self.stable_frames_required

        vx = self._clamp(
            -err_y_m * self.kp_position,
            -self.max_xy_speed_mps,
            self.max_xy_speed_mps,
        )
        vy = self._clamp(
            err_x_m * self.kp_position,
            -self.max_xy_speed_mps,
            self.max_xy_speed_mps,
        )
        if self.invert_x:
            vx = -vx
        if self.invert_y:
            vy = -vy

        vz = 0.0
        state = "ACQUIRING"
        if stable and not centered:
            state = "ALIGNING"
        elif stable and centered:
            state = "DESCENDING"
            vz = self.descent_speed_mps

        if control_alt is not None and control_alt <= self.min_landing_alt_m:
            vx = vy = vz = 0.0
            state = "HOLD_MIN_ALT"
            if centered and stable and self.final_land and not self._land_command_sent:
                sent, reason = self._send_land_command(telemetry)
                if sent:
                    self._land_command_sent = True
                    self._land_handoff_active = True
                    state = "FINAL_LAND"
                saved_image_path = self._maybe_save_marker_image(
                    frame=frame, detection=detection, confidence=confidence,
                    reference=reference, state=state, telemetry_alt=telemetry_alt,
                    visual_alt=visual_alt, now=now,
                )
                cmd = self._build_command(
                    True, centered, state, confidence, reference, marker_side_px,
                    telemetry_alt, visual_alt, control_alt, altitude_source,
                    meters_per_pixel, err_x_px, err_y_px, err_x_m, err_y_m,
                    vx, vy, vz, sent, reason,
                )
                cmd.saved_image_path = saved_image_path
                self.last_command = cmd
                return cmd

        saved_image_path = self._maybe_save_marker_image(
            frame=frame, detection=detection, confidence=confidence,
            reference=reference, state=state, telemetry_alt=telemetry_alt,
            visual_alt=visual_alt, now=now,
        )

        sent = False
        block_reason = ""
        if now - self._last_command_time >= self.command_interval_s:
            self._last_command_time = now
            sent, block_reason = self._send_velocity_command(
                vx, vy, vz, telemetry
            )

        cmd = self._build_command(
            True, centered, state, confidence, reference, marker_side_px,
            telemetry_alt, visual_alt, control_alt, altitude_source,
            meters_per_pixel, err_x_px, err_y_px, err_x_m, err_y_m,
            vx, vy, vz, sent, block_reason,
        )
        cmd.saved_image_path = saved_image_path
        self.last_command = cmd
        return cmd

    def stop(self) -> None:
        # Never override ArduPilot after LAND handoff. Sending a zero-velocity
        # command at that point could interfere with the landing controller.
        if (
            self.enable_commands
            and self.master is not None
            and not self._land_handoff_active
        ):
            self._send_velocity_command(0.0, 0.0, 0.0, None, force=True)

    def reset_for_new_flight(self) -> None:
        """Reset controller state only while the vehicle is safely disarmed."""
        self._last_command_time = 0.0
        self._last_seen_time = 0.0
        self._stable_frames = 0
        self._land_command_sent = False
        self._land_handoff_active = False
        self._last_marker_image_time = 0.0
        self._marker_image_count = 0
        self._last_capture_state = ""
        self.last_command = LandingCommand(False, False, False)

    def _build_command(
        self, detected, centered, state, confidence, reference, marker_side_px,
        telemetry_alt, visual_alt, control_alt, altitude_source,
        meters_per_pixel, err_x_px, err_y_px, err_x_m, err_y_m,
        vx, vy, vz, sent, block_reason,
    ) -> LandingCommand:
        command_status = "SENT" if sent else (
            f"BLOCKED({block_reason})" if block_reason else "DRY-RUN"
        )
        msg = (
            f"{self._mode_text()} | state={state} | ref={reference} "
            f"conf={confidence * 100:.1f}% stable={self._stable_frames}/"
            f"{self.stable_frames_required} | "
            f"alt_visual={self._fmt(visual_alt)}m "
            f"alt_fc={self._fmt(telemetry_alt)}m "
            f"alt_control={self._fmt(control_alt)}m({altitude_source}) | "
            f"marker={marker_side_px:.1f}px scale={meters_per_pixel * 1000:.3f}mm/px | "
            f"err=({err_x_px:+.0f},{err_y_px:+.0f})px "
            f"err_m=({err_x_m:+.3f},{err_y_m:+.3f}) | "
            f"cmd=({vx:+.2f},{vy:+.2f},{vz:+.2f})m/s {command_status}"
        )
        return LandingCommand(
            active=True,
            detected=detected,
            centered=centered,
            state=state,
            confidence_percent=confidence * 100.0,
            reference_name=reference,
            stable_frames=self._stable_frames,
            marker_side_px=marker_side_px,
            telemetry_alt_m=telemetry_alt,
            visual_alt_m=visual_alt,
            control_alt_m=control_alt,
            altitude_source=altitude_source,
            meters_per_pixel=meters_per_pixel,
            error_x_px=err_x_px,
            error_y_px=err_y_px,
            error_x_m=err_x_m,
            error_y_m=err_y_m,
            vx_mps=vx,
            vy_mps=vy,
            vz_mps=vz,
            command_sent=sent,
            block_reason=block_reason,
            message=msg,
        )

    def _maybe_save_marker_image(
        self, frame, detection, confidence: float, reference: str, state: str,
        telemetry_alt: Optional[float], visual_alt: Optional[float], now: float,
    ) -> str:
        """Save a rate-limited annotated evidence image for the final report."""
        if not self.save_marker_images or frame is None or cv2 is None:
            return ""
        if self.marker_image_max_count and self._marker_image_count >= self.marker_image_max_count:
            return ""
        state_changed = state != self._last_capture_state
        if (
            not (self.save_state_transition_images and state_changed)
            and now - self._last_marker_image_time < self.marker_image_interval_s
        ):
            return ""

        try:
            image = frame.copy()
            corners = getattr(detection, "corners", None)
            if corners is not None:
                pts = corners.reshape(-1, 2).astype("int32")
                cv2.polylines(image, [pts], True, (0, 255, 0), 2)

            center = self._get_detection_center(detection)
            if center is not None:
                cv2.circle(image, center, 6, (0, 0, 255), -1)

            lines = [
                f"Marker: {reference}",
                f"Confidence: {confidence * 100:.1f}%",
                f"State: {state}",
                f"Visual alt: {self._fmt(visual_alt)} m",
                f"FC alt: {self._fmt(telemetry_alt)} m",
            ]
            y = 28
            for line in lines:
                cv2.putText(
                    image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA,
                )
                y += 26

            self.marker_image_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            safe_state = state.lower().replace(" ", "_")
            filename = (
                f"marker_{timestamp}_{safe_state}_{reference}_"
                f"conf_{confidence * 100:.1f}.jpg"
            ).replace(" ", "_")
            path = self.marker_image_dir / filename
            if not cv2.imwrite(str(path), image):
                return ""

            self._last_marker_image_time = now
            self._last_capture_state = state
            self._marker_image_count += 1
            return str(path)
        except Exception:
            # Evidence capture must never interrupt landing control.
            return ""

    def _select_altitude(
        self, telemetry_alt: Optional[float], visual_alt: Optional[float]
    ) -> tuple[Optional[float], str]:
        if self.altitude_source == "visual":
            return (
                (visual_alt, "visual") if visual_alt is not None
                else (telemetry_alt, "telemetry-fallback")
            )
        if self.altitude_source == "telemetry":
            return (
                (telemetry_alt, "telemetry") if telemetry_alt is not None
                else (visual_alt, "visual-fallback")
            )
        # auto: visual calibration is preferred while marker is visible.
        return (
            (visual_alt, "visual") if visual_alt is not None
            else (telemetry_alt, "telemetry")
        )

    def _maybe_send_zero_hold(self, telemetry, now: float) -> tuple[bool, str]:
        if not self._last_seen_time:
            return False, "no prior marker lock"
        if now - self._last_seen_time < self.marker_lost_timeout_s:
            return False, "inside marker-loss grace period"
        if now - self._last_command_time < self.command_interval_s:
            return False, "command interval"
        self._last_command_time = now
        return self._send_velocity_command(0.0, 0.0, 0.0, telemetry)

    def _send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        telemetry,
        force: bool = False,
    ) -> tuple[bool, str]:
        if not self.enable_commands:
            return False, "commands disabled"
        if self.master is None:
            return False, "MAVLink master unavailable"

        if not force and telemetry is not None:
            if getattr(telemetry, "armed", None) is not True:
                return False, "vehicle not armed"
            if (
                self.require_guided
                and str(getattr(telemetry, "mode", "")).upper() != "GUIDED"
            ):
                return False, f"mode is {getattr(telemetry, 'mode', 'unknown')}, not GUIDED"

        type_mask = 0b0000111111000111
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )
        return True, ""

    def _send_land_command(self, telemetry) -> tuple[bool, str]:
        if not self.enable_commands:
            return False, "commands disabled"
        if self.master is None:
            return False, "MAVLink master unavailable"
        if getattr(telemetry, "armed", None) is not True:
            return False, "vehicle not armed"
        if (
            self.require_guided
            and str(getattr(telemetry, "mode", "")).upper() != "GUIDED"
        ):
            return False, "vehicle not in GUIDED"

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        return True, ""

    def _mode_text(self) -> str:
        return "REAL" if self.enable_commands else "DRY-RUN"


    @staticmethod
    def _telemetry_reports_landed(telemetry) -> bool:
        """Best-effort landed check supporting common telemetry field names."""
        if telemetry is None:
            return False

        for name in ("landed", "is_landed", "land_complete"):
            value = getattr(telemetry, name, None)
            if value is True:
                return True

        landed_state = getattr(telemetry, "landed_state", None)
        if landed_state is not None:
            text = str(landed_state).upper()
            if text in {"1", "ON_GROUND", "MAV_LANDED_STATE_ON_GROUND"}:
                return True
        return False

    @staticmethod
    def _fmt(value: Optional[float]) -> str:
        return "N/A" if value is None else f"{value:.2f}"

    @staticmethod
    def _positive_float(value) -> Optional[float]:
        try:
            number = float(value)
            return number if number > 0 else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _get_detection_center(detection) -> Optional[Tuple[int, int]]:
        center = getattr(detection, "center", None)
        if center is not None:
            return int(center[0]), int(center[1])
        corners = getattr(detection, "corners", None)
        if corners is None:
            return None
        try:
            pts = corners.reshape(-1, 2)
            return int(pts[:, 0].mean()), int(pts[:, 1].mean())
        except Exception:
            return None

    @staticmethod
    def _get_marker_side_px(detection) -> Optional[float]:
        corners = getattr(detection, "corners", None)
        if corners is None:
            return None
        try:
            pts = corners.reshape(-1, 2)
            lengths = [
                float((((pts[(i + 1) % 4] - pts[i]) ** 2).sum()) ** 0.5)
                for i in range(4)
            ]
            return sum(lengths) / len(lengths)
        except Exception:
            return None
