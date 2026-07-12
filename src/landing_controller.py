from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil


@dataclass
class LandingCommand:
    active: bool
    detected: bool
    centered: bool
    error_x_px: Optional[float] = None
    error_y_px: Optional[float] = None
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    message: str = "Landing controller inactive"


class VisualLandingController:
    """
    Simple visual-servo landing controller for the capstone project.

    Important safety design:
    - By default it runs in DRY-RUN mode and only prints the commands it would send.
    - It never arms the drone and never performs takeoff.
    - Real MAVLink movement commands are sent only when enable_commands=True.
    - First tests should be performed in SITL or with propellers removed.

    Coordinate note:
    SET_POSITION_TARGET_LOCAL_NED uses vx forward/north, vy right/east, vz down.
    If the camera is mounted differently, invert_x/invert_y can be adjusted.
    """

    def __init__(
        self,
        master,
        enable_commands: bool = False,
        require_guided: bool = True,
        kp_xy: float = 0.0008,
        max_xy_speed_mps: float = 0.25,
        descent_speed_mps: float = 0.20,
        center_tolerance_px: int = 70,
        min_landing_alt_m: float = 0.60,
        command_interval_s: float = 0.20,
        final_land: bool = False,
        invert_x: bool = False,
        invert_y: bool = False,
    ):
        self.master = master
        self.enable_commands = enable_commands
        self.require_guided = require_guided
        self.kp_xy = kp_xy
        self.max_xy_speed_mps = max_xy_speed_mps
        self.descent_speed_mps = descent_speed_mps
        self.center_tolerance_px = center_tolerance_px
        self.min_landing_alt_m = min_landing_alt_m
        self.command_interval_s = command_interval_s
        self.final_land = final_land
        self.invert_x = invert_x
        self.invert_y = invert_y
        self._last_command_time = 0.0
        self._land_command_sent = False

    def update(self, frame_shape, detection, telemetry) -> LandingCommand:
        if detection is None or not getattr(detection, "detected", False):
            return LandingCommand(False, False, False, message="Landing inactive: marker not detected")

        center = self._get_detection_center(detection)
        if center is None:
            return LandingCommand(False, True, False, message="Landing inactive: marker center not available")

        h, w = int(frame_shape[0]), int(frame_shape[1])
        center_x, center_y = center
        err_x = float(center_x - (w / 2.0))
        err_y = float(center_y - (h / 2.0))

        centered = abs(err_x) <= self.center_tolerance_px and abs(err_y) <= self.center_tolerance_px

        # Camera image error to horizontal velocity correction.
        # If marker appears low in the image, move forward/back depending on camera mounting.
        vx = self._clamp(-err_y * self.kp_xy, -self.max_xy_speed_mps, self.max_xy_speed_mps)
        vy = self._clamp(err_x * self.kp_xy, -self.max_xy_speed_mps, self.max_xy_speed_mps)
        if self.invert_x:
            vx = -vx
        if self.invert_y:
            vy = -vy

        # Descend only when approximately centered above marker.
        vz = self.descent_speed_mps if centered else 0.0

        rel_alt = getattr(telemetry, "relative_alt_m", None)
        if rel_alt is not None and rel_alt <= self.min_landing_alt_m:
            vx = 0.0
            vy = 0.0
            vz = 0.0
            if centered and self.final_land and not self._land_command_sent:
                self._send_land_command(telemetry)
                self._land_command_sent = True
                return LandingCommand(True, True, centered, err_x, err_y, vx, vy, vz, "Final LAND command requested")
            return LandingCommand(True, True, centered, err_x, err_y, vx, vy, vz, f"Minimum landing altitude reached ({rel_alt:.2f} m). Holding position.")

        now = time.time()
        if now - self._last_command_time >= self.command_interval_s:
            self._last_command_time = now
            self._send_velocity_command(vx, vy, vz, telemetry)

        mode = "REAL COMMAND" if self.enable_commands else "DRY RUN"
        msg = f"{mode}: vx={vx:.2f}, vy={vy:.2f}, vz_down={vz:.2f}, centered={centered}, err=({err_x:.0f},{err_y:.0f}) px"
        return LandingCommand(True, True, centered, err_x, err_y, vx, vy, vz, msg)

    def stop(self) -> None:
        if self.enable_commands and self.master is not None:
            self._send_velocity_command(0.0, 0.0, 0.0, None, force=True)

    def _send_velocity_command(self, vx: float, vy: float, vz: float, telemetry, force: bool = False) -> None:
        if not self.enable_commands:
            return
        if self.master is None:
            return

        if telemetry is not None:
            if getattr(telemetry, "armed", None) is not True:
                return
            if self.require_guided and str(getattr(telemetry, "mode", "")).upper() != "GUIDED":
                return

        # Ignore position, acceleration, yaw and yaw-rate. Use velocity only.
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

    def _send_land_command(self, telemetry) -> None:
        if not self.enable_commands:
            return
        if self.master is None:
            return
        if telemetry is not None and self.require_guided and str(getattr(telemetry, "mode", "")).upper() != "GUIDED":
            return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

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
