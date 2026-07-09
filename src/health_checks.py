from dataclasses import dataclass
from typing import List, Optional
import config


@dataclass
class HealthResult:
    status: str
    messages: List[str]


def worst_status(current: str, new: str) -> str:
    priority = {"OK": 0, "WARNING": 1, "CRITICAL": 2}
    return new if priority[new] > priority[current] else current


def check_drone_health(tel, vibration_rms: Optional[float], cpu_temp: Optional[float],
                       cpu_usage: float, ram_usage: float) -> HealthResult:
    status = "OK"
    messages = []

    # MAVLink / flight controller
    if tel.heartbeat_count <= 0:
        status = worst_status(status, "CRITICAL")
        messages.append("No Pixhawk heartbeat received")
    if tel.mode is None:
        status = worst_status(status, "WARNING")
        messages.append("Flight mode is missing")

    # Battery
    if tel.battery_voltage_v is None:
        status = worst_status(status, "WARNING")
        messages.append("Battery telemetry missing")
    elif tel.battery_voltage_v <= config.BATTERY_CRITICAL_VOLTAGE:
        status = worst_status(status, "CRITICAL")
        messages.append(f"Battery critical: {tel.battery_voltage_v:.2f} V")
    elif tel.battery_voltage_v <= config.BATTERY_MIN_VOLTAGE:
        status = worst_status(status, "WARNING")
        messages.append(f"Battery low: {tel.battery_voltage_v:.2f} V")

    # RC
    if tel.rc_channel_count is None:
        status = worst_status(status, "WARNING")
        messages.append("RC link telemetry missing")
    elif tel.rc_channel_count < 4:
        status = worst_status(status, "CRITICAL")
        messages.append(f"RC channel count too low: {tel.rc_channel_count}")
    elif tel.rc_rssi is not None and tel.rc_rssi != 255 and tel.rc_rssi < 40:
        status = worst_status(status, "WARNING")
        messages.append(f"RC RSSI is low: {tel.rc_rssi}")

    # GPS
    if tel.gps_fix_type is None:
        status = worst_status(status, "WARNING")
        messages.append("GPS telemetry missing")
    else:
        if tel.gps_fix_type < config.MIN_GPS_FIX_TYPE:
            status = worst_status(status, "WARNING")
            messages.append(f"GPS fix weak: fix_type={tel.gps_fix_type}")
        if tel.gps_satellites is not None and tel.gps_satellites < config.MIN_GPS_SATELLITES:
            status = worst_status(status, "WARNING")
            messages.append(f"Low GPS satellites: {tel.gps_satellites}")
        if tel.gps_hdop is not None:
            if tel.gps_hdop >= config.GPS_HDOP_CRITICAL:
                status = worst_status(status, "CRITICAL")
                messages.append(f"GPS HDOP critical: {tel.gps_hdop:.2f}")
            elif tel.gps_hdop >= config.GPS_HDOP_WARNING:
                status = worst_status(status, "WARNING")
                messages.append(f"GPS HDOP warning: {tel.gps_hdop:.2f}")

    # EKF
    ekf_values = [
        tel.ekf_velocity_variance,
        tel.ekf_pos_horiz_variance,
        tel.ekf_pos_vert_variance,
        tel.ekf_compass_variance,
        tel.ekf_terrain_alt_variance,
    ]
    if all(v is None for v in ekf_values):
        status = worst_status(status, "WARNING")
        messages.append("EKF telemetry missing")
    elif any(v is not None and v > 1.0 for v in ekf_values):
        status = worst_status(status, "WARNING")
        messages.append("EKF variance is high")

    # RAW_IMU vibration
    if vibration_rms is None:
        status = worst_status(status, "WARNING")
        messages.append("Not enough IMU samples for vibration RMS")
    elif vibration_rms >= config.VIBRATION_CRITICAL_RMS:
        status = worst_status(status, "CRITICAL")
        messages.append(f"Critical RAW_IMU vibration RMS: {vibration_rms:.2f} m/s^2")
    elif vibration_rms >= config.VIBRATION_WARNING_RMS:
        status = worst_status(status, "WARNING")
        messages.append(f"High RAW_IMU vibration RMS: {vibration_rms:.2f} m/s^2")

    # ArduPilot VIBE
    vibe_values = [tel.vibe_x, tel.vibe_y, tel.vibe_z]
    if all(v is not None for v in vibe_values):
        max_vibe = max(vibe_values)
        if max_vibe >= config.VIBE_CRITICAL:
            status = worst_status(status, "CRITICAL")
            messages.append(f"ArduPilot VIBE critical: max={max_vibe:.1f}")
        elif max_vibe >= config.VIBE_WARNING:
            status = worst_status(status, "WARNING")
            messages.append(f"ArduPilot VIBE high: max={max_vibe:.1f}")

    # IMU clipping
    clipping_values = [tel.clipping_0, tel.clipping_1, tel.clipping_2]
    if any(c is not None and c >= config.CLIPPING_WARNING for c in clipping_values):
        status = worst_status(status, "WARNING")
        messages.append(f"IMU clipping detected: {clipping_values}")

    # PWM / motors
    active_pwm = [
        pwm for pwm in tel.servo_outputs[:4]
        if pwm is not None and pwm > 900
    ]
    if len(active_pwm) == 0:
        # Normal when disarmed; warning only for missing data, not for PWM=0.
        pass

    # Autopilot status text
    if tel.statustext_errors > 0:
        status = worst_status(status, "WARNING")
        messages.append(f"Autopilot status errors detected: {tel.statustext_errors}")
    elif tel.statustext_warnings > 0:
        status = worst_status(status, "WARNING")
        messages.append(f"Autopilot warnings detected: {tel.statustext_warnings}")

    # Raspberry Pi
    if cpu_temp is not None:
        if cpu_temp >= config.CPU_TEMP_CRITICAL_C:
            status = worst_status(status, "CRITICAL")
            messages.append(f"CPU temperature critical: {cpu_temp:.1f} C")
        elif cpu_temp >= config.CPU_TEMP_WARNING_C:
            status = worst_status(status, "WARNING")
            messages.append(f"CPU temperature high: {cpu_temp:.1f} C")

    if cpu_usage >= config.CPU_USAGE_WARNING_PERCENT:
        status = worst_status(status, "WARNING")
        messages.append(f"High CPU usage: {cpu_usage:.1f}%")

    if ram_usage >= config.RAM_USAGE_WARNING_PERCENT:
        status = worst_status(status, "WARNING")
        messages.append(f"High RAM usage: {ram_usage:.1f}%")

    if not messages:
        messages.append("All monitored systems look normal")

    return HealthResult(status=status, messages=messages)
