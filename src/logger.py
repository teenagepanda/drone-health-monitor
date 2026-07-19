import csv
from pathlib import Path
from datetime import datetime


class CsvLogger:
    def __init__(self, log_dir: str = "logs"):
        Path(log_dir).mkdir(exist_ok=True)
        filename = datetime.now().strftime("drone_health_%Y%m%d_%H%M%S.csv")
        self.path = Path(log_dir) / filename
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "timestamp",
            "heartbeat_count", "mavlink_message_count",
            "seconds_since_heartbeat", "seconds_since_message",
            "armed", "mode", "system_status",
            "battery_voltage_v", "battery_current_a", "battery_remaining_percent",
            "gps_fix_type", "gps_satellites", "gps_hdop",
            "gps_global_lat", "gps_global_lon", "gps_global_alt_m", "relative_alt_m",
            "home_position_received", "home_lat", "home_lon", "home_alt_m",
            "acc_x", "acc_y", "acc_z", "vibration_rms",
            "vibe_x", "vibe_y", "vibe_z",
            "clipping_0", "clipping_1", "clipping_2",
            "rc_rssi", "rc_channel_count",
            "rc_chan1_raw", "rc_chan2_raw", "rc_chan3_raw", "rc_chan4_raw",
            "rc_chan5_raw", "rc_chan6_raw", "rc_chan7_raw", "rc_chan8_raw",
            "servo1", "servo2", "servo3", "servo4",
            "servo5", "servo6", "servo7", "servo8",
            "ekf_flags", "ekf_velocity_variance", "ekf_pos_horiz_variance",
            "ekf_pos_vert_variance", "ekf_compass_variance", "ekf_terrain_alt_variance",
            "gyro_present", "gyro_healthy",
            "accel_present", "accel_healthy",
            "compass_present", "compass_healthy",
            "barometer_present", "barometer_healthy",
            "gps_present", "gps_healthy",
            "fs_thr_enable", "fs_batt_enable", "fs_gcs_enable", "fs_ekf_action",
            "batt_fs_low_act", "batt_fs_crt_act", "batt_low_volt", "batt_crt_volt",
            "rtl_alt",
            "statustext_errors", "statustext_warnings", "last_statustext", "prearm_messages",
            "cpu_temp_c", "cpu_usage_percent", "ram_usage_percent",
            "health_status", "health_messages"
        ])

    def write(self, tel, vibration_rms, cpu_temp, cpu_usage, ram_usage, health):
        seconds_since_heartbeat = None
        seconds_since_message = None

        if tel.last_heartbeat_time is not None:
            seconds_since_heartbeat = tel.timestamp - tel.last_heartbeat_time
        if tel.last_message_time is not None:
            seconds_since_message = tel.timestamp - tel.last_message_time

        self.writer.writerow([
            tel.timestamp,
            tel.heartbeat_count, tel.mavlink_message_count,
            seconds_since_heartbeat, seconds_since_message,
            tel.armed, tel.mode, tel.system_status,
            tel.battery_voltage_v, tel.battery_current_a, tel.battery_remaining_percent,
            tel.gps_fix_type, tel.gps_satellites, tel.gps_hdop,
            tel.gps_global_lat, tel.gps_global_lon, tel.gps_global_alt_m, tel.relative_alt_m,
            tel.home_position_received, tel.home_lat, tel.home_lon, tel.home_alt_m,
            tel.acc_x, tel.acc_y, tel.acc_z, vibration_rms,
            tel.vibe_x, tel.vibe_y, tel.vibe_z,
            tel.clipping_0, tel.clipping_1, tel.clipping_2,
            tel.rc_rssi, tel.rc_channel_count,
            tel.rc_chan1_raw, tel.rc_chan2_raw, tel.rc_chan3_raw, tel.rc_chan4_raw,
            tel.rc_chan5_raw, tel.rc_chan6_raw, tel.rc_chan7_raw, tel.rc_chan8_raw,
            *tel.servo_outputs[:8],
            tel.ekf_flags, tel.ekf_velocity_variance, tel.ekf_pos_horiz_variance,
            tel.ekf_pos_vert_variance, tel.ekf_compass_variance, tel.ekf_terrain_alt_variance,
            tel.gyro_present, tel.gyro_healthy,
            tel.accel_present, tel.accel_healthy,
            tel.compass_present, tel.compass_healthy,
            tel.barometer_present, tel.barometer_healthy,
            tel.gps_present, tel.gps_healthy,
            tel.param_fs_thr_enable, tel.param_fs_batt_enable, tel.param_fs_gcs_enable,
            tel.param_fs_ekf_action,
            tel.param_batt_fs_low_act, tel.param_batt_fs_crt_act,
            tel.param_batt_low_volt, tel.param_batt_crt_volt,
            tel.param_rtl_alt,
            tel.statustext_errors, tel.statustext_warnings, tel.last_statustext,
            " | ".join(tel.prearm_messages),
            cpu_temp, cpu_usage, ram_usage,
            health.status, " | ".join(health.messages)
        ])
        self.file.flush()

    def close(self):
        self.file.close()
