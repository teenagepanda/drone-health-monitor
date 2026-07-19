from dataclasses import dataclass, field
from typing import Optional, List
from pymavlink import mavutil
import time


def bit_is_set(value: Optional[int], bit: int) -> Optional[bool]:
    if value is None:
        return None
    return bool(value & bit)


@dataclass
class DroneTelemetry:
    timestamp: float

    heartbeat_count: int = 0
    last_heartbeat_time: Optional[float] = None
    mavlink_message_count: int = 0
    last_message_time: Optional[float] = None

    armed: Optional[bool] = None
    mode: Optional[str] = None
    system_status: Optional[int] = None

    battery_voltage_v: Optional[float] = None
    battery_current_a: Optional[float] = None
    battery_remaining_percent: Optional[int] = None

    gps_fix_type: Optional[int] = None
    gps_satellites: Optional[int] = None
    gps_hdop: Optional[float] = None

    acc_x: Optional[float] = None
    acc_y: Optional[float] = None
    acc_z: Optional[float] = None

    vibe_x: Optional[float] = None
    vibe_y: Optional[float] = None
    vibe_z: Optional[float] = None
    clipping_0: Optional[int] = None
    clipping_1: Optional[int] = None
    clipping_2: Optional[int] = None

    rc_rssi: Optional[int] = None
    rc_channel_count: Optional[int] = None
    rc_chan1_raw: Optional[int] = None
    rc_chan2_raw: Optional[int] = None
    rc_chan3_raw: Optional[int] = None
    rc_chan4_raw: Optional[int] = None
    rc_chan5_raw: Optional[int] = None
    rc_chan6_raw: Optional[int] = None
    rc_chan7_raw: Optional[int] = None
    rc_chan8_raw: Optional[int] = None

    servo_outputs: List[Optional[int]] = field(default_factory=lambda: [None] * 8)

    ekf_flags: Optional[int] = None
    ekf_velocity_variance: Optional[float] = None
    ekf_pos_horiz_variance: Optional[float] = None
    ekf_pos_vert_variance: Optional[float] = None
    ekf_compass_variance: Optional[float] = None
    ekf_terrain_alt_variance: Optional[float] = None

    onboard_control_sensors_present: Optional[int] = None
    onboard_control_sensors_enabled: Optional[int] = None
    onboard_control_sensors_health: Optional[int] = None

    gyro_present: Optional[bool] = None
    gyro_healthy: Optional[bool] = None
    accel_present: Optional[bool] = None
    accel_healthy: Optional[bool] = None
    compass_present: Optional[bool] = None
    compass_healthy: Optional[bool] = None
    barometer_present: Optional[bool] = None
    barometer_healthy: Optional[bool] = None
    gps_present: Optional[bool] = None
    gps_healthy: Optional[bool] = None

    home_position_received: bool = False
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None
    home_alt_m: Optional[float] = None

    gps_global_lat: Optional[float] = None
    gps_global_lon: Optional[float] = None
    gps_global_alt_m: Optional[float] = None
    relative_alt_m: Optional[float] = None

    param_fs_thr_enable: Optional[float] = None
    param_fs_batt_enable: Optional[float] = None
    param_fs_gcs_enable: Optional[float] = None
    param_fs_ekf_action: Optional[float] = None
    param_batt_fs_low_act: Optional[float] = None
    param_batt_fs_crt_act: Optional[float] = None
    param_batt_low_volt: Optional[float] = None
    param_batt_crt_volt: Optional[float] = None
    param_rtl_alt: Optional[float] = None

    last_statustext: Optional[str] = None
    statustext_errors: int = 0
    statustext_warnings: int = 0
    prearm_messages: List[str] = field(default_factory=list)


class MavlinkReader:
    def __init__(self, connection: str, baud: int = 115200):
        self.connection = connection
        self.baud = baud
        self.master = None
        self.telemetry = DroneTelemetry(timestamp=time.time())

    def connect(self, timeout: int = 30) -> None:
        print(f"Connecting to Pixhawk on {self.connection} at {self.baud} baud...")
        self.master = mavutil.mavlink_connection(self.connection, baud=self.baud)
        self.master.wait_heartbeat(timeout=timeout)
        print("Heartbeat received. MAVLink connection is active.")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )

        for msg_name, hz in [
            ("HEARTBEAT", 1), ("SYS_STATUS", 2), ("GPS_RAW_INT", 2),
            ("GLOBAL_POSITION_INT", 2), ("RAW_IMU", 10), ("VIBRATION", 5),
            ("SERVO_OUTPUT_RAW", 5), ("RC_CHANNELS", 5),
            ("EKF_STATUS_REPORT", 2), ("STATUSTEXT", 2), ("HOME_POSITION", 1)
        ]:
            self._request_message_interval(msg_name, hz)

        self._request_param("FS_THR_ENABLE")
        self._request_param("FS_BATT_ENABLE")
        self._request_param("FS_GCS_ENABLE")
        self._request_param("FS_EKF_ACTION")
        self._request_param("BATT_FS_LOW_ACT")
        self._request_param("BATT_FS_CRT_ACT")
        self._request_param("BATT_LOW_VOLT")
        self._request_param("BATT_CRT_VOLT")
        self._request_param("RTL_ALT")

    def _request_message_interval(self, message_name: str, hz: float) -> None:
        try:
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}")
            interval_us = int(1_000_000 / hz)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
        except Exception as error:
            print(f"Warning: could not request {message_name}: {error}")

    def _request_param(self, param_name: str) -> None:
        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode("ascii"),
                -1
            )
        except Exception as error:
            print(f"Warning: could not request parameter {param_name}: {error}")

    def read_messages(self) -> DroneTelemetry:
        if self.master is None:
            raise RuntimeError("MAVLink is not connected. Call connect() first.")

        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            now = time.time()

            self.telemetry.timestamp = now
            self.telemetry.mavlink_message_count += 1
            self.telemetry.last_message_time = now

            if msg_type == "HEARTBEAT":
                self.telemetry.heartbeat_count += 1
                self.telemetry.last_heartbeat_time = now
                self.telemetry.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.telemetry.system_status = int(msg.system_status)
                try:
                    self.telemetry.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    self.telemetry.mode = "UNKNOWN"

            elif msg_type == "SYS_STATUS":
                if msg.voltage_battery != 65535:
                    self.telemetry.battery_voltage_v = msg.voltage_battery / 1000.0
                if msg.current_battery != -1:
                    self.telemetry.battery_current_a = msg.current_battery / 100.0
                if msg.battery_remaining != -1:
                    self.telemetry.battery_remaining_percent = int(msg.battery_remaining)

                self.telemetry.onboard_control_sensors_present = int(msg.onboard_control_sensors_present)
                self.telemetry.onboard_control_sensors_enabled = int(msg.onboard_control_sensors_enabled)
                self.telemetry.onboard_control_sensors_health = int(msg.onboard_control_sensors_health)
                self._parse_sensor_bits()

            elif msg_type == "GPS_RAW_INT":
                self.telemetry.gps_fix_type = int(msg.fix_type)
                self.telemetry.gps_satellites = int(msg.satellites_visible)
                if getattr(msg, "eph", 65535) != 65535:
                    self.telemetry.gps_hdop = msg.eph / 100.0

            elif msg_type == "GLOBAL_POSITION_INT":
                self.telemetry.gps_global_lat = msg.lat / 1e7
                self.telemetry.gps_global_lon = msg.lon / 1e7
                self.telemetry.gps_global_alt_m = msg.alt / 1000.0
                self.telemetry.relative_alt_m = msg.relative_alt / 1000.0

            elif msg_type == "HOME_POSITION":
                self.telemetry.home_position_received = True
                self.telemetry.home_lat = msg.latitude / 1e7
                self.telemetry.home_lon = msg.longitude / 1e7
                self.telemetry.home_alt_m = msg.altitude / 1000.0

            elif msg_type == "RAW_IMU":
                self.telemetry.acc_x = (msg.xacc / 1000.0) * 9.80665
                self.telemetry.acc_y = (msg.yacc / 1000.0) * 9.80665
                self.telemetry.acc_z = (msg.zacc / 1000.0) * 9.80665

            elif msg_type == "VIBRATION":
                self.telemetry.vibe_x = float(msg.vibration_x)
                self.telemetry.vibe_y = float(msg.vibration_y)
                self.telemetry.vibe_z = float(msg.vibration_z)
                self.telemetry.clipping_0 = int(msg.clipping_0)
                self.telemetry.clipping_1 = int(msg.clipping_1)
                self.telemetry.clipping_2 = int(msg.clipping_2)

            elif msg_type == "SERVO_OUTPUT_RAW":
                self.telemetry.servo_outputs = [
                    getattr(msg, f"servo{i}_raw", None) for i in range(1, 9)
                ]

            elif msg_type == "RC_CHANNELS":
                self.telemetry.rc_channel_count = int(getattr(msg, "chancount", 0))
                self.telemetry.rc_rssi = int(getattr(msg, "rssi", 255))
                for i in range(1, 9):
                    setattr(self.telemetry, f"rc_chan{i}_raw", getattr(msg, f"chan{i}_raw", None))

            elif msg_type == "EKF_STATUS_REPORT":
                self.telemetry.ekf_flags = int(getattr(msg, "flags", 0))
                self.telemetry.ekf_velocity_variance = float(getattr(msg, "velocity_variance", 0.0))
                self.telemetry.ekf_pos_horiz_variance = float(getattr(msg, "pos_horiz_variance", 0.0))
                self.telemetry.ekf_pos_vert_variance = float(getattr(msg, "pos_vert_variance", 0.0))
                self.telemetry.ekf_compass_variance = float(getattr(msg, "compass_variance", 0.0))
                self.telemetry.ekf_terrain_alt_variance = float(getattr(msg, "terrain_alt_variance", 0.0))

            elif msg_type == "PARAM_VALUE":
                self._parse_param_value(msg)

            elif msg_type == "STATUSTEXT":
                text = str(getattr(msg, "text", "")).strip()
                severity = int(getattr(msg, "severity", 6))
                self.telemetry.last_statustext = text

                if "PreArm:" in text and text not in self.telemetry.prearm_messages:
                    self.telemetry.prearm_messages.append(text)

                if severity <= 3:
                    self.telemetry.statustext_errors += 1
                elif severity == 4:
                    self.telemetry.statustext_warnings += 1

        return self.telemetry

    def _parse_param_value(self, msg) -> None:
        try:
            raw_id = msg.param_id
            if isinstance(raw_id, bytes):
                param_id = raw_id.decode("ascii", errors="ignore").strip("\x00")
            else:
                param_id = str(raw_id).strip("\x00")
            value = float(msg.param_value)

            mapping = {
                "FS_THR_ENABLE": "param_fs_thr_enable",
                "FS_BATT_ENABLE": "param_fs_batt_enable",
                "FS_GCS_ENABLE": "param_fs_gcs_enable",
                "FS_EKF_ACTION": "param_fs_ekf_action",
                "BATT_FS_LOW_ACT": "param_batt_fs_low_act",
                "BATT_FS_CRT_ACT": "param_batt_fs_crt_act",
                "BATT_LOW_VOLT": "param_batt_low_volt",
                "BATT_CRT_VOLT": "param_batt_crt_volt",
                "RTL_ALT": "param_rtl_alt",
            }
            attr = mapping.get(param_id)
            if attr:
                setattr(self.telemetry, attr, value)
        except Exception:
            pass

    def _parse_sensor_bits(self) -> None:
        # Common MAV_SYS_STATUS_SENSOR bits:
        # 1 gyro, 2 accelerometer, 4 compass, 8 absolute pressure/barometer, 32 GPS
        present = self.telemetry.onboard_control_sensors_present
        health = self.telemetry.onboard_control_sensors_health

        self.telemetry.gyro_present = bit_is_set(present, 1)
        self.telemetry.gyro_healthy = bit_is_set(health, 1)

        self.telemetry.accel_present = bit_is_set(present, 2)
        self.telemetry.accel_healthy = bit_is_set(health, 2)

        self.telemetry.compass_present = bit_is_set(present, 4)
        self.telemetry.compass_healthy = bit_is_set(health, 4)

        self.telemetry.barometer_present = bit_is_set(present, 8)
        self.telemetry.barometer_healthy = bit_is_set(health, 8)

        self.telemetry.gps_present = bit_is_set(present, 32)
        self.telemetry.gps_healthy = bit_is_set(health, 32)
