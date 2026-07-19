from pathlib import Path
from datetime import datetime
import pandas as pd


def numeric(df, col):
    if col not in df.columns:
        return pd.Series(dtype=float)
    return pd.to_numeric(df[col], errors="coerce")


def avg(df, col):
    values = numeric(df, col).dropna()
    return None if values.empty else float(values.mean())


def min_val(df, col):
    values = numeric(df, col).dropna()
    return None if values.empty else float(values.min())


def max_val(df, col):
    values = numeric(df, col).dropna()
    return None if values.empty else float(values.max())


def latest_text(df, col):
    if col not in df.columns:
        return None
    values = df[col].dropna()
    if values.empty:
        return None
    return str(values.iloc[-1])


def bool_any_true(df, col):
    if col not in df.columns:
        return False
    values = df[col].dropna().astype(str).str.lower()
    return any(v == "true" for v in values)


def status_line(name, status, message):
    return f"<tr><td><b>{name}</b></td><td>{status}</td><td>{message}</td></tr>"


def add_result(rows, plain_lines, problems, name, status, message, is_problem=False, critical=True):
    rows.append(status_line(name, status, message))
    plain_lines.append(f"{name}: {status} — {message}")
    if is_problem:
        prefix = "חובה לתקן" if critical else "מומלץ לבדוק"
        problems.append(f"{prefix} - {name}: {message}")


def friendly_status_rows(df):
    rows, plain_lines, problems = [], [], []

    msg_count = max_val(df, "mavlink_message_count")
    hb_count = max_val(df, "heartbeat_count")
    mode = latest_text(df, "mode")
    armed = latest_text(df, "armed")

    if msg_count is None or msg_count <= 0 or hb_count is None or hb_count <= 0:
        add_result(rows, plain_lines, problems, "תקשורת Pixhawk ↔ Raspberry Pi", "❌ לא תקין",
                   "לא התקבלה תקשורת MAVLink תקינה מהבקר.", True)
    else:
        add_result(rows, plain_lines, problems, "תקשורת Pixhawk ↔ Raspberry Pi", "✅ תקין",
                   f"התקבלו הודעות MAVLink ו־Heartbeat. מצב טיסה: {mode}, Armed: {armed}.")

    if mode:
        if mode in ["GUIDED", "LOITER", "AUTO", "RTL"]:
            mode_note = "מתאים לבדיקות אוטונומיות/ניווט."
        elif mode in ["STABILIZE", "ALT_HOLD"]:
            mode_note = "מתאים לבדיקה ידנית, אך לא אידיאלי למשימת נחיתה אוטונומית."
        else:
            mode_note = "מצב טיסה מזוהה."
        add_result(rows, plain_lines, problems, "מצב טיסה", "✅ מזוהה", f"{mode} — {mode_note}",
                   is_problem=(mode in ["STABILIZE", "ALT_HOLD"]), critical=False)
    else:
        add_result(rows, plain_lines, problems, "מצב טיסה", "⚠️ חסר מידע", "לא התקבל מצב טיסה.", True)

    rc_channels = avg(df, "rc_channel_count")
    rc_rssi_raw = avg(df, "rc_rssi")
    ch1, ch2, ch3, ch4 = [avg(df, f"rc_chan{i}_raw") for i in range(1, 5)]

    pwm_values = [ch1, ch2, ch3, ch4]
    pwm_text = ", ".join(
        "N/A" if value is None else str(int(round(value)))
        for value in pwm_values
    )

    # MAVLink RC_CHANNELS.rssi is 0..254. The value 255 means unavailable.
    rssi_available = (
        rc_rssi_raw is not None
        and 0 <= rc_rssi_raw <= 254
    )
    rssi_percent = (
        max(0.0, min(100.0, rc_rssi_raw / 254.0 * 100.0))
        if rssi_available
        else None
    )

    if rc_channels is None:
        add_result(rows, plain_lines, problems, "שלט RC / ELRS", "⚠️ חסר מידע",
                   "לא התקבלו נתוני RC_CHANNELS.", True)
    elif rc_channels >= 4:
        base_message = (
            f"זוהו {rc_channels:.0f} ערוצים. "
            f"ערוצי RC 1–4 (PWM): {pwm_text}."
        )

        if rssi_percent is None:
            add_result(
                rows, plain_lines, problems,
                "שלט RC / ELRS", "✅ תקין",
                base_message + " RSSI לא זמין בהודעת MAVLink, אך ערוצי RC מתקבלים."
            )
        elif rssi_percent >= 60:
            add_result(
                rows, plain_lines, problems,
                "שלט RC / ELRS", "✅ תקין",
                base_message
                + f" RSSI ממוצע: {rssi_percent:.0f}% "
                  f"(ערך MAVLink גולמי {rc_rssi_raw:.0f}/254)."
            )
        elif rssi_percent >= 40:
            add_result(
                rows, plain_lines, problems,
                "שלט RC / ELRS", "⚠️ גבולי",
                base_message + f" RSSI ממוצע נמוך: {rssi_percent:.0f}%.",
                True, False
            )
        else:
            add_result(
                rows, plain_lines, problems,
                "שלט RC / ELRS", "❌ לא תקין",
                base_message + f" RSSI נמוך מאוד: {rssi_percent:.0f}%.",
                True
            )
    else:
        add_result(rows, plain_lines, problems, "שלט RC / ELRS", "❌ לא תקין",
                   f"זוהו רק {rc_channels:.0f} ערוצים. נדרשים לפחות 4 ערוצים.", True)

    fs_thr = avg(df, "fs_thr_enable")
    fs_batt_legacy = avg(df, "fs_batt_enable")
    fs_gcs = avg(df, "fs_gcs_enable")
    fs_ekf = avg(df, "fs_ekf_action")
    rtl_alt = avg(df, "rtl_alt")

    # Newer ArduPilot battery-failsafe parameters. These columns are used
    # automatically once the MAVLink reader/logger include them in the CSV.
    batt_low_act = avg(df, "batt_fs_low_act")
    batt_crt_act = avg(df, "batt_fs_crt_act")
    batt_low_volt = avg(df, "batt_low_volt")
    batt_crt_volt = avg(df, "batt_crt_volt")

    fs_missing = all(
        value is None
        for value in [
            fs_thr, fs_batt_legacy, fs_gcs, fs_ekf,
            batt_low_act, batt_crt_act, batt_low_volt, batt_crt_volt,
        ]
    )

    if fs_missing:
        add_result(
            rows, plain_lines, problems,
            "Failsafe Configuration", "⚠️ חסר מידע",
            "לא התקבלו פרמטרי Failsafe מהבקר.", True, False
        )
    else:
        notes = []
        critical_problem = False
        warning_problem = False

        if fs_thr is None:
            notes.append("RC Failsafe: לא התקבל בפרטי הלוג")
            warning_problem = True
        elif fs_thr == 0:
            notes.append("RC Failsafe: כבוי")
            critical_problem = True
        else:
            notes.append(f"RC Failsafe: מוגדר (FS_THR_ENABLE={int(fs_thr)})")

        battery_params_available = any(
            value is not None
            for value in [batt_low_act, batt_crt_act, batt_low_volt, batt_crt_volt]
        )

        if battery_params_available:
            low_action_text = (
                "לא ידוע" if batt_low_act is None
                else ("אזהרה בלבד" if batt_low_act == 0 else f"פעולה {int(batt_low_act)}")
            )
            crt_action_text = (
                "לא ידוע" if batt_crt_act is None
                else ("אזהרה בלבד" if batt_crt_act == 0 else f"פעולה {int(batt_crt_act)}")
            )
            notes.append(
                "Battery Failsafe: "
                f"LOW={low_action_text}, CRITICAL={crt_action_text}"
            )

            if batt_low_volt is not None:
                notes.append(f"BATT_LOW_VOLT={batt_low_volt:.1f}V")
            if batt_crt_volt is not None:
                notes.append(f"BATT_CRT_VOLT={batt_crt_volt:.1f}V")

            low_enabled = (
                batt_low_volt is not None
                and batt_low_volt > 0
                and batt_low_act is not None
                and batt_low_act > 0
            )
            critical_enabled = (
                batt_crt_volt is not None
                and batt_crt_volt > 0
                and batt_crt_act is not None
                and batt_crt_act > 0
            )

            if not low_enabled:
                warning_problem = True
            if batt_crt_volt is not None and batt_crt_volt > 0 and not critical_enabled:
                warning_problem = True
        elif fs_batt_legacy is not None:
            if fs_batt_legacy == 0:
                notes.append("Battery Failsafe legacy: כבוי")
                warning_problem = True
            else:
                notes.append(
                    f"Battery Failsafe legacy: מוגדר "
                    f"(FS_BATT_ENABLE={int(fs_batt_legacy)})"
                )
        else:
            notes.append(
                "Battery Failsafe: פרמטרי BATT_* אינם כלולים עדיין בלוג"
            )

        if fs_gcs is not None:
            notes.append(f"GCS Failsafe setting={int(fs_gcs)}")
        if fs_ekf is not None:
            notes.append(f"EKF Failsafe Action={int(fs_ekf)}")
        if rtl_alt is not None:
            notes.append(f"RTL_ALT={int(rtl_alt)}cm")

        if critical_problem:
            fs_status = "❌ לא תקין"
            is_problem = True
            critical = True
        elif warning_problem:
            fs_status = "⚠️ דורש בדיקה"
            is_problem = True
            critical = False
        else:
            fs_status = "✅ תקין"
            is_problem = False
            critical = False

        add_result(
            rows,
            plain_lines,
            problems,
            "Failsafe Configuration",
            fs_status,
            "; ".join(notes)
            + ". אלו פרמטרי תצורה ואינם מעידים ש-Failsafe פעיל כרגע.",
            is_problem,
            critical,
        )

    home_ok = bool_any_true(df, "home_position_received")
    home_lat = avg(df, "home_lat")
    home_lon = avg(df, "home_lon")
    if home_ok:
        add_result(rows, plain_lines, problems, "Home Position", "✅ תקין",
                   f"נקבע Home Position. Lat={home_lat:.6f}, Lon={home_lon:.6f}.")
    else:
        add_result(rows, plain_lines, problems, "Home Position", "❌ לא תקין",
                   "לא התקבל HOME_POSITION. עבור RTL חובה לוודא Home Position לפני טיסה.", True)

    prearm = latest_text(df, "prearm_messages")
    if prearm:
        items = [x.strip() for x in prearm.split("|") if x.strip()]
        unique = []
        for x in items:
            if x not in unique:
                unique.append(x)
        add_result(rows, plain_lines, problems, "PreArm Checks", "❌ לא תקין",
                   " | ".join(unique), True)
    else:
        add_result(rows, plain_lines, problems, "PreArm Checks", "✅ תקין",
                   "לא התקבלו הודעות PreArm שמונעות טיסה במהלך הבדיקה.")

    for heb_name, col_present, col_health in [
        ("Gyroscope", "gyro_present", "gyro_healthy"),
        ("Accelerometer", "accel_present", "accel_healthy"),
        ("Compass Sensor", "compass_present", "compass_healthy"),
        ("Barometer", "barometer_present", "barometer_healthy"),
        ("GPS Sensor", "gps_present", "gps_healthy"),
    ]:
        present = bool_any_true(df, col_present)
        healthy = bool_any_true(df, col_health)
        if present and healthy:
            add_result(rows, plain_lines, problems, heb_name, "✅ תקין", "החיישן מזוהה ובריא לפי SYS_STATUS.")
        elif present and not healthy:
            add_result(rows, plain_lines, problems, heb_name, "❌ לא תקין", "החיישן מזוהה אך לא בריא לפי SYS_STATUS.", True)
        else:
            add_result(rows, plain_lines, problems, heb_name, "⚠️ חסר מידע", "החיישן לא זוהה ב־SYS_STATUS.", True, False)

    battery_v = avg(df, "battery_voltage_v")
    battery_min = min_val(df, "battery_voltage_v")
    battery_percent = avg(df, "battery_remaining_percent")
    if battery_v is None:
        add_result(rows, plain_lines, problems, "סוללה", "⚠️ חסר מידע", "לא התקבלו נתוני סוללה.", True)
    elif battery_v >= 21.0:
        add_result(rows, plain_lines, problems, "סוללה", "✅ תקינה",
                   f"מתח ממוצע {battery_v:.2f}V, מתח מינימלי {battery_min:.2f}V, אחוז ממוצע {battery_percent:.0f}%.")
    elif battery_v >= 20.0:
        add_result(rows, plain_lines, problems, "סוללה", "⚠️ אזהרה",
                   f"מתח ממוצע {battery_v:.2f}V. מומלץ להטעין לפני טיסה.", True, False)
    else:
        add_result(rows, plain_lines, problems, "סוללה", "❌ לא תקינה",
                   f"מתח ממוצע {battery_v:.2f}V נמוך מדי.", True)

    gps_fix, sats, hdop = avg(df, "gps_fix_type"), avg(df, "gps_satellites"), avg(df, "gps_hdop")
    if gps_fix is None or sats is None or hdop is None:
        add_result(rows, plain_lines, problems, "GPS Quality", "⚠️ חסר מידע", "לא התקבלו נתוני GPS מלאים.", True)
    elif gps_fix >= 3 and sats >= 8 and hdop < 1.5:
        quality = "Excellent" if hdop < 1.2 and sats >= 10 else "Good"
        add_result(rows, plain_lines, problems, "GPS Quality", "✅ תקין",
                   f"{quality}: Fix Type {gps_fix:.0f}, {sats:.0f} לוויינים, HDOP {hdop:.2f}.")
    elif gps_fix >= 3 and sats >= 8 and hdop < 2.5:
        add_result(rows, plain_lines, problems, "GPS Quality", "⚠️ גבולי",
                   f"HDOP {hdop:.2f}. מומלץ לבדוק בחוץ.", True, False)
    else:
        add_result(rows, plain_lines, problems, "GPS Quality", "❌ לא יציב",
                   f"Fix Type {gps_fix:.0f}, לוויינים {sats:.0f}, HDOP {hdop:.2f}.", True)

    ekf_vel, ekf_horiz, ekf_vert, ekf_compass = [avg(df, c) for c in [
        "ekf_velocity_variance", "ekf_pos_horiz_variance", "ekf_pos_vert_variance", "ekf_compass_variance"
    ]]
    if all(v is None for v in [ekf_vel, ekf_horiz, ekf_vert, ekf_compass]):
        add_result(rows, plain_lines, problems, "EKF", "⚠️ חסר מידע", "לא התקבל EKF_STATUS_REPORT.", True, False)
    elif max(v for v in [ekf_vel or 0, ekf_horiz or 0, ekf_vert or 0, ekf_compass or 0]) < 1.0:
        add_result(rows, plain_lines, problems, "EKF", "✅ תקין", f"Variance נמוך. Compass variance {ekf_compass:.2f}.")
    else:
        add_result(rows, plain_lines, problems, "EKF", "⚠️ אזהרה",
                   f"Variance גבוהה: Velocity={ekf_vel}, Position={ekf_horiz}, Compass={ekf_compass}.", True)

    vib = avg(df, "vibration_rms")
    vibe_x, vibe_y, vibe_z = avg(df, "vibe_x"), avg(df, "vibe_y"), avg(df, "vibe_z")
    max_vibe = max([v for v in [vibe_x, vibe_y, vibe_z] if v is not None], default=None)
    if vib is None:
        add_result(rows, plain_lines, problems, "רעידות IMU", "⚠️ חסר מידע", "לא נאספו מספיק דגימות.", True, False)
    elif vib < 3.0 and (max_vibe is None or max_vibe < 30):
        add_result(rows, plain_lines, problems, "רעידות IMU", "✅ תקין", f"RMS ממוצע {vib:.3f} m/s².")
    elif vib < 6.0:
        add_result(rows, plain_lines, problems, "רעידות IMU", "⚠️ גבולי", f"RMS {vib:.2f} m/s².", True, False)
    else:
        add_result(rows, plain_lines, problems, "רעידות IMU", "❌ לא תקין", f"RMS {vib:.2f} m/s² גבוה מדי.", True)

    clip0, clip1, clip2 = max_val(df, "clipping_0") or 0, max_val(df, "clipping_1") or 0, max_val(df, "clipping_2") or 0
    if clip0 == 0 and clip1 == 0 and clip2 == 0:
        add_result(rows, plain_lines, problems, "IMU Clipping", "✅ תקין", "לא זוהו חריגות Clipping.")
    else:
        add_result(rows, plain_lines, problems, "IMU Clipping", "⚠️ אזהרה", f"זוהה Clipping: {clip0}, {clip1}, {clip2}.", True)

    servo_values = [avg(df, f"servo{i}") for i in range(1, 5)]
    if all(v is None for v in servo_values):
        add_result(rows, plain_lines, problems, "מנועים / PWM", "⚠️ חסר מידע", "לא התקבל SERVO_OUTPUT_RAW.", True, False)
    elif all((v is not None and v == 0) for v in servo_values):
        add_result(rows, plain_lines, problems, "מנועים / PWM", "✅ תקין במצב DISARM", "יציאות המנועים 1–4 הן 0, תקין כאשר הרחפן לא חמוש.")
    elif sum(1 for v in servo_values if v is not None and v > 900) >= 4:
        add_result(rows, plain_lines, problems, "מנועים / PWM", "✅ פעיל",
                   f"PWM בערוצים 1–4: {', '.join(str(int(v)) for v in servo_values)}.")
    else:
        add_result(rows, plain_lines, problems, "מנועים / PWM", "⚠️ גבולי", f"נתוני PWM חלקיים: {servo_values}.", True, False)

    errors = max_val(df, "statustext_errors") or 0
    warnings = max_val(df, "statustext_warnings") or 0
    last_text = latest_text(df, "last_statustext")
    if errors > 0:
        add_result(rows, plain_lines, problems, "הודעות בקר טיסה", "⚠️ אזהרה",
                   f"זוהו {errors:.0f} הודעות שגיאה. הודעה אחרונה: {last_text}", True)
    elif warnings > 0:
        add_result(rows, plain_lines, problems, "הודעות בקר טיסה", "⚠️ אזהרה",
                   f"זוהו {warnings:.0f} הודעות אזהרה. הודעה אחרונה: {last_text}", True, False)
    else:
        add_result(rows, plain_lines, problems, "הודעות בקר טיסה", "✅ תקין", "לא זוהו הודעות שגיאה/אזהרה.")

    cpu_temp, cpu_usage, ram_usage = avg(df, "cpu_temp_c"), avg(df, "cpu_usage_percent"), avg(df, "ram_usage_percent")
    if cpu_temp is None:
        add_result(rows, plain_lines, problems, "Raspberry Pi", "⚠️ חסר מידע", "לא התקבלו נתוני CPU.", True, False)
    elif cpu_temp < 70 and cpu_usage < 85 and ram_usage < 85:
        add_result(rows, plain_lines, problems, "Raspberry Pi", "✅ תקין",
                   f"טמפרטורה {cpu_temp:.1f}°C, CPU {cpu_usage:.1f}%, RAM {ram_usage:.1f}%.")
    else:
        add_result(rows, plain_lines, problems, "Raspberry Pi", "⚠️ אזהרה",
                   f"טמפ׳ {cpu_temp:.1f}°C, CPU {cpu_usage:.1f}%, RAM {ram_usage:.1f}%.", True, False)

    return rows, plain_lines, problems


def metric_row(df, label, col, unit=""):
    values = numeric(df, col).dropna()
    if values.empty:
        return f"<tr><td>{label}</td><td>N/A</td><td>N/A</td><td>N/A</td><td>{unit}</td></tr>"
    return f"<tr><td>{label}</td><td>{values.mean():.2f}</td><td>{values.min():.2f}</td><td>{values.max():.2f}</td><td>{unit}</td></tr>"


def generate_report(log_path: str, out_dir: str = "reports") -> Path:
    df = pd.read_csv(log_path)
    Path(out_dir).mkdir(exist_ok=True)

    friendly_rows, plain_lines, problems = friendly_status_rows(df)
    status_counts = df["health_status"].value_counts().to_dict() if "health_status" in df else {}

    critical_problems = [p for p in problems if p.startswith("חובה לתקן")]
    if critical_problems:
        overall_status = "❌ לא מוכן לטיסה"
        overall_reason = "<br>".join(critical_problems)
    elif problems:
        overall_status = "⚠️ מוכן חלקית / דרוש שיפור"
        overall_reason = "<br>".join(problems)
    else:
        overall_status = "✅ מוכן לטיסה"
        overall_reason = "כל המערכות המרכזיות תקינות לפי הבדיקה."

    metric_cols = [
        ("MAVLink Messages", "mavlink_message_count", ""),
        ("Heartbeat Count", "heartbeat_count", ""),
        ("RC RSSI", "rc_rssi", ""),
        ("RC Channel Count", "rc_channel_count", ""),
        ("FS_THR_ENABLE", "fs_thr_enable", ""),
        ("FS_BATT_ENABLE", "fs_batt_enable", ""),
        ("FS_GCS_ENABLE", "fs_gcs_enable", ""),
        ("FS_EKF_ACTION", "fs_ekf_action", ""),
        ("BATT_FS_LOW_ACT", "batt_fs_low_act", ""),
        ("BATT_FS_CRT_ACT", "batt_fs_crt_act", ""),
        ("BATT_LOW_VOLT", "batt_low_volt", "V"),
        ("BATT_CRT_VOLT", "batt_crt_volt", "V"),
        ("RTL_ALT", "rtl_alt", "cm"),
        ("Battery Voltage", "battery_voltage_v", "V"),
        ("Battery Current", "battery_current_a", "A"),
        ("GPS Satellites", "gps_satellites", ""),
        ("GPS HDOP", "gps_hdop", ""),
        ("Relative Altitude", "relative_alt_m", "m"),
        ("EKF Compass Variance", "ekf_compass_variance", ""),
        ("EKF Velocity Variance", "ekf_velocity_variance", ""),
        ("RAW_IMU Vibration RMS", "vibration_rms", "m/s²"),
        ("VIBE X", "vibe_x", ""),
        ("VIBE Y", "vibe_y", ""),
        ("VIBE Z", "vibe_z", ""),
        ("CPU Temperature", "cpu_temp_c", "°C"),
        ("CPU Usage", "cpu_usage_percent", "%"),
        ("RAM Usage", "ram_usage_percent", "%"),
    ]
    metric_rows = [metric_row(df, *args) for args in metric_cols]

    html = f"""<!doctype html>
<html lang="he" dir="rtl">
<head>
<meta charset="utf-8">
<title>Drone Full Pre-Flight Health Report</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 30px; direction: rtl; }}
table {{ border-collapse: collapse; width: 100%; margin-top: 10px; }}
td, th {{ border: 1px solid #cccccc; padding: 10px; text-align: right; }}
th {{ background: #eeeeee; }}
.box {{ background: #f7f7f7; border: 1px solid #dddddd; padding: 15px; border-radius: 8px; margin-bottom: 20px; }}
.overall {{ font-size: 22px; font-weight: bold; }}
</style>
</head>
<body>
<h1>דוח תקינות Pre-Flight מלא לרחפן</h1>
<p><b>תאריך יצירה:</b> {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}</p>
<p><b>קובץ לוג:</b> {Path(log_path).name}</p>
<p><b>מספר דגימות:</b> {len(df)}</p>

<div class="box">
<h2>סטטוס כללי</h2>
<p class="overall">{overall_status}</p>
<p>{overall_reason}</p>
</div>

<div class="box">
<h2>סיכום פשוט למשתמש</h2>
<table>
<tr><th>מערכת</th><th>מצב</th><th>פירוט</th></tr>
{''.join(friendly_rows)}
</table>
</div>

<h2>נתונים מספריים</h2>
<table>
<tr><th>מדד</th><th>ממוצע</th><th>מינימום</th><th>מקסימום</th><th>יחידה</th></tr>
{''.join(metric_rows)}
</table>

<h2>ספירת מצבי מערכת</h2>
<pre>{status_counts}</pre>

<h2>הערה</h2>
<p>
הדוח הוא בדיקת Pre-Flight אוטומטית. לפני טיסה אמיתית יש לאמת ידנית גם:
כיוון מנועים, כיול מצפן, Failsafe, ARM/DISARM, ובדיקת Mission Planner/QGroundControl.
</p>
</body>
</html>"""

    report_path = Path(out_dir) / f"full_preflight_drone_health_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
    report_path.write_text(html, encoding="utf-8")
    return report_path


def text_summary(log_path: str) -> str:
    df = pd.read_csv(log_path)
    _, plain_lines, problems = friendly_status_rows(df)
    critical_problems = [p for p in problems if p.startswith("חובה לתקן")]

    if critical_problems:
        overall = "סטטוס כללי: לא מוכן לטיסה ❌"
        problem_text = "\n".join(f"- {p}" for p in critical_problems)
    elif problems:
        overall = "סטטוס כללי: מוכן חלקית / דרוש שיפור ⚠️"
        problem_text = "\n".join(f"- {p}" for p in problems)
    else:
        overall = "סטטוס כללי: מוכן לטיסה ✅"
        problem_text = "לא נמצאו בעיות מרכזיות."

    return f"""דוח תקינות Pre-Flight מלא לרחפן

{overall}

סיבות/הערות:
{problem_text}

סיכום מערכות:
{chr(10).join(plain_lines)}

מספר דגימות: {len(df)}

מצורפים למייל:
1. דוח HTML ידידותי
2. קובץ CSV מלא עם כל הנתונים
"""
