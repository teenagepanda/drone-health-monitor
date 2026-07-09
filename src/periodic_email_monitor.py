import argparse
import time
from datetime import datetime

from mavlink_reader import MavlinkReader
from vibration import VibrationAnalyzer
from health_checks import check_drone_health
from logger import CsvLogger
from pi_health import get_cpu_temp_c, get_cpu_usage_percent, get_ram_usage_percent
from report_generator import generate_report, text_summary
from email_sender import send_email_report
import config


def run_single_interval(reader, duration, log_dir, report_dir):
    vibration = VibrationAnalyzer(window_size=100)
    logger = CsvLogger(log_dir)

    start = time.time()
    last_print = 0.0

    try:
        while time.time() - start < duration:
            tel = reader.read_messages()

            vibration.add_sample(tel.acc_x, tel.acc_y, tel.acc_z)
            vib_rms = vibration.rms()

            cpu_temp = get_cpu_temp_c()
            cpu_usage = get_cpu_usage_percent()
            ram_usage = get_ram_usage_percent()

            health = check_drone_health(tel, vib_rms, cpu_temp, cpu_usage, ram_usage)
            logger.write(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health)

            now = time.time()
            if now - last_print >= config.STATUS_PRINT_INTERVAL_SECONDS:
                last_print = now
                remaining = int(duration - (now - start))
                print_status(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health, remaining)

            time.sleep(config.LOG_INTERVAL_SECONDS)

    finally:
        logger.close()

    report_path = generate_report(str(logger.path), report_dir)
    summary = text_summary(str(logger.path))

    return str(logger.path), str(report_path), summary


def main():
    parser = argparse.ArgumentParser(description="Continuous drone health monitor with periodic email reports")
    parser.add_argument("--connection", default=config.DEFAULT_CONNECTION)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--interval", type=int, default=300, help="Email interval in seconds. Example: 300 = 5 minutes")
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--report-dir", default="reports")
    args = parser.parse_args()

    print("Starting continuous periodic email monitor.")
    print(f"Email interval: {args.interval} seconds")
    print("Stop with CTRL+C.")
    print("Safety: first tests should be done without propellers.")

    reader = MavlinkReader(args.connection, args.baud)
    reader.connect()

    cycle = 1

    try:
        while True:
            print("=" * 80)
            print(f"Starting cycle #{cycle} at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

            log_path, report_path, summary = run_single_interval(
                reader=reader,
                duration=args.interval,
                log_dir=args.log_dir,
                report_dir=args.report_dir,
            )

            subject = f"Drone Health Report - Cycle #{cycle}"
            send_email_report(
                subject=subject,
                body=summary,
                attachments=[log_path, report_path],
            )

            print(f"Cycle #{cycle} completed.")
            print(f"CSV log: {log_path}")
            print(f"HTML report: {report_path}")
            print("Email sent successfully.")

            cycle += 1

    except KeyboardInterrupt:
        print("\nPeriodic monitor stopped by user.")


def fmt(value, suffix="", digits=2):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}{suffix}"
    return f"{value}{suffix}"


def print_status(tel, vib_rms, cpu_temp, cpu_usage, ram_usage, health, remaining):
    print("-" * 80)
    print(f"Remaining until email: {remaining}s | Status: {health.status}")
    print(f"Mode: {tel.mode} | Armed: {tel.armed}")
    print(
        f"Battery: {fmt(tel.battery_voltage_v, ' V')} | "
        f"Current: {fmt(tel.battery_current_a, ' A')} | "
        f"Remaining: {fmt(tel.battery_remaining_percent, '%', 0)}"
    )
    print(f"GPS: fix_type={tel.gps_fix_type} | satellites={tel.gps_satellites} | HDOP={fmt(tel.gps_hdop)}")
    print(f"RAW_IMU vibration RMS: {fmt(vib_rms, ' m/s^2')}")
    print(f"ArduPilot VIBE: X={fmt(tel.vibe_x)} | Y={fmt(tel.vibe_y)} | Z={fmt(tel.vibe_z)}")
    print(f"IMU clipping: {tel.clipping_0}, {tel.clipping_1}, {tel.clipping_2}")
    print(f"Pi CPU: {cpu_usage:.1f}% | RAM: {ram_usage:.1f}% | Temp: {fmt(cpu_temp, ' C')}")
    print("Messages:")
    for msg in health.messages:
        print(f" - {msg}")


if __name__ == "__main__":
    main()
