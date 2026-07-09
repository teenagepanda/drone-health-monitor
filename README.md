# Drone Health Monitor V3 — Raspberry Pi + Pixhawk USB + Email Report

Runs a timed drone health test, logs MAVLink telemetry, creates an HTML report, and sends the report by email.

## Connection
Raspberry Pi USB → Pixhawk Type-C

Default port:
```bash
/dev/ttyACM0
```

## Install
```bash
sudo apt update
sudo apt install -y python3-dev build-essential python3-pip python3-setuptools python3-wheel

python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

## Configure email safely
```bash
cp .env.example .env
nano .env
```

Inside `.env`:
```text
EMAIL_FROM=your_email@gmail.com
EMAIL_PASSWORD=your_google_app_password
EMAIL_TO=your_email@gmail.com
```

Use Gmail App Password, not your normal Gmail password.

## Run 3-minute health test and send email
```bash
source .venv/bin/activate
python3 src/run_health_test.py --connection /dev/ttyACM0 --baud 115200 --duration 180 --send-email
```

## Run without email
```bash
python3 src/run_health_test.py --connection /dev/ttyACM0 --baud 115200 --duration 180
```

## Safety
First tests should be done without propellers.


## Continuous periodic email mode

Run forever and send a report every 5 minutes:

```bash
python3 src/periodic_email_monitor.py --connection /dev/ttyACM0 --baud 115200 --interval 300
```

Run forever and send a report every 15 minutes:

```bash
python3 src/periodic_email_monitor.py --connection /dev/ttyACM0 --baud 115200 --interval 900
```

Stop with:

```text
Ctrl + C
```

Each interval creates:
- A new CSV log
- A new HTML report
- One email with the summary and attachments


## V6 Pre-Flight report additions

This version adds:
- Pixhawk ↔ Raspberry Pi MAVLink communication status
- Flight controller status
- RC / ELRS link using RC_CHANNELS
- EKF status
- Compass status using EKF compass variance
- PWM / motor output status
- Autopilot STATUSTEXT warning/error count
- Overall result: Ready for flight / Not ready for flight

Run one test and send email:

```bash
python3 src/run_health_test.py --connection /dev/ttyACM0 --baud 115200 --duration 60 --send-email
```


## V7 Full Pre-Flight additions

Added:
- Failsafe parameters: FS_THR_ENABLE, FS_BATT_ENABLE, FS_GCS_ENABLE, FS_EKF_ACTION, RTL_ALT
- Home Position check through HOME_POSITION
- Full PreArm message list
- Flight-mode suitability note
- Sensor health from SYS_STATUS: gyro, accelerometer, compass, barometer, GPS
- GPS global/relative altitude fields
- Improved final status:
  - Ready for flight
  - Partially ready / needs improvement
  - Not ready for flight

Run:

```bash
python3 src/run_health_test.py --connection /dev/ttyACM0 --baud 115200 --duration 60 --send-email
```

## Visual marker detection test

This version includes a camera test for the visual landing marker.
The reference image is saved here:

```text
markers/reference_marker.png
```

Install the added dependencies:

```bash
pip install -r requirements.txt
```

Run only the visual-marker test:

```bash
python3 src/run_marker_test.py --camera-index 0 --duration 60 --send-email
```

Expected terminal output when the camera sees the marker:

```text
✅ Visual marker detected | score=0.82
FINAL RESULT: visual marker was detected.
```

Run the regular drone health test and also check the marker during the test:

```bash
python3 src/main.py --connection /dev/ttyACM0 --baud 115200 --duration 180 --detect-marker --camera-index 0 --email-on-marker --send-email
```

Notes:

- First tests should be done without propellers.
- Keep the marker flat and inside the camera frame.
- Use good lighting and avoid glare on the printed page.
- If the camera is not found, try `--camera-index 1`.
- If the detector is too strict or too loose, adjust `--marker-threshold`, for example `--marker-threshold 0.68`.
