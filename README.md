# Drone Health Monitor V14 — direct summary CSV calibration

This revision loads calibration directly from:

```text
reports/height_calibration_summary.csv
```

No separate `camera_height_calibration.csv` is needed.

## Calibration behavior

- Only rows with `test_type=center` are used.
- `real_height_m` is used as the measured height.
- `avg_marker_width_px` and `avg_marker_height_px` are converted to pixels-per-meter using the real printed marker size.
- If a height appears more than once, the newest row by `timestamp` is used.
- Heights between measured points use linear interpolation.
- Heights outside the measured range are clamped to the closest measured height.
- Calibration is loaded once when V14 starts.

## Required CSV columns

```text
timestamp
real_height_m
test_type
avg_marker_width_px
avg_marker_height_px
```

## Install/update

Copy the contents into the repository, then:

```bash
cd ~/drone-health-monitor
source venv/bin/activate
pip install -r requirements.txt
```

## Validate the saved calibration

For a 20 cm marker:

```bash
python3 src/validate_summary_calibration.py \
  reports/height_calibration_summary.csv \
  --marker-size 0.20
```

## Camera-only test

```bash
python3 src/v14_main.py \
  --calibration-summary reports/height_calibration_summary.csv \
  --marker-size 0.20 \
  --marker-id 0 \
  --manual-height 0.25 \
  --show
```

## Test with Pixhawk height

```bash
python3 src/v14_main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --calibration-summary reports/height_calibration_summary.csv \
  --marker-size 0.20 \
  --marker-id 0 \
  --show
```

## Safety

V14 calculates and logs proposed centering velocities only. It does not transmit movement commands to the flight controller.
Verify direction signs without propellers before adding real movement transmission.
