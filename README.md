# Drone Health Monitor V14

V14 adds CSV-based camera/marker calibration to the ArUco detection loop.

## Main behavior

1. Loads empirical calibration measurements from `data/camera_height_calibration.csv`.
2. Interpolates calibration values between measured heights.
3. Converts marker pixel offset into metric X/Y offset.
4. Keeps autonomous movement disabled by default.
5. Logs detections and the calibration row used.
6. Optionally loads camera intrinsics from `data/camera_intrinsics.csv`.

## Expected empirical CSV

Required columns:

```csv
height_m,px_per_meter_x,px_per_meter_y
0.25,1450,1430
0.50,725,715
...
```

Optional columns:

- `center_x_px`
- `center_y_px`
- `notes`

The values in the included CSV are placeholders. Replace them with the values obtained from your tests.

## Optional intrinsic CSV

```csv
fx,fy,cx,cy,k1,k2,p1,p2,k3
...
```

## Install

```bash
cd ~/drone_health_monitor_V14
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Camera-only test

```bash
python3 src/main.py \
  --calibration data/camera_height_calibration.csv \
  --intrinsics data/camera_intrinsics.csv \
  --marker-size 0.20 \
  --marker-id 0 \
  --show
```

Press `q` to stop.

## MAVLink-connected test

```bash
python3 src/main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --calibration data/camera_height_calibration.csv \
  --marker-size 0.20 \
  --marker-id 0 \
  --show
```

V14 does not send movement commands unless `--enable-control` is explicitly supplied.
Even with that flag, the included controller is limited to generating/logging a proposed velocity command.
It does not transmit the command to the flight controller.

## Safety

- First run without propellers.
- Verify the displayed direction signs by moving the marker manually.
- Confirm the measured height source.
- Do not enable automatic landing until all calibration rows are validated.
