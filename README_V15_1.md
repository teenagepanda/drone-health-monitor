# V15.1 - Camera-only test separation

## Fixes

- Fixed mixed tabs/spaces in `src/visual_marker_detector.py`.
- Added a camera-only mode to `src/run_health_test.py`.
- Camera-only mode does not create a MAVLink connection and therefore does not require `/dev/ttyACM0`.
- Autonomous landing commands are blocked in camera-only mode.
- Existing standalone `src/run_marker_test.py` remains available for detection and height-calibration measurements.

## Camera-only test through main.py

Run from the project root:

```bash
python3 src/main.py \
  --no-flight-controller \
  --duration 60 \
  --camera-backend picamera2 \
  --marker-type template \
  --marker-reference markers/references \
  --show
```

Press `q` to close the preview window, or `Ctrl+C` to stop.

## Standalone marker test

```bash
python3 src/run_marker_test.py \
  --camera-backend picamera2 \
  --marker-type template \
  --reference markers/references \
  --duration 60 \
  --show
```

## Full flight-controller mode

Do not use `--no-flight-controller` when the Pixhawk is connected:

```bash
python3 src/main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --duration 180
```

## Safety

`--enable-autolanding` cannot be combined with `--no-flight-controller`.
