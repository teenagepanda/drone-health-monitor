# V16 — Validated Height-Calibrated Landing

## Main changes

- Height calibration loads only the `original` reference image.
- Calibration samples are accepted only at confidence >= 90% by default.
- Rejected samples do not count toward the requested sample total.
- Runtime width stability is displayed using coefficient of variation (CV).
- Center-calibration summaries are checked for a decreasing marker size as height increases.
- The landing controller converts pixel error to meters using the validated summary CSV.
- Linear interpolation is used between measured heights.
- Invalid or unavailable calibration automatically causes a safe pixel-control fallback.
- Camera-only operation remains available through `--no-flight-controller`.

## Install/update

Copy the files from `src/` into the repository's existing `src/` directory, replacing the older versions.

Then verify:

```bash
cd ~/drone-health-monitor
python3 -m compileall -q src
python3 src/run_marker_test.py --help | grep calibration-min-confidence
```

## Height calibration example

```bash
python3 src/run_marker_test.py \
  --camera-backend picamera2 \
  --marker-type template \
  --reference markers/references \
  --calibrate-height \
  --real-height 1.75 \
  --test-type center \
  --calibration-samples 30 \
  --calibration-min-confidence 90 \
  --show
```

Expected startup output includes:

```text
Calibration reference lock: 'original' only
minimum confidence=90.0%
```

## Camera-only test

```bash
python3 src/main.py \
  --no-flight-controller \
  --duration 60 \
  --camera-backend picamera2 \
  --marker-type template \
  --marker-reference markers/references \
  --show
```

## Calibrated landing dry run

Run this only with the flight controller connected and first without propellers:

```bash
python3 src/main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --duration 180 \
  --autoland-on-marker \
  --camera-backend picamera2 \
  --marker-type template \
  --marker-reference markers/references \
  --landing-calibration-csv reports/height_calibration_summary.csv \
  --landing-marker-size-m 0.20
```

Do not add `--enable-autolanding` until dry-run values have been checked.
