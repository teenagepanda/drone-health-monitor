# Height calibration series update

This update adds calibration-series labels and planned physical offsets to the height-calibration mode.

## New arguments

- `--test-type center|right|left|forward|backward|custom`
- `--offset-x <cm>`: right positive, left negative
- `--offset-y <cm>`: forward positive, backward negative
- `--calibration-summary-output <path>`

When no offsets are entered, the standard series use these defaults:

- center: X=0 cm, Y=0 cm
- right: X=+5 cm, Y=0 cm
- left: X=-5 cm, Y=0 cm
- forward: X=0 cm, Y=+5 cm
- backward: X=0 cm, Y=-5 cm

Raw samples append to:

```text
reports/height_calibration_raw.csv
```

One summary row per run appends to:

```text
reports/height_calibration_summary.csv
```

## Recommended sample plan per height

- Center: 30 samples
- Right: 10 samples
- Left: 10 samples
- Forward: 10 samples
- Backward: 10 samples

## Example at 0.25 m

Center:

```bash
python3 src/run_marker_test.py --camera-backend picamera2 --marker-type template --reference markers/references --show --duration 60 --calibrate-height --real-height 0.25 --test-type center --calibration-samples 30
```

Right by 5 cm:

```bash
python3 src/run_marker_test.py --camera-backend picamera2 --marker-type template --reference markers/references --show --duration 60 --calibrate-height --real-height 0.25 --test-type right --offset-x 5 --offset-y 0 --calibration-samples 10
```

Repeat for `left`, `forward`, and `backward`, then continue at 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, and 2.00 m.
