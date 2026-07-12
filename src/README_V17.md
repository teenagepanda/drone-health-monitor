# V17 — Dry-Run and Landing Controller Integration

## Main changes

- Dedicated landing-debug CSV for every controller update.
- On-screen landing overlay with state, confidence, visual altitude, FC altitude,
  offset in pixels/meters and calculated MAVLink velocity.
- Visual altitude estimation from the clean height-calibration table.
- Direct pixel-to-meter scale from the currently detected physical marker.
- Only the `original` reference is accepted by default.
- Landing confidence threshold defaults to 90%.
- Consecutive stable frames are required before alignment/descent.
- Marker-loss zero-velocity hold.
- Real commands remain blocked unless explicitly enabled, armed and in GUIDED.
- `height_calibration_clean.csv` is now the default calibration file.
- Clean CSV files without a `reference` column are supported.

## Safe indoor dry-run

Remove the propellers. Connect Pixhawk and Raspberry Pi. Do not add
`--enable-autolanding`.

```bash
python3 src/main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --duration 180 \
  --autoland-on-marker \
  --camera-backend picamera2 \
  --marker-type template \
  --marker-reference markers/references/original.png \
  --landing-calibration-csv reports/height_calibration_clean.csv \
  --landing-altitude-source visual \
  --landing-min-confidence 0.90 \
  --landing-stable-frames 5 \
  --show
```

Press `q` in the camera window or `Ctrl+C` in the terminal to stop.

## Expected dry-run output

The terminal prints once per second:

- controller state: SEARCHING / ACQUIRING / ALIGNING / DESCENDING / HOLD_MIN_ALT
- visual height and flight-controller relative altitude
- marker side in pixels and image scale
- horizontal error in pixels and meters
- calculated vx, vy and vz
- reason real commands are blocked

A dedicated file is generated:

```text
logs/landing_debug_YYYYMMDD_HHMMSS.csv
```

## Direction test

Move the drone by hand while holding it level over the floor marker:

1. Centered: state should become DESCENDING after stable frames.
2. Move right/left: horizontal error and `vy` should change.
3. Move forward/back: horizontal error and `vx` should change.
4. Hide marker: state should become MARKER_LOST and command should be zero.
5. Move near the 0.25–0.30 m point: state should become HOLD_MIN_ALT.

If an axis is reversed, add `--landing-invert-x` or `--landing-invert-y`.

## Real-command bench test

Only after the dry-run values are verified, with propellers still removed:

```bash
... --enable-autolanding
```

Real velocity output is still blocked unless the vehicle is armed and in GUIDED.
The program never arms or takes off by itself.
