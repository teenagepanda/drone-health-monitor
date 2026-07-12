# Drone Health Monitor v12

Version 12 merges the visual-marker, Picamera2, email-reporting and visual-landing components into one update.

## Included

- Raspberry Pi CSI camera support through Picamera2.
- Multi-reference template matching in the fixed order: `original`, `A`, `B`, `C`, `D`, `E`.
- Optional ArUco mode.
- Console output with confidence percentage, first-detection time, frame-processing time and FPS.
- On-screen overlay with marker outline, marker center, camera center, correction arrow, X/Y pixel offset, confidence, reference name and system state.
- Email notification with the detected frame.
- Visual landing controller in **dry-run mode by default**.
- Real MAVLink landing commands only when `--enable-autolanding` is supplied.

## Install this update

From the project root:

```bash
unzip -o drone_health_monitor_v12_unified_update.zip
```

The ZIP only replaces relevant code and marker-reference files. It does not contain `.env`, `config/email_config.json`, logs, reports or the virtual environment.

## Marker-only test

```bash
python src/run_marker_test.py --camera-backend picamera2 --marker-type template --reference markers/references --duration 60 --show --send-email
```

Expected output example:

```text
FIRST DETECTION | reference=C | confidence=86.7% | detection time=1.84s
```

## Full health test with dry-run landing

```bash
python src/main.py --connection /dev/ttyACM0 --baud 115200 --duration 180 --detect-marker --camera-backend picamera2 --marker-type template --marker-reference markers/references --autoland-on-marker --send-email
```

Do not add `--enable-autolanding` during camera, bench or hand-held tests.

## Reference order

```text
markers/references/original.png
markers/references/A.png
markers/references/B.png
markers/references/C.png
markers/references/D.png
markers/references/E.png
```

The next reference is checked only if the previous reference does not pass the confidence threshold.

## Safety

- First integration tests: propellers removed.
- Dry-run mode only until direction signs, camera orientation and X/Y offsets are verified.
- The program never arms the aircraft and never initiates takeoff.
- Actual commands require the explicit `--enable-autolanding` flag.
