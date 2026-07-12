# Drone Health Monitor v11 – Template Marker + Visual Landing Controller

This update keeps the old template/reference marker as the default until the ArUco marker is printed.

## What changed

- `run_marker_test.py` default marker type changed back to `template`.
- `run_health_test.py` default marker type changed back to `template`.
- Added `src/landing_controller.py`.
- Added optional visual landing controller flags to the full health test.

## Safety behavior

Autonomous landing is disabled by default. When `--autoland-on-marker` is used without `--enable-autolanding`, the controller runs in dry-run mode and only prints the MAVLink velocity commands it would send.

The script does not arm the drone and does not take off. Test first in SITL or with propellers removed.

## Marker test with old image

```bash
python src/run_marker_test.py --camera-backend picamera2 --marker-type template --reference markers/reference_marker.png --duration 60 --show --send-email
```

## Full test with old image and dry-run landing controller

```bash
python src/main.py --connection /dev/ttyACM0 --baud 115200 --duration 180 --detect-marker --camera-backend picamera2 --marker-type template --marker-reference markers/reference_marker.png --autoland-on-marker --send-email
```

## Real command mode, only after safe tests

```bash
python src/main.py --connection /dev/ttyACM0 --baud 115200 --duration 180 --detect-marker --camera-backend picamera2 --marker-type template --marker-reference markers/reference_marker.png --autoland-on-marker --enable-autolanding
```

Recommended first real tests: very low altitude, open area, kill switch ready, and GUIDED mode verified.
