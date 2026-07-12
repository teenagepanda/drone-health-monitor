# Height calibration mode

This update adds a calibration mode to `src/run_marker_test.py`.

Measure height from the camera lens to the marker surface. Keep the camera as vertical and stable as possible.

Example for 0.50 m:

```bash
python3 src/run_marker_test.py --camera-backend picamera2 --marker-type template --reference markers/references --show --duration 60 --calibrate-height --real-height 0.50 --calibration-samples 30
```

Repeat at these heights:

- 0.50 m
- 0.75 m
- 1.00 m
- 1.25 m
- 1.50 m
- 1.75 m
- 2.00 m

All runs append to:

```text
reports/height_calibration.csv
```

Recorded fields include real height, reference used, confidence, marker width and height in pixels, marker area, X/Y offsets, processing time, FPS, and elapsed time.

Use `q` to stop early when `--show` is enabled. If the requested sample count is reached, the program stops automatically.
