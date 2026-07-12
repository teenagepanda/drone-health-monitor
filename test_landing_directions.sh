#!/usr/bin/env bash
set -euo pipefail

cd ~/drone-health-monitor

echo "DRY RUN — ללא מדחפים וללא פקודות אמיתיות."
echo "במהלך 90 השניות:"
echo "1. החזק מעל מרכז הסמן."
echo "2. הזז ימינה."
echo "3. הזז שמאלה."
echo "4. הזז קדימה."
echo "5. הזז אחורה."
echo "לחץ q בחלון או Ctrl+C לעצירה."

python3 src/main.py \
  --connection /dev/ttyACM0 \
  --baud 115200 \
  --duration 90 \
  --autoland-on-marker \
  --camera-backend picamera2 \
  --marker-type template \
  --marker-reference markers/references/original.png \
  --landing-calibration-csv reports/height_calibration_clean.csv \
  --landing-altitude-source visual \
  --landing-min-confidence 0.90 \
  --landing-stable-frames 5 \
  --show

LATEST_LOG="$(ls -t logs/landing_debug_*.csv | head -n 1)"

echo
echo "Latest landing log: $LATEST_LOG"
echo "Showing accepted measurements:"

python3 - "$LATEST_LOG" <<'PY'
import csv
import sys

path = sys.argv[1]

print(
    f"{'Time':>8} {'State':>12} "
    f"{'X(px)':>8} {'Y(px)':>8} "
    f"{'X(m)':>9} {'Y(m)':>9} "
    f"{'VX':>8} {'VY':>8}"
)

with open(path, newline="", encoding="utf-8") as file:
    rows = list(csv.DictReader(file))

for row in rows:
    if row["detected"].lower() != "true":
        continue

    print(
        f"{float(row['elapsed_s']):8.2f} "
        f"{row['state']:>12} "
        f"{float(row['error_x_px'] or 0):8.0f} "
        f"{float(row['error_y_px'] or 0):8.0f} "
        f"{float(row['error_x_m'] or 0):9.3f} "
        f"{float(row['error_y_m'] or 0):9.3f} "
        f"{float(row['vx_mps'] or 0):8.3f} "
        f"{float(row['vy_mps'] or 0):8.3f}"
    )
PY
