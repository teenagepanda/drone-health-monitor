#!/usr/bin/env bash
set -euo pipefail

cd ~/drone-health-monitor

HEIGHTS=("0.50" "1.00" "1.50" "2.00")

echo "בדיקת גובה חזותי — ללא מדחפים."
echo "יש למדוד מהעדשה עד פני הסמן."

for REAL_HEIGHT in "${HEIGHTS[@]}"; do
    echo
    echo "======================================================"
    echo "מקם את עדשת המצלמה בגובה ${REAL_HEIGHT} מטר מעל הסמן."
    read -r -p "לחץ Enter כשהרחפן יציב בגובה זה..."

    BEFORE_LOG="$(ls -t logs/landing_debug_*.csv 2>/dev/null | head -n 1 || true)"

    python3 src/main.py \
      --connection /dev/ttyACM0 \
      --baud 115200 \
      --duration 20 \
      --autoland-on-marker \
      --camera-backend picamera2 \
      --marker-type template \
      --marker-reference markers/references/original.png \
      --landing-calibration-csv reports/height_calibration_clean.csv \
      --landing-altitude-source visual \
      --landing-min-confidence 0.90 \
      --landing-stable-frames 5

    NEW_LOG="$(ls -t logs/landing_debug_*.csv | head -n 1)"
    OUTPUT="logs/height_test_${REAL_HEIGHT//./_}m.csv"
    cp "$NEW_LOG" "$OUTPUT"

    python3 - "$OUTPUT" "$REAL_HEIGHT" <<'PY'
import csv
import statistics
import sys

path = sys.argv[1]
real_height = float(sys.argv[2])

values = []

with open(path, newline="", encoding="utf-8") as file:
    for row in csv.DictReader(file):
        try:
            if row["detected"].lower() != "true":
                continue
            if float(row["confidence_percent"]) < 90:
                continue
            value = float(row["visual_alt_m"])
            values.append(value)
        except (ValueError, KeyError):
            pass

if not values:
    print("לא נמצאו מדידות גובה תקינות.")
    raise SystemExit(1)

average = statistics.mean(values)
stdev = statistics.stdev(values) if len(values) > 1 else 0.0
error = average - real_height
error_percent = abs(error) / real_height * 100

print()
print(f"Real height:       {real_height:.2f} m")
print(f"Visual average:    {average:.3f} m")
print(f"Standard deviation:{stdev:.3f} m")
print(f"Signed error:      {error:+.3f} m")
print(f"Absolute error:    {abs(error):.3f} m")
print(f"Error percentage:  {error_percent:.1f}%")
print(f"Accepted samples:  {len(values)}")
PY
done

echo
echo "Height tests completed."
echo "Saved files:"
ls -1 logs/height_test_*m.csv
