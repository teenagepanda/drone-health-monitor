# V18 — Flight Readiness Report

Copy `src/flight_readiness_report.py` into the project's `src/` directory.

Run after a V17 dry-run:

```bash
python3 src/flight_readiness_report.py --profile prop-less
```

Optional X/Y confirmation:

```bash
cp direction_test_results.example.json direction_test_results.json
nano direction_test_results.json
```

Set:

```json
{
  "x_correct": true,
  "y_correct": true
}
```

Then run:

```bash
python3 src/flight_readiness_report.py   --profile prop-less   --direction-results direction_test_results.json
```

Outputs are written to `reports/` as TXT, CSV and JSON.

Profiles:

```bash
--profile prop-less
--profile first-flight
--profile autonomous-flight
```

Missing evidence is reported as NOT_TESTED or WARNING. Invalid calibration or
Python syntax is reported as FAIL. The report does not arm or control the drone.
