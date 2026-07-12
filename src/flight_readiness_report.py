from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Optional

PASS = "PASS"
WARNING = "WARNING"
FAIL = "FAIL"
NOT_TESTED = "NOT_TESTED"


@dataclass
class CheckResult:
    name: str
    status: str
    value: str = ""
    details: str = ""
    score: float = 0.0


@dataclass
class SectionResult:
    name: str
    status: str
    score: float
    checks: list[CheckResult] = field(default_factory=list)


@dataclass
class ReadinessReport:
    generated_at: str
    project_version: str
    profile: str
    sections: list[SectionResult]
    overall_score: float
    overall_status: str
    readiness_level: str
    actions_required: list[str]
    source_files: dict[str, str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate autonomous landing flight-readiness report."
    )
    parser.add_argument(
        "--profile",
        choices=["prop-less", "first-flight", "autonomous-flight"],
        default="prop-less",
    )
    parser.add_argument("--calibration", default="reports/height_calibration_clean.csv")
    parser.add_argument("--summary-calibration", default="reports/height_calibration_summary.csv")
    parser.add_argument("--landing-log")
    parser.add_argument("--height-test-glob", default="logs/height_test_*m.csv")
    parser.add_argument("--direction-results")
    parser.add_argument("--version-file", default="VERSION")
    parser.add_argument("--output-dir", default="reports")
    parser.add_argument("--min-confidence", type=float, default=90.0)
    parser.add_argument("--max-height-mean-error-m", type=float, default=0.08)
    parser.add_argument("--max-height-error-m", type=float, default=0.15)
    parser.add_argument(
        "--expected-heights",
        default="0.25,0.50,0.75,1.00,1.25,1.50,1.75,2.00",
    )
    return parser.parse_args()


def newest_file(pattern: str) -> Optional[Path]:
    files = [p for p in Path(".").glob(pattern) if p.is_file()]
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


def read_csv(path: Optional[Path]) -> list[dict[str, str]]:
    if path is None or not path.exists():
        return []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def number(value: Any) -> Optional[float]:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def boolean(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "pass", "ok"}:
        return True
    if text in {"false", "0", "no", "fail"}:
        return False
    return None


def make_check(name: str, status: str, value: str = "", details: str = "") -> CheckResult:
    score_map = {PASS: 100.0, WARNING: 65.0, FAIL: 0.0, NOT_TESTED: 40.0}
    return CheckResult(name, status, value, details, score_map[status])


def section_result(name: str, checks: list[CheckResult]) -> SectionResult:
    scored = [c.score for c in checks if c.status != NOT_TESTED]
    score = statistics.mean(scored) if scored else 0.0
    if any(c.status == FAIL for c in checks):
        status = FAIL
    elif any(c.status in {WARNING, NOT_TESTED} for c in checks):
        status = WARNING
    else:
        status = PASS
    return SectionResult(name, status, score, checks)


def software_section(version_file: Path) -> SectionResult:
    checks: list[CheckResult] = []
    version = version_file.read_text(encoding="utf-8").strip() if version_file.exists() else "unknown"
    checks.append(make_check(
        "Project version",
        PASS if version.startswith("V18") else WARNING,
        version,
        "V18 is the readiness-report release.",
    ))

    required = [
        Path("src/main.py"),
        Path("src/run_health_test.py"),
        Path("src/landing_controller.py"),
        Path("src/landing_debug.py"),
        Path("src/camera_calibration.py"),
        Path("src/camera_capture.py"),
    ]
    missing = [str(p) for p in required if not p.exists()]
    checks.append(make_check(
        "Required source files",
        PASS if not missing else FAIL,
        f"{len(required) - len(missing)}/{len(required)}",
        "Missing: " + ", ".join(missing) if missing else "All required integration files exist.",
    ))

    errors = []
    for path in Path("src").glob("*.py"):
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            errors.append(f"{path.name}: {result.stderr.strip()}")
    checks.append(make_check(
        "Python syntax",
        PASS if not errors else FAIL,
        "all files compile" if not errors else f"{len(errors)} errors",
        "\n".join(errors),
    ))
    return section_result("Software integration", checks)


def calibration_section(
    clean_path: Path,
    summary_path: Path,
    expected_heights: list[float],
    min_confidence: float,
) -> SectionResult:
    checks: list[CheckResult] = []
    clean_rows = read_csv(clean_path)
    if not clean_rows:
        return section_result("Height calibration", [
            make_check("Calibration file", FAIL, str(clean_path), "Missing or empty clean calibration file.")
        ])

    points: list[tuple[float, float, float]] = []
    for row in clean_rows:
        h = number(row.get("real_height_m"))
        w = number(row.get("avg_marker_width_px"))
        mh = number(row.get("avg_marker_height_px"))
        if h is not None and w is not None and mh is not None:
            points.append((h, w, mh))
    points.sort()

    found = {round(h, 2) for h, _, _ in points}
    missing = [h for h in expected_heights if round(h, 2) not in found]
    checks.append(make_check(
        "Calibration height points",
        PASS if not missing else FAIL,
        f"{len(found)}/{len(expected_heights)}",
        "Missing: " + ", ".join(f"{h:.2f}m" for h in missing) if missing else "All expected heights exist.",
    ))

    problems = []
    for lower, upper in zip(points, points[1:]):
        lower_side = (lower[1] + lower[2]) / 2
        upper_side = (upper[1] + upper[2]) / 2
        if upper_side >= lower_side:
            problems.append(
                f"{lower[0]:.2f}m={lower_side:.1f}px -> {upper[0]:.2f}m={upper_side:.1f}px"
            )
    checks.append(make_check(
        "Monotonic marker size",
        PASS if not problems else FAIL,
        "decreases with height" if not problems else "invalid",
        "; ".join(problems) if problems else "Marker size decreases at every higher point.",
    ))

    ratios = [max(w, h) / min(w, h) for _, w, h in points if min(w, h) > 0]
    max_ratio = max(ratios) if ratios else math.inf
    checks.append(make_check(
        "Marker geometry consistency",
        PASS if max_ratio <= 1.20 else WARNING,
        f"max side ratio={max_ratio:.3f}",
        "Near 1.0 indicates a square marker.",
    ))

    summary_rows = read_csv(summary_path)
    confidences = [number(r.get("avg_confidence_percent")) for r in summary_rows]
    confidences = [v for v in confidences if v is not None]
    if confidences:
        avg_conf = statistics.mean(confidences)
        checks.append(make_check(
            "Calibration confidence",
            PASS if avg_conf >= min_confidence else WARNING,
            f"{avg_conf:.1f}%",
            f"Target is at least {min_confidence:.1f}%.",
        ))
    else:
        checks.append(make_check(
            "Calibration confidence",
            NOT_TESTED,
            "",
            "No confidence values were found in the summary file.",
        ))
    return section_result("Height calibration", checks)


def landing_section(path: Optional[Path], min_confidence: float) -> SectionResult:
    rows = read_csv(path)
    if not rows:
        return section_result("Landing controller", [
            make_check("Landing debug log", FAIL, str(path or "not found"), "Run a V17 dry-run first.")
        ])

    checks: list[CheckResult] = []
    detected = [r for r in rows if boolean(r.get("detected")) is True]
    rate = len(detected) / len(rows) * 100 if rows else 0
    checks.append(make_check(
        "Marker detection rate",
        PASS if rate >= 80 else WARNING if rate >= 50 else FAIL,
        f"{rate:.1f}%",
        f"{len(detected)} detections out of {len(rows)} updates.",
    ))

    refs = {str(r.get("reference", "")).strip().lower() for r in detected}
    refs.discard("")
    checks.append(make_check(
        "Landing reference",
        PASS if refs == {"original"} else FAIL,
        ", ".join(sorted(refs)) or "none",
        "Landing control should accept only original.",
    ))

    confidences = [number(r.get("confidence_percent")) for r in detected]
    confidences = [v for v in confidences if v is not None]
    avg_conf = statistics.mean(confidences) if confidences else 0
    checks.append(make_check(
        "Marker confidence",
        PASS if avg_conf >= min_confidence else WARNING,
        f"{avg_conf:.1f}%",
        f"Target is at least {min_confidence:.1f}%.",
    ))

    states = {str(r.get("state", "")).strip() for r in rows}
    state_ok = bool(states & {"ALIGNING", "DESCENDING", "HOLD_MIN_ALT"})
    checks.append(make_check(
        "Controller state progression",
        PASS if state_ok else WARNING,
        ", ".join(sorted(states)),
        "Expected ALIGNING, DESCENDING or HOLD_MIN_ALT.",
    ))

    commands = []
    for row in detected:
        vx = number(row.get("vx_mps"))
        vy = number(row.get("vy_mps"))
        vz = number(row.get("vz_mps"))
        if vx is not None and vy is not None and vz is not None:
            commands.append((vx, vy, vz))
    has_nonzero = any(abs(vx) + abs(vy) + abs(vz) > 0.001 for vx, vy, vz in commands)
    checks.append(make_check(
        "Velocity calculation",
        PASS if has_nonzero else WARNING,
        f"{len(commands)} valid rows",
        "At least one non-zero command should be calculated.",
    ))

    block_reasons = {
        str(r.get("block_reason", "")).strip()
        for r in rows
        if str(r.get("block_reason", "")).strip()
    }
    dry_ok = (
        any("disabled" in reason.lower() for reason in block_reasons)
        or all(boolean(r.get("command_sent")) is not True for r in rows)
    )
    checks.append(make_check(
        "Dry-run protection",
        PASS if dry_ok else FAIL,
        "commands blocked" if dry_ok else "real command observed",
        "; ".join(sorted(block_reasons)) or "No real command was recorded.",
    ))

    altitudes = [number(r.get("visual_alt_m")) for r in detected]
    altitudes = [v for v in altitudes if v is not None]
    checks.append(make_check(
        "Visual altitude",
        PASS if altitudes else FAIL,
        f"{statistics.mean(altitudes):.3f}m avg" if altitudes else "unavailable",
        "Visual altitude must be available.",
    ))
    return section_result("Landing controller", checks)


def height_validation_section(files: list[Path], max_mean: float, max_single: float) -> SectionResult:
    if not files:
        return section_result("Visual height validation", [
            make_check(
                "Height-test logs",
                NOT_TESTED,
                "no files",
                "Expected logs/height_test_*m.csv from the height-test script.",
            )
        ])

    checks: list[CheckResult] = []
    errors: list[float] = []
    details = []
    for path in sorted(files):
        try:
            real = float(path.stem.split("height_test_", 1)[1].removesuffix("m").replace("_", "."))
        except (IndexError, ValueError):
            continue
        rows = read_csv(path)
        values = [
            number(r.get("visual_alt_m"))
            for r in rows
            if boolean(r.get("detected")) is True
        ]
        values = [v for v in values if v is not None]
        if not values:
            continue
        avg = statistics.mean(values)
        err = abs(avg - real)
        errors.append(err)
        details.append(f"{real:.2f}m->{avg:.3f}m (err {err:.3f}m)")

    if not errors:
        checks.append(make_check(
            "Visual-height validation",
            FAIL,
            "no valid samples",
            "Height-test files contain no valid visual altitude rows.",
        ))
    else:
        mean_err = statistics.mean(errors)
        max_err = max(errors)
        checks.append(make_check(
            "Mean visual-height error",
            PASS if mean_err <= max_mean else WARNING,
            f"{mean_err:.3f}m",
            f"Limit: {max_mean:.3f}m.",
        ))
        checks.append(make_check(
            "Maximum visual-height error",
            PASS if max_err <= max_single else WARNING if max_err <= max_single * 1.5 else FAIL,
            f"{max_err:.3f}m",
            f"Limit: {max_single:.3f}m.",
        ))
        checks.append(make_check(
            "Validated heights",
            PASS if len(errors) >= 3 else WARNING,
            str(len(errors)),
            "; ".join(details),
        ))
    return section_result("Visual height validation", checks)


def direction_section(path: Optional[Path]) -> SectionResult:
    if path is None or not path.exists():
        return section_result("Horizontal direction test", [
            make_check(
                "X/Y direction confirmation",
                NOT_TESTED,
                "manual confirmation missing",
                'Create JSON: {"x_correct": true, "y_correct": true}.',
            )
        ])
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        x_ok = boolean(data.get("x_correct"))
        y_ok = boolean(data.get("y_correct"))
        checks = [
            make_check(
                "X direction",
                PASS if x_ok is True else FAIL if x_ok is False else NOT_TESTED,
                str(x_ok),
                str(data.get("x_notes", "")),
            ),
            make_check(
                "Y direction",
                PASS if y_ok is True else FAIL if y_ok is False else NOT_TESTED,
                str(y_ok),
                str(data.get("y_notes", "")),
            ),
        ]
    except (OSError, json.JSONDecodeError) as exc:
        checks = [make_check("Direction results file", FAIL, str(path), str(exc))]
    return section_result("Horizontal direction test", checks)


def hardware_section(profile: str) -> SectionResult:
    checks = [
        make_check(
            "Pixhawk MAVLink",
            WARNING,
            "verify from latest health run",
            "V18 does not duplicate the live MAVLink health monitor.",
        ),
        make_check(
            "RC / ELRS",
            NOT_TESTED,
            "pending transmitter test",
            "Required before first flight.",
        ),
        make_check(
            "GPS / Home Position",
            NOT_TESTED,
            "outdoor test pending",
            "Expected to remain unavailable indoors.",
        ),
        make_check(
            "Arming and GUIDED",
            NOT_TESTED,
            "prop-less bench test pending",
            "Do not test with propellers installed until prior checks pass.",
        ),
    ]
    return section_result("Flight controller and RC", checks)


def determine_overall(sections: list[SectionResult], profile: str):
    score = statistics.mean(s.score for s in sections) if sections else 0
    failures = []
    warnings = []
    actions = []
    for section in sections:
        for check in section.checks:
            if check.status == FAIL:
                failures.append(f"{section.name}: {check.name}")
                actions.append(f"Fix FAIL — {section.name}: {check.name}")
            elif check.status in {WARNING, NOT_TESTED}:
                warnings.append(f"{section.name}: {check.name}")
                actions.append(f"Complete/verify — {section.name}: {check.name}")

    if failures:
        return score, FAIL, "NOT READY", list(dict.fromkeys(actions))
    if profile == "prop-less":
        level = "READY FOR PROP-LESS TEST" if score >= 70 else "MORE VALIDATION REQUIRED"
        return score, WARNING if warnings else PASS, level, list(dict.fromkeys(actions))
    if profile == "first-flight":
        critical = [w for w in warnings if any(k in w for k in ("RC / ELRS", "GPS / Home Position", "Arming and GUIDED"))]
        level = "READY FOR FIRST MANUAL FLIGHT" if not critical and score >= 85 else "NOT YET READY FOR FIRST FLIGHT"
        return score, WARNING if critical else PASS, level, list(dict.fromkeys(actions))
    level = "READY FOR AUTONOMOUS LANDING TEST" if not warnings and score >= 90 else "NOT YET READY FOR AUTONOMOUS FLIGHT"
    return score, WARNING if warnings else PASS, level, list(dict.fromkeys(actions))


def render_text(report: ReadinessReport) -> str:
    lines = [
        "=" * 76,
        "AUTONOMOUS LANDING FLIGHT READINESS REPORT",
        "=" * 76,
        f"Generated: {report.generated_at}",
        f"Version:   {report.project_version}",
        f"Profile:   {report.profile}",
        "",
    ]
    symbols = {PASS: "[PASS]", WARNING: "[WARN]", FAIL: "[FAIL]", NOT_TESTED: "[N/T ]"}
    for index, section in enumerate(report.sections, 1):
        lines += [f"{index}. {section.name.upper()}", "-" * 76]
        for check in section.checks:
            value = f" — {check.value}" if check.value else ""
            lines.append(f"{symbols[check.status]} {check.name}{value}")
            if check.details:
                for detail in str(check.details).splitlines():
                    lines.append(f"       {detail}")
        lines += [f"Section result: {section.status} | Score: {section.score:.1f}%", ""]
    lines += [
        "=" * 76,
        "FINAL RESULT",
        "=" * 76,
        f"Overall score:  {report.overall_score:.1f}%",
        f"Overall status: {report.overall_status}",
        f"Readiness:      {report.readiness_level}",
        "",
        "ACTIONS REQUIRED",
        "-" * 76,
    ]
    lines += [f"- {a}" for a in report.actions_required] or ["- No unresolved checks."]
    lines += [
        "",
        "SAFETY NOTE",
        "-" * 76,
        "This report is one engineering input only. It does not replace normal",
        "preflight inspection, legal compliance, pilot supervision or a clear test area.",
        "=" * 76,
    ]
    return "\n".join(lines) + "\n"


def write_outputs(report: ReadinessReport, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    txt = output_dir / f"flight_readiness_report_{stamp}.txt"
    csv_path = output_dir / f"flight_readiness_report_{stamp}.csv"
    json_path = output_dir / f"flight_readiness_report_{stamp}.json"

    txt.write_text(render_text(report), encoding="utf-8")
    json_path.write_text(json.dumps(asdict(report), indent=2, ensure_ascii=False), encoding="utf-8")

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fields = [
            "section", "section_status", "section_score", "check",
            "check_status", "value", "details", "check_score",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for section in report.sections:
            for check in section.checks:
                writer.writerow({
                    "section": section.name,
                    "section_status": section.status,
                    "section_score": f"{section.score:.2f}",
                    "check": check.name,
                    "check_status": check.status,
                    "value": check.value,
                    "details": check.details,
                    "check_score": f"{check.score:.2f}",
                })
    return txt, csv_path, json_path


def main() -> int:
    args = parse_args()
    version_file = Path(args.version_file)
    version = version_file.read_text(encoding="utf-8").strip() if version_file.exists() else "unknown"
    expected = [float(x.strip()) for x in args.expected_heights.split(",") if x.strip()]
    landing_path = Path(args.landing_log) if args.landing_log else newest_file("logs/landing_debug_*.csv")
    direction_path = Path(args.direction_results) if args.direction_results else None
    height_files = list(Path(".").glob(args.height_test_glob))

    sections = [
        software_section(version_file),
        calibration_section(
            Path(args.calibration),
            Path(args.summary_calibration),
            expected,
            args.min_confidence,
        ),
        landing_section(landing_path, args.min_confidence),
        height_validation_section(
            height_files,
            args.max_height_mean_error_m,
            args.max_height_error_m,
        ),
        direction_section(direction_path),
        hardware_section(args.profile),
    ]

    overall_score, overall_status, level, actions = determine_overall(sections, args.profile)
    report = ReadinessReport(
        generated_at=datetime.now().isoformat(timespec="seconds"),
        project_version=version,
        profile=args.profile,
        sections=sections,
        overall_score=overall_score,
        overall_status=overall_status,
        readiness_level=level,
        actions_required=actions,
        source_files={
            "calibration": args.calibration,
            "summary_calibration": args.summary_calibration,
            "landing_log": str(landing_path) if landing_path else "",
            "height_test_glob": args.height_test_glob,
            "direction_results": str(direction_path) if direction_path else "",
        },
    )

    txt, csv_path, json_path = write_outputs(report, Path(args.output_dir))
    print(render_text(report))
    print(f"TXT report:  {txt}")
    print(f"CSV report:  {csv_path}")
    print(f"JSON report: {json_path}")
    return 2 if overall_status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
