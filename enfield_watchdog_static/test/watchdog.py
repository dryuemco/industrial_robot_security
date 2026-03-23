# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Static Watchdog — IR-level A1–A8 rule checker + code-level SM-1–SM-7 security checker."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from enfield_watchdog_static.rules import ALL_RULES, ALL_SECURITY_RULES
from enfield_watchdog_static.violation import Violation, WatchdogReport

logger = logging.getLogger(__name__)


class StaticWatchdog:
    """Run A1–A8 safety rules (on Task IR) and SM-1–SM-7 security rules (on URScript code).

    Usage:
        wd = StaticWatchdog()

        # Safety analysis on Task IR dict
        report = wd.analyze(task_dict)

        # Security analysis on URScript code string
        report = wd.analyze_code(urscript_code, task_id="T001")

        # Combined analysis (Task IR + generated code)
        report = wd.analyze_combined(task_dict, urscript_code)
    """

    def __init__(
        self,
        rules: dict[str, Any] | None = None,
        security_rules: dict[str, Any] | None = None,
        enabled_attacks: list[str] | None = None,
        enabled_security: list[str] | None = None,
    ) -> None:
        """Initialize the watchdog.

        Args:
            rules: Override safety rule registry (default: ALL_RULES).
            security_rules: Override security rule registry (default: ALL_SECURITY_RULES).
            enabled_attacks: Subset of attack IDs to check (default: all A1–A8).
            enabled_security: Subset of security IDs to check (default: all SM-1–SM-7).
        """
        self._rules = rules or dict(ALL_RULES)
        self._security_rules = security_rules or dict(ALL_SECURITY_RULES)
        self._enabled = set(enabled_attacks) if enabled_attacks else set(self._rules.keys())
        self._enabled_security = (
            set(enabled_security) if enabled_security
            else set(self._security_rules.keys())
        )

    # -----------------------------------------------------------------
    # Safety analysis (Task IR level) — existing functionality
    # -----------------------------------------------------------------

    def analyze(self, task: dict[str, Any], source_file: str = "") -> WatchdogReport:
        """Analyze a single Task IR dict for safety violations (A1–A8).

        Args:
            task: Parsed JSON task dictionary.
            source_file: Optional filename for the report.

        Returns:
            WatchdogReport with all detected violations.
        """
        task_id = task.get("task", {}).get("id", "?")
        report = WatchdogReport(task_id=task_id, source_file=source_file)

        for attack_id in sorted(self._enabled):
            rule_fn = self._rules.get(attack_id)
            if rule_fn is None:
                continue

            try:
                violations = rule_fn(task)
                report.violations.extend(violations)
                report.checks_run += 1
            except Exception as exc:
                logger.error("Rule %s failed on %s: %s", attack_id, task_id, exc)
                report.checks_run += 1

        return report

    # -----------------------------------------------------------------
    # Security analysis (URScript code level) — NEW
    # -----------------------------------------------------------------

    def analyze_code(
        self,
        code: str,
        task_id: str = "?",
        source_file: str = "",
    ) -> WatchdogReport:
        """Analyze URScript code for security violations (SM-1–SM-7).

        Args:
            code: URScript source code string.
            task_id: Task identifier for the report.
            source_file: Optional source filename.

        Returns:
            WatchdogReport with detected security violations.
        """
        report = WatchdogReport(task_id=task_id, source_file=source_file)

        for rule_id in sorted(self._enabled_security):
            rule_fn = self._security_rules.get(rule_id)
            if rule_fn is None:
                continue

            try:
                violations = rule_fn(code)
                report.violations.extend(violations)
                report.checks_run += 1
            except Exception as exc:
                logger.error("Security rule %s failed on %s: %s",
                             rule_id, task_id, exc)
                report.checks_run += 1

        return report

    # -----------------------------------------------------------------
    # Combined analysis (Task IR + code) — NEW
    # -----------------------------------------------------------------

    def analyze_combined(
        self,
        task: dict[str, Any],
        code: str,
        source_file: str = "",
    ) -> WatchdogReport:
        """Run both safety (A1–A8) and security (SM-1–SM-7) analysis.

        Args:
            task: Parsed JSON task dictionary.
            code: URScript source code string.
            source_file: Optional source filename.

        Returns:
            Single WatchdogReport with safety + security violations.
        """
        # Safety analysis
        safety_report = self.analyze(task, source_file=source_file)

        # Security analysis
        task_id = task.get("task", {}).get("id", "?")
        security_report = self.analyze_code(code, task_id=task_id)

        # Merge into one report
        safety_report.violations.extend(security_report.violations)
        safety_report.checks_run += security_report.checks_run

        return safety_report

    # -----------------------------------------------------------------
    # File and batch operations — existing functionality
    # -----------------------------------------------------------------

    def analyze_file(self, filepath: str | Path) -> WatchdogReport:
        """Load a JSON file and analyze it.

        Args:
            filepath: Path to .json Task IR file.

        Returns:
            WatchdogReport.
        """
        path = Path(filepath)
        with open(path) as f:
            task = json.load(f)
        return self.analyze(task, source_file=path.name)

    def batch_analyze(
        self,
        directory: str | Path,
        pattern: str = "*.json",
    ) -> list[WatchdogReport]:
        """Analyze all matching JSON files in a directory.

        Args:
            directory: Directory path.
            pattern: Glob pattern.

        Returns:
            List of WatchdogReport objects.
        """
        dir_path = Path(directory)
        reports: list[WatchdogReport] = []

        for jf in sorted(dir_path.glob(pattern)):
            if jf.name == "manifest.json":
                continue
            try:
                report = self.analyze_file(jf)
                reports.append(report)
                status = "PASS" if report.safe else f"FAIL ({report.violation_count})"
                logger.info("  %s: %s", jf.name, status)
            except Exception as exc:
                logger.error("  %s: ERROR — %s", jf.name, exc)

        return reports

    def batch_report_json(
        self,
        reports: list[WatchdogReport],
        output_path: str | Path,
    ) -> Path:
        """Write batch analysis results to a JSON file.

        Args:
            reports: List of WatchdogReport.
            output_path: Output JSON path.

        Returns:
            Path to written file.
        """
        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "total_files": len(reports),
            "safe_count": sum(1 for r in reports if r.safe),
            "violation_count": sum(r.violation_count for r in reports),
            "reports": [r.to_dict() for r in reports],
        }

        with open(out, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        logger.info(
            "Report: %s (%d files, %d safe, %d violations)",
            out, data["total_files"], data["safe_count"], data["violation_count"],
        )
        return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    """CLI: run static watchdog on Task IR files."""
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(
        description="ENFIELD Static Watchdog — A1–A8 safety + SM-1–SM-7 security checker."
    )
    parser.add_argument(
        "input",
        type=str,
        help="JSON file or directory to analyze.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output JSON report path (default: stdout summary).",
    )
    parser.add_argument(
        "--attacks",
        type=str,
        nargs="+",
        default=None,
        help="Subset of attack IDs (e.g. A1 A3 A5). Default: all.",
    )
    parser.add_argument(
        "--security",
        type=str,
        nargs="+",
        default=None,
        help="Subset of security IDs (e.g. SM-1 SM-3). Default: all.",
    )
    parser.add_argument(
        "--code",
        type=str,
        default=None,
        help="URScript file to analyze for security violations.",
    )
    parser.add_argument(
        "--pattern",
        type=str,
        default="*.json",
        help="Glob pattern for directory mode.",
    )

    args = parser.parse_args()
    wd = StaticWatchdog(
        enabled_attacks=args.attacks,
        enabled_security=args.security,
    )
    input_path = Path(args.input)

    if args.code:
        # Security-only analysis on URScript file
        code_path = Path(args.code)
        code = code_path.read_text()
        report = wd.analyze_code(code, task_id=code_path.stem)
        print(report.summary())
    elif input_path.is_dir():
        reports = wd.batch_analyze(input_path, pattern=args.pattern)
        for r in reports:
            print(r.summary())
            print()
        if args.output:
            wd.batch_report_json(reports, args.output)
    else:
        report = wd.analyze_file(input_path)
        print(report.summary())
        if args.output:
            out = Path(args.output)
            out.parent.mkdir(parents=True, exist_ok=True)
            with open(out, "w") as f:
                json.dump(report.to_dict(), f, indent=2)


if __name__ == "__main__":
    main()
