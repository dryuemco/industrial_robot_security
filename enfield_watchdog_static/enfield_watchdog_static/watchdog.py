# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Static Watchdog — IR-level A1–A8 rule checker for Task IR files."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from enfield_watchdog_static.rules import ALL_RULES
from enfield_watchdog_static.violation import Violation, WatchdogReport

logger = logging.getLogger(__name__)


class StaticWatchdog:
    """Run A1–A8 static detection rules against Task IR dicts.

    Usage:
        wd = StaticWatchdog()
        report = wd.analyze(task_dict)
        print(report.summary())
    """

    def __init__(
        self,
        rules: dict[str, Any] | None = None,
        enabled_attacks: list[str] | None = None,
    ) -> None:
        """Initialize the watchdog.

        Args:
            rules: Override rule registry (default: ALL_RULES).
            enabled_attacks: Subset of attack IDs to check (default: all A1–A8).
        """
        self._rules = rules or dict(ALL_RULES)
        self._enabled = set(enabled_attacks) if enabled_attacks else set(self._rules.keys())

    def analyze(self, task: dict[str, Any], source_file: str = "") -> WatchdogReport:
        """Analyze a single Task IR dict.

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
        description="ENFIELD Static Watchdog — A1–A8 rule checker for Task IR."
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
        "--pattern",
        type=str,
        default="*.json",
        help="Glob pattern for directory mode.",
    )

    args = parser.parse_args()
    wd = StaticWatchdog(enabled_attacks=args.attacks)
    input_path = Path(args.input)

    if input_path.is_dir():
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
