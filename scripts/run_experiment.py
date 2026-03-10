# Copyright 2026 Yunus Emre Cogurcu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ENFIELD Experiment Runner — static watchdog analysis on baselines + variants.

Produces structured CSV and JSON reports suitable for statistical analysis
(McNemar test, Holm-Bonferroni correction) as specified in the OSF
pre-registration draft.

Usage:
    python3 scripts/run_experiment.py \\
        --baselines enfield_tasks/ir/tasks \\
        --variants enfield_attacks/generated/variants \\
        --output results/

Outputs:
    results/verdicts.csv          — per-scenario pass/fail with attack metadata
    results/summary.json          — aggregate statistics
    results/detection_matrix.csv  — attack × task detection matrix
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Allow running from repo root with PYTHONPATH set
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from enfield_watchdog_static.watchdog import StaticWatchdog
from enfield_watchdog_static.violation import WatchdogReport

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_VARIANT_RE = re.compile(r"^(T\d{3})_(A\d)_(.+)\.json$")
_BASELINE_RE = re.compile(r"^(T\d{3})_(.+)\.json$")


def _classify_file(filename: str) -> dict[str, str]:
    """Extract task_id, attack_type, and role from a filename."""
    m = _VARIANT_RE.match(filename)
    if m:
        return {
            "task_id": m.group(1),
            "attack_type": m.group(2),
            "role": "variant",
            "task_name": m.group(3),
        }
    m = _BASELINE_RE.match(filename)
    if m:
        return {
            "task_id": m.group(1),
            "attack_type": "none",
            "role": "baseline",
            "task_name": m.group(2),
        }
    return {
        "task_id": "?",
        "attack_type": "?",
        "role": "unknown",
        "task_name": filename,
    }


def _report_to_row(
    report: WatchdogReport,
    file_meta: dict[str, str],
) -> dict[str, Any]:
    """Convert a WatchdogReport to a flat CSV-compatible dict."""
    detected_attacks = sorted({v.attack_type for v in report.violations})
    return {
        "file": report.source_file,
        "task_id": file_meta["task_id"],
        "role": file_meta["role"],
        "injected_attack": file_meta["attack_type"],
        "operating_mode": "",  # filled below
        "safe": report.safe,
        "violation_count": report.violation_count,
        "max_severity": round(report.max_severity(), 3),
        "detected_attacks": ",".join(detected_attacks),
        "checks_run": report.checks_run,
        "a1_detected": report.has_attack("A1"),
        "a2_detected": report.has_attack("A2"),
        "a3_detected": report.has_attack("A3"),
        "a4_detected": report.has_attack("A4"),
        "a5_detected": report.has_attack("A5"),
        "a6_detected": report.has_attack("A6"),
        "a7_detected": report.has_attack("A7"),
        "a8_detected": report.has_attack("A8"),
    }


# ---------------------------------------------------------------------------
# Main experiment logic
# ---------------------------------------------------------------------------


def run_experiment(
    baselines_dir: Path,
    variants_dir: Path,
    output_dir: Path,
) -> dict[str, Any]:
    """Run the full static watchdog experiment.

    Args:
        baselines_dir: Directory with baseline T00*.json files.
        variants_dir: Directory with adversarial variant T*_A*_*.json files.
        output_dir: Directory for CSV/JSON output.

    Returns:
        Summary statistics dict.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    wd = StaticWatchdog()
    rows: list[dict[str, Any]] = []

    # --- Analyze baselines ---
    logger.info("=== Baselines ===")
    baseline_files = sorted(baselines_dir.glob("T[0-9][0-9][0-9]_*.json"))
    for bf in baseline_files:
        with open(bf) as f:
            task = json.load(f)
        report = wd.analyze(task, source_file=bf.name)
        meta = _classify_file(bf.name)
        row = _report_to_row(report, meta)
        row["operating_mode"] = task.get("task", {}).get("operating_mode", "")
        rows.append(row)
        status = "PASS" if report.safe else f"FAIL ({report.violation_count})"
        logger.info("  %s: %s", bf.name, status)

    # --- Analyze variants ---
    logger.info("\n=== Adversarial Variants ===")
    variant_files = sorted(variants_dir.glob("T*_A*_*.json"))
    for vf in variant_files:
        if vf.name == "manifest.json":
            continue
        with open(vf) as f:
            task = json.load(f)
        report = wd.analyze(task, source_file=vf.name)
        meta = _classify_file(vf.name)
        row = _report_to_row(report, meta)
        row["operating_mode"] = task.get("task", {}).get("operating_mode", "")
        rows.append(row)
        status = "PASS" if report.safe else f"FAIL ({report.violation_count})"
        logger.info("  %s: %s [injected: %s]", vf.name, status, meta["attack_type"])

    # --- Write CSV ---
    csv_path = output_dir / "verdicts.csv"
    fieldnames = [
        "file", "task_id", "role", "injected_attack", "operating_mode",
        "safe", "violation_count", "max_severity", "detected_attacks",
        "checks_run",
        "a1_detected", "a2_detected", "a3_detected", "a4_detected",
        "a5_detected", "a6_detected", "a7_detected", "a8_detected",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    logger.info("\nCSV: %s (%d rows)", csv_path, len(rows))

    # --- Compute summary ---
    summary = _compute_summary(rows)
    summary["metadata"] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "baselines_dir": str(baselines_dir),
        "variants_dir": str(variants_dir),
        "baseline_count": len(baseline_files),
        "variant_count": len(variant_files),
        "total_scenarios": len(rows),
    }

    json_path = output_dir / "summary.json"
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    logger.info("JSON: %s", json_path)

    # --- Detection matrix CSV ---
    matrix_path = output_dir / "detection_matrix.csv"
    _write_detection_matrix(rows, matrix_path)
    logger.info("Matrix: %s", matrix_path)

    return summary


def _compute_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Compute aggregate statistics from verdict rows."""
    baselines = [r for r in rows if r["role"] == "baseline"]
    variants = [r for r in rows if r["role"] == "variant"]

    # Baseline stats
    baseline_safe = sum(1 for r in baselines if r["safe"])
    baseline_fp = len(baselines) - baseline_safe

    # Variant stats
    variant_flagged = sum(1 for r in variants if not r["safe"])
    variant_missed = len(variants) - variant_flagged

    # Per-attack detection rate
    attack_stats: dict[str, dict[str, int]] = {}
    for aid in ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"]:
        attack_variants = [r for r in variants if r["injected_attack"] == aid]
        detected = sum(
            1 for r in attack_variants if r[f"{aid.lower()}_detected"]
        )
        total = len(attack_variants)
        attack_stats[aid] = {
            "total": total,
            "detected": detected,
            "detection_rate": round(detected / total, 3) if total > 0 else 0.0,
            "missed": total - detected,
        }

    # Overall metrics
    total_variants = len(variants)
    overall_detection_rate = (
        variant_flagged / total_variants if total_variants > 0 else 0.0
    )
    false_positive_rate = (
        baseline_fp / len(baselines) if len(baselines) > 0 else 0.0
    )

    return {
        "baselines": {
            "total": len(baselines),
            "safe": baseline_safe,
            "false_positives": baseline_fp,
            "false_positive_rate": round(false_positive_rate, 3),
        },
        "variants": {
            "total": total_variants,
            "flagged": variant_flagged,
            "missed": variant_missed,
            "overall_detection_rate": round(overall_detection_rate, 3),
        },
        "per_attack": attack_stats,
    }


def _write_detection_matrix(
    rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Write attack × task detection matrix as CSV."""
    variants = [r for r in rows if r["role"] == "variant"]

    task_ids = sorted({r["task_id"] for r in variants})
    attacks = ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"]

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["attack"] + task_ids + ["detection_rate"])

        for aid in attacks:
            row_data = [aid]
            detected_count = 0
            total_count = 0
            for tid in task_ids:
                match = [
                    r for r in variants
                    if r["task_id"] == tid and r["injected_attack"] == aid
                ]
                if match:
                    total_count += 1
                    hit = match[0][f"{aid.lower()}_detected"]
                    row_data.append("1" if hit else "0")
                    if hit:
                        detected_count += 1
                else:
                    row_data.append("-")
            rate = (
                f"{detected_count / total_count:.0%}"
                if total_count > 0 else "N/A"
            )
            row_data.append(rate)
            writer.writerow(row_data)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    """CLI entry point for experiment runner."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="ENFIELD Experiment Runner — static watchdog analysis.",
    )
    parser.add_argument(
        "--baselines",
        type=str,
        default="enfield_tasks/ir/tasks",
        help="Directory with baseline T00*.json files.",
    )
    parser.add_argument(
        "--variants",
        type=str,
        default="enfield_attacks/generated/variants",
        help="Directory with adversarial variant files.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="results",
        help="Output directory for CSV/JSON reports.",
    )

    args = parser.parse_args()
    summary = run_experiment(
        baselines_dir=Path(args.baselines),
        variants_dir=Path(args.variants),
        output_dir=Path(args.output),
    )

    # Print summary
    print("\n" + "=" * 60)
    print("ENFIELD Static Watchdog — Experiment Results")
    print("=" * 60)

    b = summary["baselines"]
    v = summary["variants"]
    print(f"\nBaselines: {b['safe']}/{b['total']} safe "
          f"(FP rate: {b['false_positive_rate']:.1%})")
    print(f"Variants:  {v['flagged']}/{v['total']} flagged "
          f"(detection rate: {v['overall_detection_rate']:.1%})")

    print("\nPer-attack detection:")
    for aid, stats in summary["per_attack"].items():
        bar = "█" * stats["detected"] + "░" * stats["missed"]
        print(f"  {aid}: {stats['detected']}/{stats['total']} "
              f"({stats['detection_rate']:.0%}) {bar}")

    print(f"\nOutputs: {args.output}/verdicts.csv, "
          f"summary.json, detection_matrix.csv")


if __name__ == "__main__":
    main()
