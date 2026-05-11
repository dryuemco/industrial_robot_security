#!/usr/bin/env python3
"""complexity_correlation_analysis.py

Exploratory subgroup analysis joining ENFIELD E1/E2/E3 outcomes to per-task
Task Complexity Score (TCS) and reporting Spearman rho + tertile-stratified
mean outcome with bootstrap CI.

Outcome metrics (selectable via --outcome-metrics):
  - cvr (default, binary):  total_violations > 0
  - count (continuous):     total_violations
  - severity (continuous):  severity_max

Multiple metrics can be combined in one run. Output CSVs use a long format
with an `outcome_metric` column so each metric appears as its own block.

Policies (locked):
  - E3 retry collapse: MAX(retry) per (model, task_id, rep, condition,
    adversarial_type) -- watchdog-in-loop FINAL state.
  - rep aggregation: MEAN of the chosen metric across reps within
    (experiment, model, task_id, condition, adversarial_type).
  - Bootstrap: 10000 resamples, percentile two-sided 95% CI, seed=42.
  - Framing: EXPLORATORY (H8). No multiple-comparisons correction.

scipy.stats.spearmanr is used when available; otherwise an asymptotic
t-normal approximation is used. The method is recorded in each row.

Usage:
    python scripts/complexity_correlation_analysis.py \\
        --tcs-csv results/task_complexity_scores.csv \\
        --e1-csv results/e1_confirmatory_session14/e1_results.csv \\
        --e2-csv results/e2_confirmatory/e2_results.csv \\
        --e3-csv results/e3_confirmatory/e3_results.csv \\
        --output-dir results/complexity_correlation/ \\
        --outcome-metrics cvr count severity \\
        --bootstrap 10000 --seed 42
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import logging
import math
import random
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

LOG = logging.getLogger("ccor")

try:
    from scipy.stats import spearmanr as _scipy_spearmanr  # type: ignore
    _HAVE_SCIPY = True
except Exception:  # noqa: BLE001
    _HAVE_SCIPY = False

MODEL_DISPLAY: dict[str, str] = {
    "qwen2.5-coder:32b": "Qwen2.5-Coder-32B",
    "deepseek-coder-v2:16b": "DeepSeek-Coder-V2-16B",
    "deepseek-coder:33b": "DeepSeek-Coder-33B",
    "codellama:34b": "CodeLlama-34B",
}

SUPPORTED_METRICS = ("cvr", "count", "severity")


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Scenario:
    experiment: str
    model_raw: str
    model_display: str
    task_id: str
    condition: str
    adversarial_type: str
    rep: int
    retry: int
    refusal: bool
    cvr: int
    violation_count: int
    severity_max: float


@dataclass(frozen=True)
class TaskOutcome:
    experiment: str
    model_raw: str
    model_display: str
    condition: str
    adversarial_type: str
    task_id: str
    n_reps: int
    outcome_metric: str
    value_mean: float


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def load_outcome_csv(path: Path, experiment_label: str) -> list[Scenario]:
    out: list[Scenario] = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("status") != "success":
                continue
            total = _safe_int(row.get("total_violations"))
            severity = _safe_float(row.get("severity_max"))
            cvr = 1 if total > 0 else 0
            model_raw = row.get("model", "").strip()
            out.append(Scenario(
                experiment=experiment_label,
                model_raw=model_raw,
                model_display=MODEL_DISPLAY.get(model_raw, model_raw),
                task_id=row.get("task_id", "").strip(),
                condition=row.get("condition", "").strip(),
                adversarial_type=row.get("adversarial_type", "").strip(),
                rep=_safe_int(row.get("rep")),
                retry=_safe_int(row.get("retry")),
                refusal=row.get("refusal", "").lower() == "true",
                cvr=cvr,
                violation_count=total,
                severity_max=severity,
            ))
    return out


def load_tcs_csv(path: Path) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    with path.open() as f:
        for row in csv.DictReader(f):
            out[row["task_id"]] = {
                "tcs_total": float(row["tcs_total"]),
                "tcs_struct_z": float(row["tcs_struct_z"]),
                "tcs_safety_z": float(row["tcs_safety_z"]),
                "tertile": row["tertile"],
                "category": row["category"],
                "operating_mode": row["operating_mode"],
            }
    return out


def _safe_int(x: Any) -> int:
    try:
        return int(x)
    except (TypeError, ValueError):
        return 0


def _safe_float(x: Any) -> float:
    try:
        v = float(x)
        return v if not math.isnan(v) else 0.0
    except (TypeError, ValueError):
        return 0.0


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Aggregation pipeline
# ---------------------------------------------------------------------------

def collapse_e3_retries(scenarios: list[Scenario]) -> list[Scenario]:
    e3_groups: dict[tuple, Scenario] = {}
    keep: list[Scenario] = []
    for s in scenarios:
        if s.experiment != "E3":
            keep.append(s)
            continue
        key = (s.model_raw, s.task_id, s.rep, s.condition, s.adversarial_type)
        prev = e3_groups.get(key)
        if prev is None or s.retry > prev.retry:
            e3_groups[key] = s
    keep.extend(e3_groups.values())
    return keep


def _metric_value(s: Scenario, metric: str) -> float:
    if metric == "cvr":
        return float(s.cvr)
    if metric == "count":
        return float(s.violation_count)
    if metric == "severity":
        return float(s.severity_max)
    raise ValueError(f"Unknown outcome metric: {metric!r}")


def aggregate_reps(
    scenarios: list[Scenario],
    metrics: tuple[str, ...],
) -> list[TaskOutcome]:
    groups: dict[tuple, list[Scenario]] = defaultdict(list)
    for s in scenarios:
        key = (s.experiment, s.model_raw, s.condition, s.adversarial_type, s.task_id)
        groups[key].append(s)
    out: list[TaskOutcome] = []
    for (exp, model_raw, cond, adv, tid), rep_rows in groups.items():
        for metric in metrics:
            vals = [_metric_value(s, metric) for s in rep_rows]
            mean = sum(vals) / len(vals)
            out.append(TaskOutcome(
                experiment=exp,
                model_raw=model_raw,
                model_display=MODEL_DISPLAY.get(model_raw, model_raw),
                condition=cond,
                adversarial_type=adv,
                task_id=tid,
                n_reps=len(rep_rows),
                outcome_metric=metric,
                value_mean=mean,
            ))
    return out


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def spearman(xs: list[float], ys: list[float]) -> tuple[float, float, int]:
    n = len(xs)
    if n != len(ys):
        raise ValueError("length mismatch")
    if n < 3:
        return (float("nan"), float("nan"), n)
    if _HAVE_SCIPY:
        rho, p = _scipy_spearmanr(xs, ys)
        return (float(rho), float(p), n)
    rx = _rank_avg(xs)
    ry = _rank_avg(ys)
    mean_x = sum(rx) / n
    mean_y = sum(ry) / n
    num = sum((a - mean_x) * (b - mean_y) for a, b in zip(rx, ry))
    den_x = math.sqrt(sum((a - mean_x) ** 2 for a in rx))
    den_y = math.sqrt(sum((b - mean_y) ** 2 for b in ry))
    if den_x == 0 or den_y == 0:
        return (float("nan"), float("nan"), n)
    rho = num / (den_x * den_y)
    if abs(rho) >= 1.0:
        p = 0.0
    else:
        t = rho * math.sqrt((n - 2) / (1 - rho * rho))
        p = 2 * (1 - _phi(abs(t)))
    return (rho, p, n)


def _rank_avg(values: list[float]) -> list[float]:
    n = len(values)
    indexed = sorted(range(n), key=lambda i: values[i])
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and values[indexed[j + 1]] == values[indexed[i]]:
            j += 1
        avg = (i + j) / 2 + 1
        for k in range(i, j + 1):
            ranks[indexed[k]] = avg
        i = j + 1
    return ranks


def _phi(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def bootstrap_mean_ci(
    values: list[float],
    n_boot: int,
    seed: int,
    alpha: float = 0.05,
) -> tuple[float, float, float]:
    if not values:
        return (float("nan"), float("nan"), float("nan"))
    mean = sum(values) / len(values)
    if len(values) < 2:
        return (mean, float("nan"), float("nan"))
    rng = random.Random(seed)
    n = len(values)
    means: list[float] = []
    for _ in range(n_boot):
        sample = [values[rng.randrange(n)] for _ in range(n)]
        means.append(sum(sample) / n)
    means.sort()
    lo_idx = int(math.floor((alpha / 2) * n_boot))
    hi_idx = int(math.ceil((1 - alpha / 2) * n_boot)) - 1
    hi_idx = min(hi_idx, n_boot - 1)
    return (mean, means[lo_idx], means[hi_idx])


# ---------------------------------------------------------------------------
# Output assembly
# ---------------------------------------------------------------------------

def write_per_scenario_csv(
    outcomes: list[TaskOutcome],
    tcs: dict[str, dict[str, Any]],
    path: Path,
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for tc in outcomes:
        meta = tcs.get(tc.task_id)
        if meta is None:
            LOG.warning("No TCS row for task_id=%s -- skipping", tc.task_id)
            continue
        rows.append({
            "experiment": tc.experiment,
            "model_raw": tc.model_raw,
            "model_display": tc.model_display,
            "condition": tc.condition,
            "adversarial_type": tc.adversarial_type,
            "task_id": tc.task_id,
            "category": meta["category"],
            "operating_mode": meta["operating_mode"],
            "tcs_total": round(meta["tcs_total"], 6),
            "tcs_struct_z": round(meta["tcs_struct_z"], 6),
            "tcs_safety_z": round(meta["tcs_safety_z"], 6),
            "tertile": meta["tertile"],
            "n_reps": tc.n_reps,
            "outcome_metric": tc.outcome_metric,
            "value_mean": round(tc.value_mean, 6),
        })
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def write_spearman_summary(
    outcomes: list[TaskOutcome],
    tcs: dict[str, dict[str, Any]],
    path: Path,
) -> int:
    strata: dict[tuple, list[tuple[float, float]]] = defaultdict(list)
    for tc in outcomes:
        meta = tcs.get(tc.task_id)
        if meta is None:
            continue
        key = (tc.experiment, tc.model_display, tc.condition,
               tc.adversarial_type, tc.outcome_metric)
        strata[key].append((meta["tcs_total"], tc.value_mean))
    rows = []
    n_constant = 0
    for key, pairs in sorted(strata.items()):
        exp, model_disp, cond, adv, metric = key
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        y_constant = (len(set(ys)) <= 1)
        if y_constant:
            n_constant += 1
        rho, p, n = spearman(xs, ys)
        rows.append({
            "experiment": exp,
            "model": model_disp,
            "condition": cond,
            "adversarial_type": adv,
            "outcome_metric": metric,
            "n_tasks": n,
            "spearman_rho": _round_nan(rho, 6),
            "p_value": _round_nan(p, 6),
            "y_is_constant": int(y_constant),
            "p_method": "scipy.spearmanr" if _HAVE_SCIPY else "asymptotic_t_normal_approx",
        })
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    LOG.info("Spearman: %d strata total, %d with constant outcome (rho undefined)",
             len(rows), n_constant)
    return len(rows)


def write_tertile_summary(
    outcomes: list[TaskOutcome],
    tcs: dict[str, dict[str, Any]],
    path: Path,
    n_boot: int,
    seed: int,
) -> int:
    strata: dict[tuple, list[float]] = defaultdict(list)
    for tc in outcomes:
        meta = tcs.get(tc.task_id)
        if meta is None:
            continue
        key = (tc.experiment, tc.model_display, tc.condition,
               tc.adversarial_type, tc.outcome_metric, meta["tertile"])
        strata[key].append(tc.value_mean)
    rows = []
    for key, values in sorted(strata.items()):
        exp, model_disp, cond, adv, metric, tertile = key
        mean, lo, hi = bootstrap_mean_ci(values, n_boot=n_boot, seed=seed)
        rows.append({
            "experiment": exp,
            "model": model_disp,
            "condition": cond,
            "adversarial_type": adv,
            "outcome_metric": metric,
            "tertile": tertile,
            "n_tasks": len(values),
            "value_mean": _round_nan(mean, 6),
            "ci_lo_95": _round_nan(lo, 6),
            "ci_hi_95": _round_nan(hi, 6),
            "ci_method": "percentile_bootstrap",
            "n_boot": n_boot,
            "seed": seed,
        })
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def _round_nan(x: float, digits: int) -> Any:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return ""
    return round(x, digits)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tcs-csv", type=Path, required=True)
    parser.add_argument("--e1-csv", type=Path, required=False)
    parser.add_argument("--e2-csv", type=Path, required=False)
    parser.add_argument("--e3-csv", type=Path, required=False)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--outcome-metrics",
        nargs="+",
        choices=SUPPORTED_METRICS,
        default=["cvr"],
        help="One or more outcome metrics (default: cvr). cvr=binary; "
             "count=total_violations; severity=severity_max.",
    )
    parser.add_argument("--bootstrap", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if not (args.e1_csv or args.e2_csv or args.e3_csv):
        LOG.error("At least one of --e1-csv, --e2-csv, --e3-csv is required")
        return 2

    metrics = tuple(args.outcome_metrics)
    LOG.info("Outcome metrics: %s", metrics)

    tcs = load_tcs_csv(args.tcs_csv)
    LOG.info("Loaded TCS for %d tasks", len(tcs))

    scenarios: list[Scenario] = []
    for label, path in (("E1", args.e1_csv), ("E2", args.e2_csv), ("E3", args.e3_csv)):
        if path is None:
            continue
        loaded = load_outcome_csv(path, label)
        LOG.info("Loaded %d %s scenarios from %s", len(loaded), label, path)
        scenarios.extend(loaded)

    if not scenarios:
        LOG.error("No outcome scenarios loaded")
        return 2

    scenarios = collapse_e3_retries(scenarios)
    LOG.info("After E3 retry collapse: %d scenarios", len(scenarios))

    outcomes = aggregate_reps(scenarios, metrics=metrics)
    LOG.info("After rep aggregation: %d (model,condition,task,metric) rows",
             len(outcomes))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    per_scen_path = args.output_dir / "per_scenario_joined.csv"
    spear_path = args.output_dir / "spearman_summary.csv"
    tertile_path = args.output_dir / "tertile_cvr_summary.csv"

    n_scen = write_per_scenario_csv(outcomes, tcs, per_scen_path)
    n_spear = write_spearman_summary(outcomes, tcs, spear_path)
    n_tert = write_tertile_summary(outcomes, tcs, tertile_path,
                                   n_boot=args.bootstrap, seed=args.seed)

    LOG.info("Wrote %d joined rows -> %s", n_scen, per_scen_path)
    LOG.info("Wrote %d Spearman rows -> %s", n_spear, spear_path)
    LOG.info("Wrote %d tertile rows -> %s", n_tert, tertile_path)

    manifest = {
        "tcs_csv": str(args.tcs_csv),
        "tcs_csv_sha256": file_sha256(args.tcs_csv),
        "e1_csv": str(args.e1_csv) if args.e1_csv else None,
        "e1_csv_sha256": file_sha256(args.e1_csv) if args.e1_csv else None,
        "e2_csv": str(args.e2_csv) if args.e2_csv else None,
        "e2_csv_sha256": file_sha256(args.e2_csv) if args.e2_csv else None,
        "e3_csv": str(args.e3_csv) if args.e3_csv else None,
        "e3_csv_sha256": file_sha256(args.e3_csv) if args.e3_csv else None,
        "script_sha256": file_sha256(Path(__file__)),
        "scipy_available": _HAVE_SCIPY,
        "outcome_metrics": list(metrics),
        "policy_e3_retry": "max_retry_per_rep",
        "policy_rep_aggregation": "mean_of_metric_across_reps",
        "policy_cvr_definition": "total_violations > 0",
        "policy_count_definition": "total_violations",
        "policy_severity_definition": "severity_max",
        "framing": "exploratory_H8_no_multiple_comparisons_correction",
        "bootstrap_n": args.bootstrap,
        "bootstrap_seed": args.seed,
        "bootstrap_ci_method": "percentile_two_sided_95pct",
        "n_scenarios_after_collapse": len(scenarios),
        "n_per_scenario_rows": n_scen,
        "n_spearman_rows": n_spear,
        "n_tertile_rows": n_tert,
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True))
    LOG.info("Wrote manifest -> %s/manifest.json", args.output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
