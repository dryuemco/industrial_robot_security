"""Tests for complexity_correlation_analysis (multi-metric).

Invariants enforced:
  1. CVR definition: total_violations > 0 -> 1, else 0.
  2. Count metric: equals total_violations.
  3. Severity metric: equals severity_max.
  4. E3 retry collapse: keeps MAX(retry) row per (model, task, rep, condition).
  5. E1/E2 pass through retry collapse unchanged.
  6. Rep aggregation: MEAN across reps for each metric independently.
  7. Spearman: perfect monotonic ascending -> rho = 1.0.
  8. Spearman: perfect monotonic descending -> rho = -1.0.
  9. Spearman: tied ranks handled via average ranks.
 10. Spearman: too few points -> nan.
 11. Bootstrap: deterministic given seed; single-value edge case handled.
 12. Bootstrap: different seeds yield differing CI bounds on diverse samples.
 13. Bootstrap: CI brackets the mean.
 14. Model normalization: known tags mapped; unknown passthrough.
 15. Refusal rows loaded with cvr=0 when no violations.
 16. End-to-end synthetic: monotonic TCS<->CVR yields rho ~ 1, correct tertile order.
 17. End-to-end E3 collapse propagates through join.
 18. Multi-metric: cvr + count + severity all produced.
 19. Count metric reveals signal where CVR is constant (ceiling effect bypass).
 20. y_is_constant column flags strata where Spearman is undefined.
 21. Unknown metric raises ValueError.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from scripts.complexity_correlation_analysis import (  # type: ignore
    MODEL_DISPLAY,
    SUPPORTED_METRICS,
    Scenario,
    TaskOutcome,
    _metric_value,
    aggregate_reps,
    bootstrap_mean_ci,
    collapse_e3_retries,
    load_outcome_csv,
    load_tcs_csv,
    spearman,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

OUTCOME_HEADER = (
    "experiment,model,task_id,task_category,operating_mode,condition,"
    "adversarial_type,rep,retry,status,refusal,code_lines,has_motion,"
    "has_safety_check,dm_violations,sm_violations,total_violations,"
    "violation_types,severity_max,tokens_in,tokens_out,latency_ms,timestamp"
)


def _row(experiment, model, task_id, condition, adv_type, rep, retry,
         total_violations, status="success", refusal="False",
         dm=0, sm=None, severity=1.0):
    if sm is None:
        sm = total_violations
    return (
        f"{experiment},{model},{task_id},pick_place,collaborative,{condition},"
        f"{adv_type},{rep},{retry},{status},{refusal},19,True,False,{dm},{sm},"
        f"{total_violations},\"SM-1\",{severity},268,428,15000,2026-04-15T00:00:00+00:00"
    )


def _write_outcome_csv(path: Path, rows: list[str]) -> None:
    path.write_text("\n".join([OUTCOME_HEADER] + rows) + "\n")


def _write_tcs_csv(path: Path, rows: list[dict]) -> None:
    header = list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)


def _mk_scenario(experiment="E1", model="m", task="T001", cond="baseline",
                 adv="", rep=1, retry=0, cvr=0, count=0, sev=0.0,
                 refusal=False):
    return Scenario(
        experiment=experiment, model_raw=model,
        model_display=MODEL_DISPLAY.get(model, model),
        task_id=task, condition=cond, adversarial_type=adv,
        rep=rep, retry=retry, refusal=refusal,
        cvr=cvr, violation_count=count, severity_max=sev,
    )


# ---------------------------------------------------------------------------
# 1-3. Metric value extraction
# ---------------------------------------------------------------------------

def test_cvr_from_total_violations(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 1, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 2, 0, 7),
    ])
    scens = load_outcome_csv(p, "E1")
    assert [s.cvr for s in scens] == [0, 1]
    assert [s.violation_count for s in scens] == [0, 7]


def test_metric_value_extraction():
    s = _mk_scenario(cvr=1, count=5, sev=0.7)
    assert _metric_value(s, "cvr") == 1.0
    assert _metric_value(s, "count") == 5.0
    assert _metric_value(s, "severity") == 0.7


def test_unknown_metric_raises():
    s = _mk_scenario()
    with pytest.raises(ValueError):
        _metric_value(s, "totally_made_up")


def test_severity_loaded(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "m", "T001", "baseline", "", 1, 0, 3, severity=0.85),
    ])
    scens = load_outcome_csv(p, "E1")
    assert scens[0].severity_max == 0.85
    assert scens[0].violation_count == 3


def test_status_nonsuccess_skipped(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "m", "T001", "baseline", "", 1, 0, 5),
        _row("E1", "m", "T001", "baseline", "", 2, 0, 0, status="error"),
    ])
    scens = load_outcome_csv(p, "E1")
    assert len(scens) == 1


# ---------------------------------------------------------------------------
# 4-5. E3 retry collapse
# ---------------------------------------------------------------------------

def test_e3_collapse_keeps_max_retry():
    rows = [
        _mk_scenario(experiment="E3", rep=1, retry=0, cvr=1, count=5),
        _mk_scenario(experiment="E3", rep=1, retry=2, cvr=0, count=0),
        _mk_scenario(experiment="E3", rep=1, retry=1, cvr=1, count=3),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 1
    assert out[0].retry == 2
    assert out[0].cvr == 0
    assert out[0].violation_count == 0


def test_e3_collapse_keeps_distinct_reps():
    rows = [
        _mk_scenario(experiment="E3", rep=1, retry=0, cvr=1),
        _mk_scenario(experiment="E3", rep=1, retry=2, cvr=0),
        _mk_scenario(experiment="E3", rep=2, retry=0, cvr=1),
        _mk_scenario(experiment="E3", rep=2, retry=1, cvr=1),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 2
    assert {s.rep for s in out} == {1, 2}


def test_e1_e2_pass_through_collapse():
    rows = [
        _mk_scenario(experiment="E1", rep=1, cvr=0),
        _mk_scenario(experiment="E1", rep=2, cvr=1),
        _mk_scenario(experiment="E2", rep=1, cvr=1),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 3


# ---------------------------------------------------------------------------
# 6. Rep aggregation (multi-metric)
# ---------------------------------------------------------------------------

def test_rep_aggregation_cvr_mean():
    rows = [
        _mk_scenario(rep=1, cvr=0, count=0, sev=0.0),
        _mk_scenario(rep=2, cvr=1, count=4, sev=0.8),
        _mk_scenario(rep=3, cvr=0, count=0, sev=0.0),
    ]
    out = aggregate_reps(rows, metrics=("cvr",))
    assert len(out) == 1
    assert out[0].outcome_metric == "cvr"
    assert abs(out[0].value_mean - 1 / 3) < 1e-9


def test_rep_aggregation_count_mean():
    rows = [
        _mk_scenario(rep=1, count=4),
        _mk_scenario(rep=2, count=8),
        _mk_scenario(rep=3, count=6),
    ]
    out = aggregate_reps(rows, metrics=("count",))
    assert out[0].value_mean == 6.0


def test_rep_aggregation_severity_mean():
    rows = [
        _mk_scenario(rep=1, sev=0.3),
        _mk_scenario(rep=2, sev=0.9),
    ]
    out = aggregate_reps(rows, metrics=("severity",))
    assert out[0].value_mean == 0.6


def test_rep_aggregation_multi_metric():
    rows = [
        _mk_scenario(rep=1, cvr=0, count=0, sev=0.0),
        _mk_scenario(rep=2, cvr=1, count=4, sev=0.8),
        _mk_scenario(rep=3, cvr=1, count=2, sev=0.5),
    ]
    out = aggregate_reps(rows, metrics=("cvr", "count", "severity"))
    by_metric = {o.outcome_metric: o for o in out}
    assert set(by_metric) == {"cvr", "count", "severity"}
    assert abs(by_metric["cvr"].value_mean - 2 / 3) < 1e-9
    assert by_metric["count"].value_mean == 2.0
    assert abs(by_metric["severity"].value_mean - 1.3 / 3) < 1e-9


def test_rep_aggregation_splits_by_condition():
    rows = [
        _mk_scenario(rep=1, cond="baseline", cvr=0),
        _mk_scenario(rep=1, cond="safety", cvr=1),
    ]
    out = aggregate_reps(rows, metrics=("cvr",))
    assert len(out) == 2
    by_cond = {x.condition: x for x in out}
    assert by_cond["baseline"].value_mean == 0.0
    assert by_cond["safety"].value_mean == 1.0


# ---------------------------------------------------------------------------
# 7-10. Spearman
# ---------------------------------------------------------------------------

def test_spearman_perfect_ascending():
    rho, p, n = spearman([1.0, 2.0, 3.0, 4.0, 5.0], [10.0, 20.0, 30.0, 40.0, 50.0])
    assert abs(rho - 1.0) < 1e-9
    assert n == 5


def test_spearman_perfect_descending():
    rho, p, n = spearman([1.0, 2.0, 3.0, 4.0, 5.0], [50.0, 40.0, 30.0, 20.0, 10.0])
    assert abs(rho + 1.0) < 1e-9


def test_spearman_with_ties():
    rho, p, n = spearman([1.0, 2.0, 2.0, 3.0], [10.0, 20.0, 20.0, 30.0])
    assert abs(rho - 1.0) < 1e-9


def test_spearman_too_few_points():
    rho, p, n = spearman([1.0], [2.0])
    assert n == 1
    assert math.isnan(rho)


# ---------------------------------------------------------------------------
# 11-13. Bootstrap
# ---------------------------------------------------------------------------

def test_bootstrap_deterministic_with_seed():
    vals = [0.0, 0.5, 1.0, 0.33, 0.67]
    a = bootstrap_mean_ci(vals, n_boot=500, seed=42)
    b = bootstrap_mean_ci(vals, n_boot=500, seed=42)
    assert a == b


def test_bootstrap_different_seeds_differ():
    vals = [i / 19.0 for i in range(20)]
    a = bootstrap_mean_ci(vals, n_boot=500, seed=1)
    b = bootstrap_mean_ci(vals, n_boot=500, seed=2)
    assert a[0] == b[0]
    assert a[1] != b[1] or a[2] != b[2]


def test_bootstrap_single_value():
    mean, lo, hi = bootstrap_mean_ci([0.5], n_boot=100, seed=42)
    assert mean == 0.5
    assert math.isnan(lo) and math.isnan(hi)


def test_bootstrap_ci_contains_mean():
    vals = [0.0, 0.33, 0.33, 0.67, 1.0]
    mean, lo, hi = bootstrap_mean_ci(vals, n_boot=2000, seed=42)
    assert lo <= mean <= hi


# ---------------------------------------------------------------------------
# 14. Model normalization
# ---------------------------------------------------------------------------

def test_model_display_known():
    assert MODEL_DISPLAY["qwen2.5-coder:32b"] == "Qwen2.5-Coder-32B"
    assert MODEL_DISPLAY["codellama:34b"] == "CodeLlama-34B"


def test_model_display_unknown_passthrough(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "future-model:99b", "T001", "baseline", "", 1, 0, 0),
    ])
    scens = load_outcome_csv(p, "E1")
    assert scens[0].model_display == "future-model:99b"


# ---------------------------------------------------------------------------
# 15. Refusal handling
# ---------------------------------------------------------------------------

def test_refusal_row_loaded(tmp_path):
    p = tmp_path / "e2.csv"
    _write_outcome_csv(p, [
        _row("E2", "codellama:34b", "T001", "adv_A8.3", "A8.3", 1, 0, 0,
             refusal="True"),
    ])
    scens = load_outcome_csv(p, "E2")
    assert len(scens) == 1
    assert scens[0].refusal is True
    assert scens[0].cvr == 0


# ---------------------------------------------------------------------------
# 16. End-to-end CVR
# ---------------------------------------------------------------------------

def test_end_to_end_cvr_synthetic(tmp_path):
    tcs_path = tmp_path / "tcs.csv"
    _write_tcs_csv(tcs_path, [
        {"task_id": "T001", "category": "pick_place", "operating_mode": "fenced",
         "tcs_struct_raw": 1.0, "tcs_safety_raw": 1.0,
         "tcs_struct_z": -1.0, "tcs_safety_z": -1.0,
         "tcs_total": -1.0, "tertile": "T_low"},
        {"task_id": "T002", "category": "welding", "operating_mode": "hybrid",
         "tcs_struct_raw": 5.0, "tcs_safety_raw": 5.0,
         "tcs_struct_z": 0.0, "tcs_safety_z": 0.0,
         "tcs_total": 0.0, "tertile": "T_med"},
        {"task_id": "T003", "category": "pick_place", "operating_mode": "collaborative",
         "tcs_struct_raw": 10.0, "tcs_safety_raw": 10.0,
         "tcs_struct_z": 1.0, "tcs_safety_z": 1.0,
         "tcs_total": 1.0, "tertile": "T_high"},
    ])
    e1_path = tmp_path / "e1.csv"
    _write_outcome_csv(e1_path, [
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 1, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 2, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 3, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T002", "baseline", "", 1, 0, 1),
        _row("E1", "qwen2.5-coder:32b", "T002", "baseline", "", 2, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T002", "baseline", "", 3, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T003", "baseline", "", 1, 0, 5),
        _row("E1", "qwen2.5-coder:32b", "T003", "baseline", "", 2, 0, 3),
        _row("E1", "qwen2.5-coder:32b", "T003", "baseline", "", 3, 0, 2),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e1-csv", str(e1_path),
        "--output-dir", str(out_dir),
        "--outcome-metrics", "cvr",
        "--bootstrap", "500", "--seed", "42",
    ])
    assert rc == 0
    spear = list(csv.DictReader((out_dir / "spearman_summary.csv").open()))
    assert len(spear) == 1
    assert spear[0]["outcome_metric"] == "cvr"
    rho = float(spear[0]["spearman_rho"])
    assert rho > 0.9
    tert = list(csv.DictReader((out_dir / "tertile_cvr_summary.csv").open()))
    by_tertile = {r["tertile"]: r for r in tert}
    assert float(by_tertile["T_low"]["value_mean"]) == 0.0
    assert abs(float(by_tertile["T_med"]["value_mean"]) - 1 / 3) < 1e-6
    assert float(by_tertile["T_high"]["value_mean"]) == 1.0
    manifest = json.loads((out_dir / "manifest.json").read_text())
    assert manifest["outcome_metrics"] == ["cvr"]
    assert manifest["framing"].startswith("exploratory_H8")


# ---------------------------------------------------------------------------
# 17. End-to-end E3 collapse propagation
# ---------------------------------------------------------------------------

def test_end_to_end_e3_collapse_propagates(tmp_path):
    tcs_path = tmp_path / "tcs.csv"
    _write_tcs_csv(tcs_path, [
        {"task_id": "T001", "category": "pick_place", "operating_mode": "fenced",
         "tcs_struct_raw": 1.0, "tcs_safety_raw": 1.0,
         "tcs_struct_z": -1.0, "tcs_safety_z": -1.0,
         "tcs_total": -1.0, "tertile": "T_low"},
        {"task_id": "T002", "category": "welding", "operating_mode": "fenced",
         "tcs_struct_raw": 5.0, "tcs_safety_raw": 5.0,
         "tcs_struct_z": 0.0, "tcs_safety_z": 0.0,
         "tcs_total": 0.0, "tertile": "T_med"},
        {"task_id": "T003", "category": "pick_place", "operating_mode": "collaborative",
         "tcs_struct_raw": 10.0, "tcs_safety_raw": 10.0,
         "tcs_struct_z": 1.0, "tcs_safety_z": 1.0,
         "tcs_total": 1.0, "tertile": "T_high"},
    ])
    e3_path = tmp_path / "e3.csv"
    _write_outcome_csv(e3_path, [
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 0, 3),
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 1, 2),
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 2, 0),
        _row("E3", "m:1b", "T001", "watchdog_loop", "", 1, 0, 0),
        _row("E3", "m:1b", "T002", "watchdog_loop", "", 1, 0, 0),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e3-csv", str(e3_path),
        "--output-dir", str(out_dir),
        "--outcome-metrics", "cvr",
        "--bootstrap", "200", "--seed", "42",
    ])
    assert rc == 0
    joined = list(csv.DictReader((out_dir / "per_scenario_joined.csv").open()))
    by_task = {r["task_id"]: r for r in joined if r["outcome_metric"] == "cvr"}
    assert float(by_task["T003"]["value_mean"]) == 0.0  # final retry safe


# ---------------------------------------------------------------------------
# 18. Multi-metric run
# ---------------------------------------------------------------------------

def test_end_to_end_all_metrics(tmp_path):
    tcs_path = tmp_path / "tcs.csv"
    _write_tcs_csv(tcs_path, [
        {"task_id": "T001", "category": "pick_place", "operating_mode": "fenced",
         "tcs_struct_raw": 1.0, "tcs_safety_raw": 1.0,
         "tcs_struct_z": -1.0, "tcs_safety_z": -1.0,
         "tcs_total": -1.0, "tertile": "T_low"},
        {"task_id": "T002", "category": "welding", "operating_mode": "hybrid",
         "tcs_struct_raw": 5.0, "tcs_safety_raw": 5.0,
         "tcs_struct_z": 0.0, "tcs_safety_z": 0.0,
         "tcs_total": 0.0, "tertile": "T_med"},
        {"task_id": "T003", "category": "pick_place", "operating_mode": "collaborative",
         "tcs_struct_raw": 10.0, "tcs_safety_raw": 10.0,
         "tcs_struct_z": 1.0, "tcs_safety_z": 1.0,
         "tcs_total": 1.0, "tertile": "T_high"},
    ])
    e1_path = tmp_path / "e1.csv"
    _write_outcome_csv(e1_path, [
        _row("E1", "m", "T001", "baseline", "", 1, 0, 2, severity=0.3),
        _row("E1", "m", "T002", "baseline", "", 1, 0, 5, severity=0.6),
        _row("E1", "m", "T003", "baseline", "", 1, 0, 9, severity=1.0),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e1-csv", str(e1_path),
        "--output-dir", str(out_dir),
        "--outcome-metrics", "cvr", "count", "severity",
        "--bootstrap", "200", "--seed", "42",
    ])
    assert rc == 0
    spear = list(csv.DictReader((out_dir / "spearman_summary.csv").open()))
    metrics_present = {r["outcome_metric"] for r in spear}
    assert metrics_present == {"cvr", "count", "severity"}


# ---------------------------------------------------------------------------
# 19. Count metric reveals signal where CVR ceilings
# ---------------------------------------------------------------------------

def test_count_metric_reveals_signal_under_ceiling(tmp_path):
    """CVR=1 for all tasks (ceiling), but violation counts differ."""
    tcs_path = tmp_path / "tcs.csv"
    _write_tcs_csv(tcs_path, [
        {"task_id": "T001", "category": "pick_place", "operating_mode": "fenced",
         "tcs_struct_raw": 1.0, "tcs_safety_raw": 1.0,
         "tcs_struct_z": -1.0, "tcs_safety_z": -1.0,
         "tcs_total": -1.0, "tertile": "T_low"},
        {"task_id": "T002", "category": "welding", "operating_mode": "hybrid",
         "tcs_struct_raw": 5.0, "tcs_safety_raw": 5.0,
         "tcs_struct_z": 0.0, "tcs_safety_z": 0.0,
         "tcs_total": 0.0, "tertile": "T_med"},
        {"task_id": "T003", "category": "pick_place", "operating_mode": "collaborative",
         "tcs_struct_raw": 10.0, "tcs_safety_raw": 10.0,
         "tcs_struct_z": 1.0, "tcs_safety_z": 1.0,
         "tcs_total": 1.0, "tertile": "T_high"},
    ])
    e1_path = tmp_path / "e1.csv"
    # All tasks have at least one violation (CVR=1 everywhere) but counts
    # ascend with TCS: 2, 5, 9.
    _write_outcome_csv(e1_path, [
        _row("E1", "m", "T001", "baseline", "", 1, 0, 2),
        _row("E1", "m", "T002", "baseline", "", 1, 0, 5),
        _row("E1", "m", "T003", "baseline", "", 1, 0, 9),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e1-csv", str(e1_path),
        "--output-dir", str(out_dir),
        "--outcome-metrics", "cvr", "count",
        "--bootstrap", "200", "--seed", "42",
    ])
    assert rc == 0
    spear = list(csv.DictReader((out_dir / "spearman_summary.csv").open()))
    by_metric = {r["outcome_metric"]: r for r in spear}
    # CVR is constant -> y_is_constant=1, rho undefined
    assert by_metric["cvr"]["y_is_constant"] == "1"
    # count has variance -> rho should be 1.0 (perfect monotonic)
    assert by_metric["count"]["y_is_constant"] == "0"
    assert float(by_metric["count"]["spearman_rho"]) > 0.99


# ---------------------------------------------------------------------------
# 20. y_is_constant flag
# ---------------------------------------------------------------------------

def test_y_constant_flag_set(tmp_path):
    tcs_path = tmp_path / "tcs.csv"
    _write_tcs_csv(tcs_path, [
        {"task_id": f"T00{i+1}", "category": "x", "operating_mode": "fenced",
         "tcs_struct_raw": float(i), "tcs_safety_raw": float(i),
         "tcs_struct_z": float(i - 1), "tcs_safety_z": float(i - 1),
         "tcs_total": float(i - 1),
         "tertile": "T_low" if i < 1 else ("T_high" if i > 1 else "T_med")}
        for i in range(3)
    ])
    e1_path = tmp_path / "e1.csv"
    _write_outcome_csv(e1_path, [
        _row("E1", "m", "T001", "baseline", "", 1, 0, 5),
        _row("E1", "m", "T002", "baseline", "", 1, 0, 5),
        _row("E1", "m", "T003", "baseline", "", 1, 0, 5),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e1-csv", str(e1_path),
        "--output-dir", str(out_dir),
        "--outcome-metrics", "cvr",
        "--bootstrap", "100", "--seed", "42",
    ])
    assert rc == 0
    spear = list(csv.DictReader((out_dir / "spearman_summary.csv").open()))
    assert spear[0]["y_is_constant"] == "1"


# ---------------------------------------------------------------------------
# 21. SUPPORTED_METRICS coverage
# ---------------------------------------------------------------------------

def test_supported_metrics_set():
    assert set(SUPPORTED_METRICS) == {"cvr", "count", "severity"}
