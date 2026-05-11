"""Tests for complexity_correlation_analysis.

Invariants enforced:
  1. CVR definition: total_violations > 0 -> 1, else 0.
  2. E3 retry collapse: keeps MAX(retry) row per (model, task, rep, condition).
  3. E1/E2 pass through retry collapse unchanged.
  4. Rep aggregation: MEAN CVR across reps; 3 reps with 0/1/0 -> 1/3.
  5. Spearman: perfect monotonic ascending -> rho = 1.0.
  6. Spearman: perfect monotonic descending -> rho = -1.0.
  7. Spearman: with ties handled via average ranks.
  8. Bootstrap: deterministic given seed; both empty and single-value
     edge cases handled.
  9. End-to-end on a synthetic 3-task * 1-model * 2-condition fixture.
 10. Model normalization applied to known tags; unknown tags pass through.
 11. Refusal rows handled: cvr still derived from total_violations.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from scripts.complexity_correlation_analysis import (  # type: ignore
    MODEL_DISPLAY,
    Scenario,
    TaskCVR,
    aggregate_reps,
    bootstrap_mean_ci,
    collapse_e3_retries,
    load_outcome_csv,
    load_tcs_csv,
    spearman,
    write_per_scenario_csv,
    write_spearman_summary,
    write_tertile_summary,
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
         dm=0, sm=None):
    if sm is None:
        sm = total_violations  # default: all violations are SM
    return (
        f"{experiment},{model},{task_id},pick_place,collaborative,{condition},"
        f"{adv_type},{rep},{retry},{status},{refusal},19,True,False,{dm},{sm},"
        f"{total_violations},\"SM-1\",1.0,268,428,15000,2026-04-15T00:00:00+00:00"
    )


def _write_outcome_csv(path: Path, rows: list[str]) -> None:
    path.write_text("\n".join([OUTCOME_HEADER] + rows) + "\n")


def _write_tcs_csv(path: Path, rows: list[dict]) -> None:
    header = list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------------------
# 1. CVR definition
# ---------------------------------------------------------------------------

def test_cvr_from_total_violations(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 1, 0, 0),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 2, 0, 7),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 3, 0, 1),
    ])
    scens = load_outcome_csv(p, "E1")
    assert [s.cvr for s in scens] == [0, 1, 1]


def test_cvr_skips_non_success(tmp_path):
    p = tmp_path / "e1.csv"
    _write_outcome_csv(p, [
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 1, 0, 5),
        _row("E1", "qwen2.5-coder:32b", "T001", "baseline", "", 2, 0, 0,
             status="error"),
    ])
    scens = load_outcome_csv(p, "E1")
    assert len(scens) == 1
    assert scens[0].cvr == 1


# ---------------------------------------------------------------------------
# 2-3. E3 retry collapse
# ---------------------------------------------------------------------------

def test_e3_collapse_keeps_max_retry():
    # Two E3 rows for same (model, task, rep, condition); retry=0 has cvr=1,
    # retry=2 has cvr=0 (watchdog fixed it). Expect retry=2 row kept.
    rows = [
        Scenario("E3", "qwen2.5-coder:32b", "Qwen2.5-Coder-32B", "T001",
                 "baseline", "", 1, 0, False, 1),
        Scenario("E3", "qwen2.5-coder:32b", "Qwen2.5-Coder-32B", "T001",
                 "baseline", "", 1, 2, False, 0),
        Scenario("E3", "qwen2.5-coder:32b", "Qwen2.5-Coder-32B", "T001",
                 "baseline", "", 1, 1, False, 1),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 1
    assert out[0].retry == 2
    assert out[0].cvr == 0


def test_e3_collapse_keeps_distinct_reps():
    rows = [
        Scenario("E3", "m", "M", "T001", "baseline", "", 1, 0, False, 1),
        Scenario("E3", "m", "M", "T001", "baseline", "", 1, 2, False, 0),
        Scenario("E3", "m", "M", "T001", "baseline", "", 2, 0, False, 1),
        Scenario("E3", "m", "M", "T001", "baseline", "", 2, 1, False, 1),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 2  # one per rep
    assert {s.rep for s in out} == {1, 2}


def test_e1_e2_pass_through_collapse():
    """E1 and E2 should be unaffected; every row is preserved."""
    rows = [
        Scenario("E1", "m", "M", "T001", "baseline", "", 1, 0, False, 0),
        Scenario("E1", "m", "M", "T001", "baseline", "", 2, 0, False, 1),
        Scenario("E2", "m", "M", "T001", "adv_A8.1", "A8.1", 1, 0, False, 1),
    ]
    out = collapse_e3_retries(rows)
    assert len(out) == 3


# ---------------------------------------------------------------------------
# 4. Rep aggregation
# ---------------------------------------------------------------------------

def test_rep_aggregation_mean():
    rows = [
        Scenario("E1", "m", "M", "T001", "baseline", "", 1, 0, False, 0),
        Scenario("E1", "m", "M", "T001", "baseline", "", 2, 0, False, 1),
        Scenario("E1", "m", "M", "T001", "baseline", "", 3, 0, False, 0),
    ]
    out = aggregate_reps(rows)
    assert len(out) == 1
    assert out[0].n_reps == 3
    assert abs(out[0].cvr_mean - 1 / 3) < 1e-9


def test_rep_aggregation_splits_by_condition():
    rows = [
        Scenario("E1", "m", "M", "T001", "baseline", "", 1, 0, False, 0),
        Scenario("E1", "m", "M", "T001", "safety", "", 1, 0, False, 1),
    ]
    out = aggregate_reps(rows)
    assert len(out) == 2
    by_cond = {x.condition: x for x in out}
    assert by_cond["baseline"].cvr_mean == 0.0
    assert by_cond["safety"].cvr_mean == 1.0


# ---------------------------------------------------------------------------
# 5-7. Spearman
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
    # Perfectly monotonic in rank-space with matching ties -> rho = 1
    assert abs(rho - 1.0) < 1e-9


def test_spearman_too_few_points():
    rho, p, n = spearman([1.0], [2.0])
    assert n == 1
    import math
    assert math.isnan(rho)


# ---------------------------------------------------------------------------
# 8. Bootstrap
# ---------------------------------------------------------------------------

def test_bootstrap_deterministic_with_seed():
    vals = [0.0, 0.5, 1.0, 0.33, 0.67]
    a = bootstrap_mean_ci(vals, n_boot=500, seed=42)
    b = bootstrap_mean_ci(vals, n_boot=500, seed=42)
    assert a == b


def test_bootstrap_different_seeds_differ():
    """Two different seeds on a moderately diverse sample should yield
    at least one differing CI bound (not a guarantee with tiny discrete
    samples, so we use n=20)."""
    vals = [i / 19.0 for i in range(20)]  # 0.0 .. 1.0 step 1/19
    a = bootstrap_mean_ci(vals, n_boot=500, seed=1)
    b = bootstrap_mean_ci(vals, n_boot=500, seed=2)
    assert a[0] == b[0]  # mean of full sample is identical
    assert a[1] != b[1] or a[2] != b[2]


def test_bootstrap_single_value():
    mean, lo, hi = bootstrap_mean_ci([0.5], n_boot=100, seed=42)
    assert mean == 0.5
    import math
    assert math.isnan(lo) and math.isnan(hi)


def test_bootstrap_ci_contains_mean():
    """For a reasonable sample size, the CI should bracket the mean."""
    vals = [0.0, 0.33, 0.33, 0.67, 1.0]
    mean, lo, hi = bootstrap_mean_ci(vals, n_boot=2000, seed=42)
    assert lo <= mean <= hi


# ---------------------------------------------------------------------------
# 10. Model normalization
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
# 9. End-to-end (synthetic 3-task fixture)
# ---------------------------------------------------------------------------

def test_end_to_end_synthetic(tmp_path):
    """Build a synthetic case where TCS is monotonically related to CVR
    by construction, so Spearman rho should be ~1 in the baseline arm."""
    # TCS: T001 lowest, T003 highest
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
    # E1 outcomes: T001 always safe, T002 1/3 violations, T003 always violated
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
        "--bootstrap", "500",
        "--seed", "42",
    ])
    assert rc == 0
    # Spearman summary should have 1 row with rho ~ 1.0
    spear = list(csv.DictReader((out_dir / "spearman_summary.csv").open()))
    assert len(spear) == 1
    rho = float(spear[0]["spearman_rho"])
    assert rho > 0.9, f"expected rho near 1.0, got {rho}"
    # Tertile summary: 3 tertiles
    tert = list(csv.DictReader((out_dir / "tertile_cvr_summary.csv").open()))
    by_tertile = {r["tertile"]: r for r in tert}
    assert set(by_tertile) == {"T_low", "T_med", "T_high"}
    assert float(by_tertile["T_low"]["mean_cvr"]) == 0.0
    assert abs(float(by_tertile["T_med"]["mean_cvr"]) - 1 / 3) < 1e-6
    assert float(by_tertile["T_high"]["mean_cvr"]) == 1.0
    # Manifest captures policies
    manifest = json.loads((out_dir / "manifest.json").read_text())
    assert manifest["policy_e3_retry"] == "max_retry_per_rep"
    assert manifest["policy_rep_aggregation"] == "mean_cvr_across_reps"
    assert manifest["framing"].startswith("exploratory_H8")


# ---------------------------------------------------------------------------
# 11. Refusal handling
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
    assert scens[0].cvr == 0  # no violations, refusal


# ---------------------------------------------------------------------------
# 12. End-to-end with E3 retry collapse
# ---------------------------------------------------------------------------

def test_end_to_end_e3_collapse(tmp_path):
    """E3 with retry: initial cvr=1, retry=2 cvr=0. Final should be 0."""
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
        # T003 rep 1: retry 0 -> violation, retry 1 -> violation, retry 2 -> fixed
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 0, 3),
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 1, 2),
        _row("E3", "m:1b", "T003", "watchdog_loop", "", 1, 2, 0),
        # T001/T002 single retry
        _row("E3", "m:1b", "T001", "watchdog_loop", "", 1, 0, 0),
        _row("E3", "m:1b", "T002", "watchdog_loop", "", 1, 0, 0),
    ])
    out_dir = tmp_path / "out"
    from scripts.complexity_correlation_analysis import main as ccor_main
    rc = ccor_main([
        "--tcs-csv", str(tcs_path),
        "--e3-csv", str(e3_path),
        "--output-dir", str(out_dir),
        "--bootstrap", "200",
        "--seed", "42",
    ])
    assert rc == 0
    joined = list(csv.DictReader((out_dir / "per_scenario_joined.csv").open()))
    by_task = {r["task_id"]: r for r in joined}
    # After retry collapse, T003 final cvr = 0
    assert float(by_task["T003"]["cvr_mean"]) == 0.0
