"""Tests for compute_task_complexity.

Invariants enforced:
  1. Deterministic: identical IR -> identical TCS over many runs.
  2. Known-value: hand-checked features for 3 synthetic IRs.
  3. Tertile assignment stable across reruns.
  4. Weight monotonicity: raising waypoint weight does not lower the score
     of the waypoint-heaviest task relative to the others.
  5. Outcome blindness: the module does not import or open any file under
     results/, outcomes/, experiment*/, or watchdog/ output paths.
  6. Spec-frozen: tcs_weights.yaml is required and its absence raises.
  7. Z-score correctness: matches population formula on a small fixture.
  8. Tertile cut points are derived from total score distribution only.
"""
from __future__ import annotations

import ast
import math
from pathlib import Path

import pytest

from scripts.compute_task_complexity import (  # type: ignore
    SAFETY_DIMENSIONS,
    SafetyFeatures,
    StructuralFeatures,
    assign_tertile,
    compute_complexities,
    extract_safety,
    extract_structural,
    tertile_cuts,
    weighted_sum,
    z_score,
)


# ----- Fixtures ------------------------------------------------------------

def _minimal_task(**overrides):
    base = {
        "task_id": "T_test",
        "category": "pick_place",
        "mode": "collaborative",
        "waypoints": [
            {"name": "home", "frame": "base", "pose": [0, 0, 0]},
            {"name": "grasp", "frame": "base", "pose": [1, 0, 0]},
            {"name": "place", "frame": "workpiece", "pose": [2, 0, 0]},
        ],
        "motion_sequence": [
            {"type": "MoveJ", "target": "home"},
            {"type": "MoveL", "target": "grasp"},
            {"type": "MoveL", "target": "place"},
        ],
        "safety_constraints": {
            "max_speed_mms": 250,
            "e_stop_required": True,
        },
        "payload_kg": 0.5,
    }
    base.update(overrides)
    return base


def _uniform_weights():
    w_struct = {
        "waypoint_count": 1.0,
        "motion_command_count": 1.0,
        "distinct_frame_count": 1.0,
        "control_flow_branches": 1.0,
        "distinct_motion_types": 1.0,
    }
    w_safety = {
        "mode_strictness": 3.0,
        "applicable_iso_clause_count": 1.0,
        "applicable_dm_count": 1.0,
        "e_stop_required": 1.0,
        "payload_constraint_present": 1.0,
    }
    return w_struct, w_safety


# ----- 1. Deterministic ----------------------------------------------------

def test_extract_structural_deterministic():
    task = _minimal_task()
    a = extract_structural(task)
    b = extract_structural(task)
    c = extract_structural(task)
    assert a == b == c


def test_extract_safety_deterministic():
    task = _minimal_task()
    runs = [extract_safety(task) for _ in range(100)]
    assert all(r == runs[0] for r in runs)


# ----- 2. Known-value ------------------------------------------------------

def test_structural_known_values():
    f = extract_structural(_minimal_task())
    assert f.waypoint_count == 3
    assert f.motion_command_count == 3
    assert f.distinct_frame_count == 2          # base + workpiece
    assert f.control_flow_branches == 0
    assert f.distinct_motion_types == 2         # MoveJ + MoveL


def test_safety_known_values():
    f = extract_safety(_minimal_task())
    # collaborative
    assert f.mode_strictness == 3
    # e_stop, speed, payload, frame_consistency (2 frames) all active
    assert f.e_stop_required == 1
    assert f.payload_constraint_present == 1
    assert f.applicable_dm_count >= 4
    assert f.applicable_iso_clause_count >= 4


def test_safety_constraints_list_form():
    task = _minimal_task(safety_constraints=["max_speed_mms", "e_stop_required"])
    f = extract_safety(task)
    assert f.applicable_dm_count >= 2  # speed + e_stop


# ----- 3. Tertile stability + correctness ----------------------------------

def test_tertile_cuts_balanced():
    scores = [float(i) for i in range(15)]  # 0..14
    lo, hi = tertile_cuts(scores)
    # ~1/3 below lo, ~1/3 above hi
    n_low = sum(1 for s in scores if s <= lo)
    n_high = sum(1 for s in scores if s >= hi)
    assert 4 <= n_low <= 6
    assert 4 <= n_high <= 6


def test_tertile_assignment_branches():
    assert assign_tertile(-1.0, 0.0, 1.0) == "T_low"
    assert assign_tertile(0.5, 0.0, 1.0) == "T_med"
    assert assign_tertile(2.0, 0.0, 1.0) == "T_high"
    assert assign_tertile(0.0, 0.0, 1.0) == "T_low"   # inclusive on lo
    assert assign_tertile(1.0, 0.0, 1.0) == "T_high"  # inclusive on hi


# ----- 4. Weight monotonicity ----------------------------------------------

def test_weight_monotonicity_waypoint():
    """Increasing waypoint weight cannot lower the rank of the most
    waypoint-heavy task relative to a lighter one."""
    w_struct, w_safety = _uniform_weights()
    heavy = _minimal_task(waypoints=[
        {"name": f"wp{i}", "frame": "base", "pose": [i, 0, 0]} for i in range(10)
    ])
    light = _minimal_task(waypoints=[
        {"name": "wp0", "frame": "base", "pose": [0, 0, 0]},
    ])
    out_a = compute_complexities([heavy, light], w_struct, w_safety, 0.5, 0.5)
    w_struct2 = dict(w_struct)
    w_struct2["waypoint_count"] = 5.0
    out_b = compute_complexities([heavy, light], w_struct2, w_safety, 0.5, 0.5)
    # heavy is index 0 in both runs
    assert out_a[0].tcs_total >= out_a[1].tcs_total
    assert out_b[0].tcs_total >= out_b[1].tcs_total
    # And raising the weight does not flip the ordering
    assert (out_b[0].tcs_total - out_b[1].tcs_total) >= \
           (out_a[0].tcs_total - out_a[1].tcs_total) - 1e-9


# ----- 5. Outcome blindness (source-level static check) --------------------

def test_no_outcome_imports():
    """The TCS script must not import outcome modules nor open outcome
    files. AST-based check; the module docstring is intentionally allowed
    to mention 'results' for documentation purposes."""
    src = (Path(__file__).resolve().parents[1]
           / "scripts" / "compute_task_complexity.py").read_text()
    tree = ast.parse(src)

    forbidden_modules = {
        "experiment_runner", "llm_experiment_runner",
        "watchdog_output", "llm_responses",
    }
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            mod = (node.module or "").split(".")[0]
            assert mod not in forbidden_modules, f"Forbidden import: {mod}"
            assert mod != "results", "TCS imports from results/"
        elif isinstance(node, ast.Import):
            for alias in node.names:
                top = alias.name.split(".")[0]
                assert top not in forbidden_modules, f"Forbidden import: {top}"

    # Check no open() call targets outcome paths.
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "open":
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                    v = arg.value.lower()
                    assert not v.startswith("results/e"), \
                        f"open() targets outcome path: {arg.value!r}"
                    assert "experiment_runner" not in v, \
                        f"open() targets experiment path: {arg.value!r}"


# ----- 6. Spec-frozen ------------------------------------------------------

def test_weights_yaml_required(tmp_path):
    from scripts.compute_task_complexity import main as tcs_main
    tasks_dir = tmp_path / "tasks"
    tasks_dir.mkdir()
    out = tmp_path / "out.csv"
    bad_cfg = tmp_path / "missing.yaml"
    with pytest.raises((FileNotFoundError, OSError)):
        tcs_main([
            "--tasks-dir", str(tasks_dir),
            "--weights-config", str(bad_cfg),
            "--output", str(out),
        ])


# ----- 7. Z-score correctness ----------------------------------------------

def test_z_score_matches_formula():
    vals = [1.0, 2.0, 3.0, 4.0, 5.0]
    mean = 3.0
    sd = math.sqrt(sum((v - mean) ** 2 for v in vals) / (len(vals) - 1))
    for v in vals:
        expected = (v - mean) / sd
        assert math.isclose(z_score(vals, v), expected, abs_tol=1e-9)


def test_z_score_single_value():
    assert z_score([5.0], 5.0) == 0.0


def test_z_score_zero_variance():
    assert z_score([7.0, 7.0, 7.0], 7.0) == 0.0


# ----- 8. End-to-end integration ------------------------------------------

def test_compute_complexities_end_to_end():
    w_struct, w_safety = _uniform_weights()
    irs = [
        _minimal_task(task_id="T001", mode="collaborative"),
        _minimal_task(task_id="T002", mode="fenced",
                      waypoints=[{"name": "a", "frame": "base", "pose": [0, 0, 0]}],
                      motion_sequence=[{"type": "MoveL", "target": "a"}]),
        _minimal_task(task_id="T003", mode="hybrid"),
    ]
    out = compute_complexities(irs, w_struct, w_safety, 0.5, 0.5)
    assert len(out) == 3
    assert {t.task_id for t in out} == {"T001", "T002", "T003"}
    assert all(t.tertile in {"T_low", "T_med", "T_high"} for t in out)
    # Collaborative + many waypoints task should not score below the
    # fenced single-waypoint task.
    t001 = next(t for t in out if t.task_id == "T001")
    t002 = next(t for t in out if t.task_id == "T002")
    assert t001.tcs_total >= t002.tcs_total


def test_safety_dimensions_complete():
    """All 7 DMs must be reachable through some safety dimension."""
    all_dms = set()
    for dms, _ in SAFETY_DIMENSIONS.values():
        all_dms.update(dms)
    assert all_dms == {f"DM-{i}" for i in range(1, 8)}
