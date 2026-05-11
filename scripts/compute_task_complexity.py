#!/usr/bin/env python3
"""compute_task_complexity.py

Deterministic Task Complexity Score (TCS) computation for ENFIELD baseline
tasks. Produces a per-task descriptor that is independent of any LLM output
or experimental outcome.

TCS has two component sub-scores, both derived solely from baseline Task IR:

  - TCS_struct (structural complexity):
        waypoint_count, motion_command_count, distinct_frame_count,
        control_flow_branches, distinct_motion_types
  - TCS_safety (safety surface):
        mode_strictness, applicable_iso_clause_count, applicable_dm_count,
        e_stop_required, payload_constraint_present

Final TCS_total = w_blend_struct * z(TCS_struct_raw)
                + w_blend_safety * z(TCS_safety_raw)

z-scoring is performed across the 15-task suite. Tertiles (T_low / T_med /
T_high) are assigned from the empirical distribution of TCS_total.

NON-NEGOTIABLE invariants (enforced by tests):
  1. Deterministic: same input -> identical output. No randomness.
  2. Outcome-blind: this module imports NOTHING from results/, experiments/,
     watchdog outputs, or LLM responses. Reads ONLY baseline Task IR.
  3. Frozen weights: weights live in configs/tcs_weights.yaml. Its SHA-256
     hash is recorded in docs/TASK_COMPLEXITY_SPEC.md at commit time.
  4. Pre-tertile commit: tertile boundaries are derived from the score
     distribution, NOT from any outcome data.

Schema assumptions (adjust extractors if your IR differs):
  task_id: str                       # e.g. "T001"
  category: str                      # pick_place | welding | palletizing | inspection
  mode: str                          # collaborative | hybrid | fenced
  payload_kg: float                  # optional
  waypoints: list[{name, frame, pose}]
  motion_sequence: list[{type, target, speed, condition?}]
  safety_constraints: dict | list
  safezones: list                    # optional
  tool_config: dict                  # optional

Usage:
    python scripts/compute_task_complexity.py \\
        --tasks-dir tasks/baseline/ \\
        --weights-config configs/tcs_weights.yaml \\
        --output results/task_complexity_scores.csv \\
        --manifest results/task_complexity_manifest.json
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import logging
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

import yaml  # type: ignore

LOG = logging.getLogger("tcs")

# Mode strictness: collaborative is hardest (humans share workspace),
# fenced is easiest (physical separation). Hybrid is in between.
MODE_STRICTNESS: dict[str, int] = {
    "collaborative": 3,
    "hybrid": 2,
    "fenced": 1,
}

# Mapping from safety constraint dimension to (DM rules, ISO clauses).
# Derived from THREAT_MODEL.md / iso_clause_mapping in the repo.
SAFETY_DIMENSIONS: dict[str, tuple[set[str], set[str]]] = {
    "speed_limit":       ({"DM-1"}, {"5.5.3"}),
    "zone_boundary":     ({"DM-2"}, {"5.7.4"}),
    "orientation":       ({"DM-3"}, {"5.7.4"}),
    "payload":           ({"DM-4"}, {"5.7.5"}),
    "e_stop":            ({"DM-5"}, {"5.4.2"}),
    "frame_consistency": ({"DM-6"}, {"5.7.4"}),
    "tool_parameter":    ({"DM-7"}, {"5.1.14", "5.1.15"}),
}


@dataclass(frozen=True)
class StructuralFeatures:
    waypoint_count: int
    motion_command_count: int
    distinct_frame_count: int
    control_flow_branches: int
    distinct_motion_types: int


@dataclass(frozen=True)
class SafetyFeatures:
    mode_strictness: int
    applicable_iso_clause_count: int
    applicable_dm_count: int
    e_stop_required: int
    payload_constraint_present: int


@dataclass
class TaskComplexity:
    task_id: str
    category: str
    mode: str
    structural: StructuralFeatures
    safety: SafetyFeatures
    tcs_struct_raw: float
    tcs_safety_raw: float
    tcs_struct_z: float = float("nan")
    tcs_safety_z: float = float("nan")
    tcs_total: float = float("nan")
    tertile: str = ""


# ----------------------------------------------------------------------------
# Feature extractors. Public for unit testing.
# ----------------------------------------------------------------------------

def extract_structural(task: dict[str, Any]) -> StructuralFeatures:
    waypoints = task.get("waypoints") or []
    motion = task.get("motion_sequence") or task.get("motions") or []
    frames: set[str] = set()
    for wp in waypoints:
        if isinstance(wp, dict) and wp.get("frame"):
            frames.add(str(wp["frame"]))
    branches = 0
    motion_types: set[str] = set()
    BRANCH_KEYWORDS = {"if", "while", "for", "error_handler", "trycatch"}
    for step in motion:
        if not isinstance(step, dict):
            continue
        st_type = step.get("type")
        if st_type in BRANCH_KEYWORDS:
            branches += 1
        if step.get("condition"):
            branches += 1
        if st_type:
            motion_types.add(str(st_type))
    return StructuralFeatures(
        waypoint_count=len(waypoints),
        motion_command_count=len(motion),
        distinct_frame_count=len(frames),
        control_flow_branches=branches,
        distinct_motion_types=len(motion_types),
    )


def _normalize_constraints(raw: Any) -> dict[str, Any]:
    """Accept both list-of-keys and dict forms of safety_constraints."""
    if raw is None:
        return {}
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, list):
        out: dict[str, Any] = {}
        for item in raw:
            if isinstance(item, str):
                out[item] = True
            elif isinstance(item, dict):
                out.update(item)
        return out
    return {}


def extract_safety(task: dict[str, Any]) -> SafetyFeatures:
    mode = str(task.get("mode", "fenced"))
    strictness = MODE_STRICTNESS.get(mode, 1)
    constraints = _normalize_constraints(task.get("safety_constraints"))

    applicable_dms: set[str] = set()
    applicable_clauses: set[str] = set()

    def _activate(dim: str) -> None:
        dms, clauses = SAFETY_DIMENSIONS[dim]
        applicable_dms.update(dms)
        applicable_clauses.update(clauses)

    if any(k in constraints for k in ("max_speed_mms", "speed_limit", "target_speed")):
        _activate("speed_limit")
    if "zone_boundary" in constraints or task.get("safezones"):
        _activate("zone_boundary")
    if "orientation_limits" in constraints or "orientation" in constraints:
        _activate("orientation")
    payload_present = int(
        bool(task.get("payload_kg") or "payload_max" in constraints or "payload" in constraints)
    )
    if payload_present:
        _activate("payload")
    e_stop_required = int(bool(constraints.get("e_stop_required", True)))
    if e_stop_required:
        _activate("e_stop")
    waypoints = task.get("waypoints") or []
    distinct_frames = {wp.get("frame") for wp in waypoints if isinstance(wp, dict)}
    distinct_frames.discard(None)
    if task.get("multi_frame") or len(distinct_frames) > 1:
        _activate("frame_consistency")
    if task.get("tool_config") or task.get("tools"):
        _activate("tool_parameter")

    return SafetyFeatures(
        mode_strictness=strictness,
        applicable_iso_clause_count=len(applicable_clauses),
        applicable_dm_count=len(applicable_dms),
        e_stop_required=e_stop_required,
        payload_constraint_present=payload_present,
    )


# ----------------------------------------------------------------------------
# Scoring
# ----------------------------------------------------------------------------

def weighted_sum(features: Any, weights: dict[str, float]) -> float:
    feat_dict = asdict(features)
    missing = set(weights) - set(feat_dict)
    if missing:
        raise KeyError(f"Weights reference unknown features: {sorted(missing)}")
    return float(sum(weights[k] * feat_dict[k] for k in weights))


def z_score(values: list[float], x: float) -> float:
    n = len(values)
    if n < 2:
        return 0.0
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    sd = math.sqrt(var)
    if sd == 0.0:
        return 0.0
    return (x - mean) / sd


def assign_tertile(score: float, lo_cut: float, hi_cut: float) -> str:
    if score <= lo_cut:
        return "T_low"
    if score >= hi_cut:
        return "T_high"
    return "T_med"


def tertile_cuts(scores: list[float]) -> tuple[float, float]:
    """Return (lo_cut, hi_cut) such that ~1/3 of tasks fall in each tertile."""
    if not scores:
        return (0.0, 0.0)
    s = sorted(scores)
    n = len(s)
    lo_idx = max(0, n // 3 - 1)
    hi_idx = min(n - 1, (2 * n) // 3)
    return (s[lo_idx], s[hi_idx])


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


# ----------------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------------

def load_tasks(tasks_dir: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for path in sorted(tasks_dir.glob("*.y*ml")):
        with path.open() as f:
            ir = yaml.safe_load(f) or {}
        if "task_id" not in ir:
            ir["task_id"] = path.stem
        out.append(ir)
    return out


def compute_complexities(
    irs: Iterable[dict[str, Any]],
    weights_struct: dict[str, float],
    weights_safety: dict[str, float],
    blend_struct: float,
    blend_safety: float,
) -> list[TaskComplexity]:
    tasks: list[TaskComplexity] = []
    for ir in irs:
        struct = extract_structural(ir)
        safety = extract_safety(ir)
        tasks.append(TaskComplexity(
            task_id=str(ir.get("task_id", "")),
            category=str(ir.get("category", "unknown")),
            mode=str(ir.get("mode", "unknown")),
            structural=struct,
            safety=safety,
            tcs_struct_raw=weighted_sum(struct, weights_struct),
            tcs_safety_raw=weighted_sum(safety, weights_safety),
        ))
    if not tasks:
        return tasks
    s_vals = [t.tcs_struct_raw for t in tasks]
    f_vals = [t.tcs_safety_raw for t in tasks]
    for t in tasks:
        t.tcs_struct_z = z_score(s_vals, t.tcs_struct_raw)
        t.tcs_safety_z = z_score(f_vals, t.tcs_safety_raw)
        t.tcs_total = blend_struct * t.tcs_struct_z + blend_safety * t.tcs_safety_z
    totals = [t.tcs_total for t in tasks]
    lo, hi = tertile_cuts(totals)
    for t in tasks:
        t.tertile = assign_tertile(t.tcs_total, lo, hi)
    return tasks


def write_csv(tasks: list[TaskComplexity], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for t in tasks:
        row: dict[str, Any] = {
            "task_id": t.task_id,
            "category": t.category,
            "mode": t.mode,
        }
        row.update({f"struct_{k}": v for k, v in asdict(t.structural).items()})
        row.update({f"safety_{k}": v for k, v in asdict(t.safety).items()})
        row["tcs_struct_raw"] = round(t.tcs_struct_raw, 6)
        row["tcs_safety_raw"] = round(t.tcs_safety_raw, 6)
        row["tcs_struct_z"] = round(t.tcs_struct_z, 6)
        row["tcs_safety_z"] = round(t.tcs_safety_z, 6)
        row["tcs_total"] = round(t.tcs_total, 6)
        row["tertile"] = t.tertile
        rows.append(row)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tasks-dir", type=Path, required=True)
    parser.add_argument("--weights-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    with args.weights_config.open() as f:
        cfg = yaml.safe_load(f)
    w_struct = cfg["weights"]["structural"]
    w_safety = cfg["weights"]["safety"]
    b_struct = float(cfg["weights"]["blend"]["struct"])
    b_safety = float(cfg["weights"]["blend"]["safety"])

    irs = load_tasks(args.tasks_dir)
    if not irs:
        LOG.error("No task IR YAML found in %s", args.tasks_dir)
        return 2

    tasks = compute_complexities(irs, w_struct, w_safety, b_struct, b_safety)
    write_csv(tasks, args.output)
    LOG.info("Wrote %d task complexity rows to %s", len(tasks), args.output)

    if args.manifest:
        manifest = {
            "spec_version": cfg.get("version", "unknown"),
            "weights_config": str(args.weights_config),
            "weights_config_sha256": file_sha256(args.weights_config),
            "tcs_script_sha256": file_sha256(Path(__file__)),
            "tasks_count": len(tasks),
            "tertile_distribution": {
                t: sum(1 for x in tasks if x.tertile == t)
                for t in ("T_low", "T_med", "T_high")
            },
        }
        args.manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True))
        LOG.info("Wrote manifest to %s", args.manifest)

    return 0


if __name__ == "__main__":
    sys.exit(main())
