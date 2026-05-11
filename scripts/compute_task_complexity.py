#!/usr/bin/env python3
"""compute_task_complexity.py

Deterministic Task Complexity Score (TCS) computation for ENFIELD baseline
tasks, aligned with task_ir_v1 schema (enfield_tasks/ir/schema/task_ir_v1.schema.json).

TCS has two sub-scores, both derived solely from baseline Task IR (no LLM
output, no watchdog output, no experiment results):

  TCS_struct (structural complexity):
      motion_command_count        - len(motion_sequence)
      distinct_command_types      - distinct motion_sequence[*].type values
      distinct_motion_frames      - distinct motion_sequence[*].frame_id values
      environment_frame_count     - len(environment.frames)
      safety_zone_count           - len(environment.safety_zones)

  TCS_safety (attack-surface complexity):
      mode_strictness             - collaborative=3, hybrid=2, fenced=1
      human_proximity_flag        - safety_requirements.human_proximity
      iso_clause_count            - len(safety_requirements.iso_clauses)
      halfspace_count             - A2 surface
      orientation_cone_count      - A3 surface
      required_safety_node_count  - A5 surface
      tool_activation_present     - A7 surface (0/1)
      work_object_spec_present    - A6 surface (0/1)
      estop_required_flag         - A5
      blocked_pattern_count       - A8 surface

Final score:
    TCS_total = w_blend_struct * z(TCS_struct_raw)
              + w_blend_safety * z(TCS_safety_raw)
z-scoring is across the task suite. Tertile boundaries are derived from the
TCS_total distribution alone, BEFORE any outcome data is joined.

NON-NEGOTIABLE invariants (enforced by tests):
  1. Deterministic: same input -> identical output.
  2. Outcome-blind: no import of results/, experiment_runner, watchdog
     outputs, or LLM responses.
  3. Frozen weights: configs/tcs_weights.yaml; SHA-256 in output manifest.
  4. Tertile boundaries derived from score distribution only.

Supports both JSON (canonical task_ir_v1) and YAML inputs.

Usage:
    python scripts/compute_task_complexity.py \\
        --tasks-dir enfield_tasks/ir/tasks/ \\
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
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml  # type: ignore

LOG = logging.getLogger("tcs")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MODE_STRICTNESS: dict[str, int] = {
    "collaborative": 3,
    "hybrid": 2,
    "fenced": 1,
}

SUPPORTED_SCHEMA_VERSIONS = {"1.0.0"}


# ---------------------------------------------------------------------------
# Feature dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class StructuralFeatures:
    motion_command_count: int
    distinct_command_types: int
    distinct_motion_frames: int
    environment_frame_count: int
    safety_zone_count: int


@dataclass(frozen=True)
class SafetyFeatures:
    mode_strictness: int
    human_proximity_flag: int
    iso_clause_count: int
    halfspace_count: int
    orientation_cone_count: int
    required_safety_node_count: int
    tool_activation_present: int
    work_object_spec_present: int
    estop_required_flag: int
    blocked_pattern_count: int


@dataclass
class TaskComplexity:
    task_id: str
    category: str
    operating_mode: str
    structural: StructuralFeatures
    safety: SafetyFeatures
    tcs_struct_raw: float
    tcs_safety_raw: float
    tcs_struct_z: float = float("nan")
    tcs_safety_z: float = float("nan")
    tcs_total: float = float("nan")
    tertile: str = ""


# ---------------------------------------------------------------------------
# Extractors. Public for unit testing.
# ---------------------------------------------------------------------------

def extract_structural(ir: dict[str, Any]) -> StructuralFeatures:
    motion = ir.get("motion_sequence") or []
    env = ir.get("environment") or {}
    distinct_types: set[str] = set()
    distinct_motion_frames: set[str] = set()
    for cmd in motion:
        if not isinstance(cmd, dict):
            continue
        t = cmd.get("type")
        if t:
            distinct_types.add(str(t))
        fid = cmd.get("frame_id")
        if fid:
            distinct_motion_frames.add(str(fid))
    env_frames = env.get("frames") or {}
    safety_zones = env.get("safety_zones") or []
    return StructuralFeatures(
        motion_command_count=len(motion),
        distinct_command_types=len(distinct_types),
        distinct_motion_frames=len(distinct_motion_frames),
        environment_frame_count=len(env_frames),
        safety_zone_count=len(safety_zones),
    )


def extract_safety(ir: dict[str, Any]) -> SafetyFeatures:
    task = ir.get("task") or {}
    mode = str(task.get("operating_mode", "fenced"))
    strictness = MODE_STRICTNESS.get(mode, 1)

    sr = ir.get("safety_requirements") or {}
    slr = ir.get("safety_logic_requirements") or {}
    tool = ir.get("tool") or {}
    wos = ir.get("work_object_specification")
    psec = ir.get("prompt_security") or {}

    safeguarded = sr.get("safeguarded_space") or {}
    halfspaces = safeguarded.get("halfspaces") or []
    orientation_cones = sr.get("forbidden_orientation_cones") or []
    iso_clauses = sr.get("iso_clauses") or []
    required_nodes = slr.get("required_nodes") or []
    blocked_patterns = psec.get("blocked_patterns") or []

    estop_required = int(bool(sr.get("estop_required", True)))
    human_proximity = int(bool(sr.get("human_proximity", False)))
    tool_activation_present = int(bool(tool.get("activation_constraints")))
    # Work object spec is "present" only if it actually contains a wobj_name
    wos_present = int(bool(wos and (wos.get("wobj_name") or wos.get("uframe_translation_mm"))))

    return SafetyFeatures(
        mode_strictness=strictness,
        human_proximity_flag=human_proximity,
        iso_clause_count=len(iso_clauses),
        halfspace_count=len(halfspaces),
        orientation_cone_count=len(orientation_cones),
        required_safety_node_count=len(required_nodes),
        tool_activation_present=tool_activation_present,
        work_object_spec_present=wos_present,
        estop_required_flag=estop_required,
        blocked_pattern_count=len(blocked_patterns),
    )


# ---------------------------------------------------------------------------
# Scoring helpers
# ---------------------------------------------------------------------------

def weighted_sum(features: Any, weights: dict[str, float]) -> float:
    feat_dict = asdict(features)
    missing = set(weights) - set(feat_dict)
    if missing:
        raise KeyError(
            f"Weights config references unknown features: {sorted(missing)}; "
            f"available: {sorted(feat_dict)}"
        )
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


def tertile_cuts(scores: list[float]) -> tuple[float, float]:
    """Return (lo_cut, hi_cut) such that ~1/3 of tasks fall in each tertile."""
    if not scores:
        return (0.0, 0.0)
    s = sorted(scores)
    n = len(s)
    lo_idx = max(0, n // 3 - 1)
    hi_idx = min(n - 1, (2 * n) // 3)
    return (s[lo_idx], s[hi_idx])


def assign_tertile(score: float, lo_cut: float, hi_cut: float) -> str:
    if score <= lo_cut:
        return "T_low"
    if score >= hi_cut:
        return "T_high"
    return "T_med"


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def _load_one(path: Path) -> dict[str, Any]:
    text = path.read_text()
    if path.suffix.lower() == ".json":
        return json.loads(text)
    if path.suffix.lower() in (".yaml", ".yml"):
        return yaml.safe_load(text) or {}
    raise ValueError(f"Unsupported task IR extension: {path.suffix}")


def load_tasks(tasks_dir: Path) -> list[tuple[Path, dict[str, Any]]]:
    out: list[tuple[Path, dict[str, Any]]] = []
    candidates: list[Path] = []
    for ext in ("*.json", "*.yaml", "*.yml"):
        candidates.extend(tasks_dir.glob(ext))
    for path in sorted(candidates):
        # Skip schema and manifest files
        name = path.name.lower()
        if "schema" in name or name.startswith("_") or name == "manifest.json":
            continue
        try:
            ir = _load_one(path)
        except Exception as exc:  # noqa: BLE001
            LOG.warning("Skipping %s: %s", path, exc)
            continue
        sv = ir.get("schema_version")
        if sv and sv not in SUPPORTED_SCHEMA_VERSIONS:
            LOG.warning("Skipping %s: unsupported schema_version %r", path, sv)
            continue
        # Must have a task block with id, otherwise skip
        if not isinstance(ir.get("task"), dict):
            LOG.debug("Skipping %s: no task block", path)
            continue
        out.append((path, ir))
    return out


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def compute_complexities(
    irs: Iterable[dict[str, Any]],
    weights_struct: dict[str, float],
    weights_safety: dict[str, float],
    blend_struct: float,
    blend_safety: float,
) -> list[TaskComplexity]:
    tasks: list[TaskComplexity] = []
    for ir in irs:
        task = ir.get("task") or {}
        struct = extract_structural(ir)
        safety = extract_safety(ir)
        tasks.append(TaskComplexity(
            task_id=str(task.get("id", "")),
            category=str(task.get("category", "unknown")),
            operating_mode=str(task.get("operating_mode", "unknown")),
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
    lo, hi = tertile_cuts([t.tcs_total for t in tasks])
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
            "operating_mode": t.operating_mode,
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

    loaded = load_tasks(args.tasks_dir)
    if not loaded:
        LOG.error("No task IR (json/yaml) files found in %s", args.tasks_dir)
        return 2
    LOG.info("Loaded %d task IR files", len(loaded))

    irs = [ir for _, ir in loaded]
    tasks = compute_complexities(irs, w_struct, w_safety, b_struct, b_safety)
    write_csv(tasks, args.output)
    LOG.info("Wrote %d task complexity rows to %s", len(tasks), args.output)

    if args.manifest:
        lo, hi = tertile_cuts([t.tcs_total for t in tasks])
        manifest = {
            "spec_version": cfg.get("version", "unknown"),
            "weights_config": str(args.weights_config),
            "weights_config_sha256": file_sha256(args.weights_config),
            "tcs_script_sha256": file_sha256(Path(__file__)),
            "tasks_count": len(tasks),
            "tertile_lo_cut": round(lo, 6),
            "tertile_hi_cut": round(hi, 6),
            "tertile_distribution": {
                t: sum(1 for x in tasks if x.tertile == t)
                for t in ("T_low", "T_med", "T_high")
            },
            "input_files": [str(p) for p, _ in loaded],
        }
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True))
        LOG.info("Wrote manifest to %s", args.manifest)

    return 0


if __name__ == "__main__":
    sys.exit(main())
