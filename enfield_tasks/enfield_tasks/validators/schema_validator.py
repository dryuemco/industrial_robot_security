"""Task IR schema validator — validates JSON task files against task_ir_v1.schema.json."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import jsonschema
from jsonschema import Draft202012Validator, ValidationError

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Schema loading
# ---------------------------------------------------------------------------

_SCHEMA_DIR = Path(__file__).resolve().parent.parent.parent / "ir" / "schema"
_SCHEMA_FILE = _SCHEMA_DIR / "task_ir_v1.schema.json"

# Fallback: installed via colcon data_files
_SHARE_SCHEMA_DIR = Path("/ros2_ws/install/enfield_tasks/share/enfield_tasks/ir/schema")


def _find_schema_path() -> Path:
    """Locate task_ir_v1.schema.json in dev or installed layout."""
    if _SCHEMA_FILE.exists():
        return _SCHEMA_FILE
    installed = _SHARE_SCHEMA_DIR / "task_ir_v1.schema.json"
    if installed.exists():
        return installed
    raise FileNotFoundError(
        f"Schema not found at {_SCHEMA_FILE} or {installed}. "
        "Ensure enfield_tasks is built or run from the source tree."
    )


def load_schema() -> dict[str, Any]:
    """Load and return the Task IR JSON Schema."""
    path = _find_schema_path()
    with open(path, "r") as f:
        schema = json.load(f)
    # Validate the schema itself
    Draft202012Validator.check_schema(schema)
    return schema


# Cache
_schema_cache: dict[str, Any] | None = None


def _get_schema() -> dict[str, Any]:
    global _schema_cache
    if _schema_cache is None:
        _schema_cache = load_schema()
    return _schema_cache


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


class TaskIRValidationResult:
    """Result of a Task IR validation run."""

    def __init__(self) -> None:
        self.schema_errors: list[str] = []
        self.semantic_warnings: list[str] = []

    @property
    def valid(self) -> bool:
        return len(self.schema_errors) == 0

    def summary(self) -> str:
        lines = []
        if self.valid:
            lines.append("PASS — Task IR is schema-valid.")
        else:
            lines.append(f"FAIL — {len(self.schema_errors)} schema error(s):")
            for e in self.schema_errors:
                lines.append(f"  - {e}")
        if self.semantic_warnings:
            lines.append(f"WARNINGS ({len(self.semantic_warnings)}):")
            for w in self.semantic_warnings:
                lines.append(f"  - {w}")
        return "\n".join(lines)


def validate_task_ir(task_data: dict[str, Any]) -> TaskIRValidationResult:
    """Validate a parsed Task IR dict against the JSON Schema.

    Args:
        task_data: Parsed JSON/YAML task dictionary.

    Returns:
        TaskIRValidationResult with schema errors and semantic warnings.
    """
    result = TaskIRValidationResult()
    schema = _get_schema()

    validator = Draft202012Validator(schema)

    for error in sorted(validator.iter_errors(task_data), key=lambda e: list(e.path)):
        path_str = ".".join(str(p) for p in error.absolute_path) or "(root)"
        result.schema_errors.append(f"[{path_str}] {error.message}")

    # Semantic checks (beyond what JSON Schema can express)
    if result.valid:
        result.semantic_warnings = _semantic_checks(task_data)

    return result


def _semantic_checks(task_data: dict[str, Any]) -> list[str]:
    """Run lightweight semantic validations on a schema-valid task."""
    warnings: list[str] = []

    mode = task_data["task"]["operating_mode"]
    safety = task_data.get("safety_requirements", {})

    # A1: Speed vs operating mode consistency
    max_speed = safety.get("max_tcp_speed_mm_s")
    if max_speed is not None:
        mode_limits = {
            "collaborative": 250.0,
            "fenced": 500.0,
            "hybrid": 300.0,
        }
        limit = mode_limits.get(mode, 500.0)
        if max_speed > limit:
            warnings.append(
                f"A1: max_tcp_speed_mm_s={max_speed} exceeds "
                f"{mode} mode limit ({limit} mm/s)"
            )

    # A2: Motion waypoints vs safeguarded space (static spot-check)
    sg = safety.get("safeguarded_space")
    if sg:
        halfspaces = sg["halfspaces"]
        for cmd in task_data.get("motion_sequence", []):
            tp = cmd.get("target_pose")
            if tp and "position" in tp:
                pos = tp["position"]
                p = [pos["x"], pos["y"], pos["z"]]
                for hs in halfspaces:
                    n = hs["normal"]
                    d = hs["offset"]
                    dot = sum(ni * pi for ni, pi in zip(n, p))
                    if dot > d:
                        warnings.append(
                            f"A2: motion seq={cmd['seq']} waypoint {p} "
                            f"violates halfspace n={n}, d={d} "
                            f"(dot={dot:.1f} > {d})"
                        )

    # A4: Payload vs robot adapter limits
    payload = task_data.get("robot", {}).get("payload_kg")
    tp = safety.get("tool_payload")
    if payload is not None and tp is not None:
        if payload < tp.get("mass_min_kg", 0):
            warnings.append(
                f"A4: robot.payload_kg={payload} < tool_payload.mass_min_kg={tp['mass_min_kg']}"
            )
        if payload > tp.get("mass_max_kg", float("inf")):
            warnings.append(
                f"A4: robot.payload_kg={payload} > tool_payload.mass_max_kg={tp['mass_max_kg']}"
            )

    # A5: E-Stop required in collaborative mode
    if mode == "collaborative" and not safety.get("estop_required", True):
        warnings.append("A5: collaborative mode should have estop_required=true (ISO 5.4)")

    # A5: Check that estop_check command exists in motion_sequence
    seq_types = [c["type"] for c in task_data.get("motion_sequence", [])]
    if safety.get("estop_required", True) and "estop_check" not in seq_types:
        warnings.append(
            "A5: estop_required=true but no 'estop_check' command in motion_sequence"
        )

    # A6: Frame consistency — motion frame_ids must exist in environment.frames
    defined_frames = set(task_data.get("environment", {}).get("frames", {}).keys())
    for cmd in task_data.get("motion_sequence", []):
        fid = cmd.get("frame_id")
        if fid is not None and fid not in defined_frames:
            warnings.append(
                f"A6: motion seq={cmd['seq']} references frame '{fid}' "
                f"not defined in environment.frames"
            )

    # Sequence ordering check
    seqs = [c["seq"] for c in task_data.get("motion_sequence", [])]
    if seqs != sorted(seqs):
        warnings.append("motion_sequence 'seq' values are not strictly ascending")
    if len(set(seqs)) != len(seqs):
        warnings.append("motion_sequence contains duplicate 'seq' values")

    return warnings


# ---------------------------------------------------------------------------
# File-level helpers
# ---------------------------------------------------------------------------


def validate_file(filepath: str | Path) -> TaskIRValidationResult:
    """Load a JSON file and validate it.

    Args:
        filepath: Path to a .json Task IR file.

    Returns:
        TaskIRValidationResult.
    """
    path = Path(filepath)
    if not path.exists():
        result = TaskIRValidationResult()
        result.schema_errors.append(f"File not found: {path}")
        return result

    with open(path, "r") as f:
        data = json.load(f)

    return validate_task_ir(data)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """CLI: validate one or more Task IR files."""
    import sys

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if len(sys.argv) < 2:
        print("Usage: python -m enfield_tasks.validators.schema_validator <file.json> ...")
        sys.exit(1)

    all_valid = True
    for fpath in sys.argv[1:]:
        logger.info(f"\n=== Validating: {fpath} ===")
        result = validate_file(fpath)
        logger.info(result.summary())
        if not result.valid:
            all_valid = False

    sys.exit(0 if all_valid else 1)


if __name__ == "__main__":
    main()
