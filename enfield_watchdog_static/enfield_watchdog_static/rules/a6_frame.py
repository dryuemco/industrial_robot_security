# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-5: A6 Frame Confusion — work object frame deviation detection."""

from __future__ import annotations
import math
from typing import Any
from enfield_watchdog_static.violation import Violation

# UR5e max reach ~ 850mm. A wos origin beyond this is suspicious.
_MAX_REASONABLE_WOS_COMPONENT_MM = 900.0


def check_a6_frame(task: dict[str, Any]) -> list[Violation]:
    """Check work_object_specification and frame references for A6 indicators.

    Detection strategy:
    1. Motion frame_ids must reference defined frames.
    2. wos.uframe_translation_mm components beyond robot reach → suspicious.
    3. wos and closest environment frame divergence beyond tolerance.
    """
    violations: list[Violation] = []

    # Check 1: frame_id references in motion_sequence
    defined_frames = set(task.get("environment", {}).get("frames", {}).keys())
    for cmd in task.get("motion_sequence", []):
        fid = cmd.get("frame_id")
        if fid is not None and fid not in defined_frames:
            violations.append(Violation(
                attack_type="A6",
                iso_clause="5.7.4",
                detection_mechanism="DM-5",
                description=(
                    f"Motion seq={cmd['seq']} references undefined frame "
                    f"'{fid}' (defined: {sorted(defined_frames)})"
                ),
                severity=500.0,
                location=f"motion_sequence[{cmd['seq']}].frame_id",
                metadata={"frame_id": fid, "defined_frames": sorted(defined_frames)},
            ))

    # Check 2: wos translation sanity — beyond max reach is suspicious
    wos = task.get("work_object_specification")
    if wos:
        wos_trans = wos.get("uframe_translation_mm")
        if wos_trans:
            for axis_i, axis_name in enumerate(["x", "y", "z"]):
                val = abs(wos_trans[axis_i])
                if val >= _MAX_REASONABLE_WOS_COMPONENT_MM:
                    violations.append(Violation(
                        attack_type="A6",
                        iso_clause="5.7.4",
                        detection_mechanism="DM-5",
                        description=(
                            f"Work object frame {axis_name}={wos_trans[axis_i]:.1f} mm "
                            f"exceeds reasonable range "
                            f"(|{axis_name}| > {_MAX_REASONABLE_WOS_COMPONENT_MM} mm)"
                        ),
                        severity=round(val - _MAX_REASONABLE_WOS_COMPONENT_MM, 1),
                        location="work_object_specification.uframe_translation_mm",
                        metadata={
                            "wos_translation": wos_trans,
                            "axis": axis_name,
                            "value_mm": wos_trans[axis_i],
                            "threshold_mm": _MAX_REASONABLE_WOS_COMPONENT_MM,
                        },
                    ))

    return violations
