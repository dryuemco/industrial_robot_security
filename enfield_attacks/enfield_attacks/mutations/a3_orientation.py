# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A3 — Orientation Anomaly: rotate tool into forbidden orientation cone."""

from __future__ import annotations

import math
from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult


class A3OrientationAnomaly(Mutation):
    """Flip quaternion so tool Z-axis points into the first forbidden cone.

    Strategy: if a forbidden cone has axis [0,0,1] (upward), set the
    quaternion to identity [qx=0,qy=0,qz=0,qw=1] so tool Z points +Z_world.
    If no cones exist, rotate tool to point straight up as a generic hazard.
    """

    attack_id = "A3"
    iso_clause = "5.3"
    description = "Orientation Anomaly — tool points into forbidden direction cone"

    # Quaternions that produce specific tool-Z directions:
    # identity [0,0,0,1] → tool Z = +Z_world (upward)
    # [0,1,0,0] → tool Z = -Z_world (downward) — usually safe
    # [0, 0.707, 0, 0.707] → tool Z = +X_world

    _UPWARD_QUAT = {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    _PLUS_X_QUAT = {"qx": 0.0, "qy": 0.7071, "qz": 0.0, "qw": 0.7071}
    _PLUS_Y_QUAT = {"qx": -0.7071, "qy": 0.0, "qz": 0.0, "qw": 0.7071}

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        cones = task.get("safety_requirements", {}).get("forbidden_orientation_cones", [])
        mutated_fields: list[str] = []
        target_quat = self._UPWARD_QUAT  # default: point up
        cone_label = "upward (default)"

        # Pick quaternion that violates the first cone
        if cones:
            axis = cones[0].get("axis", [0, 0, 1])
            cone_label = cones[0].get("label", str(axis))
            if axis == [0, 0, 1] or axis == [0.0, 0.0, 1.0]:
                target_quat = self._UPWARD_QUAT
            elif axis == [1, 0, 0] or axis == [1.0, 0.0, 0.0]:
                target_quat = self._PLUS_X_QUAT
            elif axis == [0, 1, 0] or axis == [0.0, 1.0, 0.0]:
                target_quat = self._PLUS_Y_QUAT
            elif axis == [0, -1, 0] or axis == [0.0, -1.0, 0.0]:
                # -Y cone → point tool toward -Y
                target_quat = {"qx": 0.7071, "qy": 0.0, "qz": 0.0, "qw": 0.7071}
            else:
                target_quat = self._UPWARD_QUAT

        # Apply to all move_linear commands with orientation
        for cmd in task.get("motion_sequence", []):
            tp = cmd.get("target_pose")
            if tp and "orientation" in tp:
                tp["orientation"] = dict(target_quat)
                mutated_fields.append(f"motion_sequence[{cmd['seq']}].target_pose.orientation")

        half_angle = cones[0]["half_angle_deg"] if cones else 45.0
        result = MutationResult(
            attack_type="A3",
            iso_clause=self.iso_clause,
            description=f"All orientations set to violate cone '{cone_label}'",
            mutated_fields=mutated_fields,
            severity_estimate=half_angle,
            metadata={"target_quat": target_quat, "cone_label": cone_label},
        )
        return task, result
