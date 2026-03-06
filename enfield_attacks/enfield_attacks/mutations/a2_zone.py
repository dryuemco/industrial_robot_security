# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A2 — Zone Penetration: push waypoints outside safeguarded space."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult


class A2ZonePenetration(Mutation):
    """Shift the first waypoint with a target_pose beyond the safeguarded space.

    Strategy: find the tightest halfspace constraint, then push the
    corresponding coordinate past the boundary by ``margin_mm``.
    """

    attack_id = "A2"
    iso_clause = "5.12.3"
    description = "Zone Penetration — waypoint outside safeguarded space"

    def __init__(self, seed: int = 42, margin_mm: float = 150.0) -> None:
        super().__init__(seed)
        self.margin_mm = margin_mm

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        sg = task.get("safety_requirements", {}).get("safeguarded_space", {})
        halfspaces = sg.get("halfspaces", [])
        mutated_fields: list[str] = []
        penetration_mm = 0.0

        # Find first motion command with target_pose
        for cmd in task.get("motion_sequence", []):
            tp = cmd.get("target_pose")
            if tp and "position" in tp:
                pos = tp["position"]

                # Pick the first halfspace with a positive single-axis normal
                # (e.g. [1,0,0] means x <= offset) and push past it
                for hs in halfspaces:
                    n = hs["normal"]
                    d = hs["offset"]
                    # Find dominant axis
                    for axis, key in enumerate(["x", "y", "z"]):
                        if abs(n[axis]) == 1.0 and all(n[j] == 0 for j in range(3) if j != axis):
                            if n[axis] > 0:
                                # constraint: pos[key] <= d → violate by setting pos[key] = d + margin
                                new_val = d + self.margin_mm
                            else:
                                # constraint: -pos[key] <= d → pos[key] >= -d → violate: pos[key] = -d - margin
                                new_val = -d - self.margin_mm

                            old_val = pos[key]
                            pos[key] = round(new_val, 1)
                            penetration_mm = self.margin_mm
                            mutated_fields.append(
                                f"motion_sequence[{cmd['seq']}].target_pose.position.{key}"
                            )
                            break
                    if mutated_fields:
                        break
                break  # Only mutate first waypoint

        result = MutationResult(
            attack_type="A2",
            iso_clause=self.iso_clause,
            description=f"Waypoint pushed {self.margin_mm} mm outside safeguarded space",
            mutated_fields=mutated_fields,
            severity_estimate=penetration_mm,
            metadata={"margin_mm": self.margin_mm},
        )
        return task, result
