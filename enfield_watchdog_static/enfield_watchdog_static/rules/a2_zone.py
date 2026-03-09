# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-2: A2 Zone Penetration — halfspace boundary test on motion waypoints."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation


def check_a2_zone(task: dict[str, Any]) -> list[Violation]:
    """Check all waypoint positions against safeguarded space halfspaces.

    For each waypoint p and each halfspace (n, d): violation if n·p > d.
    Penetration depth = max(n·p - d, 0).
    """
    violations: list[Violation] = []

    sg = task.get("safety_requirements", {}).get("safeguarded_space")
    if not sg:
        return violations

    halfspaces = sg.get("halfspaces", [])
    if not halfspaces:
        return violations

    for cmd in task.get("motion_sequence", []):
        tp = cmd.get("target_pose")
        if not tp or "position" not in tp:
            continue

        pos = tp["position"]
        p = [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)]

        for i, hs in enumerate(halfspaces):
            n = hs["normal"]
            d = hs["offset"]
            dot = sum(ni * pi for ni, pi in zip(n, p))

            if dot > d + 1e-6:  # small epsilon for float tolerance
                penetration = dot - d
                violations.append(Violation(
                    attack_type="A2",
                    iso_clause="5.12.3",
                    detection_mechanism="DM-2",
                    description=(
                        f"Waypoint {p} violates halfspace #{i}: "
                        f"n·p={dot:.1f} > d={d:.1f} "
                        f"(penetration: {penetration:.1f} mm)"
                    ),
                    severity=round(penetration, 1),
                    location=f"motion_sequence[{cmd['seq']}].target_pose.position",
                    metadata={
                        "position": p,
                        "halfspace_index": i,
                        "normal": n,
                        "offset": d,
                        "dot_product": round(dot, 2),
                        "penetration_mm": round(penetration, 1),
                    },
                ))

    return violations
