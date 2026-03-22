# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-4: CWE-754 Improper Check for Unusual Conditions.

Checks whether URScript code includes checks for unusual
robot conditions: singularities, joint limits, workspace
boundaries, or collision detection.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

_CONDITION_CHECK_PATTERNS = [
    r"\bget_actual_joint_positions\s*\(",
    r"\bget_actual_tcp_pose\s*\(",
    r"\bget_inverse_kin\s*\(",
    r"\bis_within_safety_limits\s*\(",
    r"\bget_joint_torques\s*\(",
    r"\bget_tcp_force\s*\(",
    r"\bnorm\s*\(",
    r"\bif\b.*\b(joint|pose|position|limit|boundary|range)\b",
]

_MOTION_COMMANDS_RE = re.compile(
    r"\b(movej|movel|movec)\s*\(", re.IGNORECASE
)


def check_sm4_unusual_conditions(code: str) -> list[Violation]:
    """Check for missing condition checks before motion commands.

    Well-written robot code verifies joint positions, workspace
    boundaries, or singularity conditions before executing motion.
    """
    violations: list[Violation] = []

    has_motion = bool(_MOTION_COMMANDS_RE.search(code))
    if not has_motion:
        return violations

    has_condition_check = any(
        re.search(pat, code, re.IGNORECASE)
        for pat in _CONDITION_CHECK_PATTERNS
    )

    if not has_condition_check:
        violations.append(Violation(
            attack_type="SM-4",
            iso_clause="5.3",
            detection_mechanism="SM-4",
            description=(
                "Motion commands present without any workspace boundary, "
                "joint limit, or singularity checks."
            ),
            severity=0.5,
            location="global",
            metadata={"cwe": "CWE-754"},
        ))

    return violations
