# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-6: Missing Safety Preamble — no safety configuration block.

Well-structured robot code starts with a safety preamble:
setting TCP, payload, speed limits, and tool configuration
before any motion commands. This rule checks for that pattern.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

# Safety preamble indicators — at least some should appear before motion
_PREAMBLE_PATTERNS = [
    r"\bset_tcp\s*\(",
    r"\bset_payload\s*\(",
    r"\bset_gravity\s*\(",
    r"\bset_tool_voltage\s*\(",
    r"\bset_standard_analog_out\s*\(",
]

_FIRST_MOTION_RE = re.compile(
    r"\b(movej|movel|movec|servoj|speedl|speedj)\s*\(",
    re.IGNORECASE,
)


def check_sm6_missing_preamble(code: str) -> list[Violation]:
    """Check that safety configuration appears before first motion command."""
    violations: list[Violation] = []

    lines = code.split("\n")

    # Find first motion command line number
    first_motion_line = None
    for i, line in enumerate(lines):
        if _FIRST_MOTION_RE.search(line):
            first_motion_line = i
            break

    if first_motion_line is None:
        # No motion commands — nothing to check
        return violations

    # Check if any preamble pattern appears before the first motion
    preamble_before_motion = code[:sum(
        len(l) + 1 for l in lines[:first_motion_line]
    )]

    has_tcp = bool(re.search(r"\bset_tcp\s*\(", preamble_before_motion, re.IGNORECASE))
    has_payload = bool(re.search(r"\bset_payload\s*\(", preamble_before_motion, re.IGNORECASE))

    if not has_tcp:
        violations.append(Violation(
            attack_type="SM-6",
            iso_clause="5.1.14",
            detection_mechanism="SM-6",
            description=(
                "No set_tcp() call before first motion command. "
                "Tool center point is undefined."
            ),
            severity=0.6,
            location=f"line:{first_motion_line + 1}",
            metadata={
                "cwe": "robot-specific",
                "missing": "set_tcp",
                "first_motion_line": first_motion_line + 1,
            },
        ))

    if not has_payload:
        violations.append(Violation(
            attack_type="SM-6",
            iso_clause="5.1.15",
            detection_mechanism="SM-6",
            description=(
                "No set_payload() call before first motion command. "
                "Payload mass/CoG is undefined, risking incorrect "
                "torque compensation."
            ),
            severity=0.6,
            location=f"line:{first_motion_line + 1}",
            metadata={
                "cwe": "robot-specific",
                "missing": "set_payload",
                "first_motion_line": first_motion_line + 1,
            },
        ))

    return violations
