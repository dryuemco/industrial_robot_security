# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-1: CWE-20 Improper Input Validation — unchecked motion parameters.

Checks whether URScript code uses motion commands (movej, movel, etc.)
without explicit speed/acceleration parameter values, relying on
potentially unsafe defaults.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

# Motion commands that should have explicit speed/accel parameters
_MOTION_COMMANDS = ["movej", "movel", "movec", "servoj", "speedl", "speedj"]

# Patterns for motion calls with missing parameters
_MOTION_CALL_RE = re.compile(
    r"\b(movej|movel|movec)\s*\("
    r"[^)]*"
    r"\)",
    re.IGNORECASE,
)


def check_sm1_input_validation(code: str) -> list[Violation]:
    """Check for motion commands without explicit speed/accel parameters.

    URScript motion commands accept optional a= and v= parameters.
    Omitting them uses robot defaults which may exceed safe limits.
    """
    violations: list[Violation] = []

    for i, line in enumerate(code.split("\n"), 1):
        stripped = line.strip()
        if stripped.startswith("#") or stripped.startswith("//"):
            continue

        for match in _MOTION_CALL_RE.finditer(stripped):
            call_text = match.group(0)
            cmd_name = match.group(1).lower()

            has_speed = bool(re.search(r"\bv\s*=", call_text))
            has_accel = bool(re.search(r"\ba\s*=", call_text))

            if not has_speed:
                violations.append(Violation(
                    attack_type="SM-1",
                    iso_clause="5.1.16",
                    detection_mechanism="SM-1",
                    description=(
                        f"{cmd_name}() without explicit speed parameter (v=). "
                        f"Robot default may exceed safe limits."
                    ),
                    severity=0.6,
                    location=f"line:{i}",
                    metadata={
                        "cwe": "CWE-20",
                        "command": cmd_name,
                        "missing": "speed",
                        "line": i,
                    },
                ))

            if not has_accel:
                violations.append(Violation(
                    attack_type="SM-1",
                    iso_clause="5.1.16",
                    detection_mechanism="SM-1",
                    description=(
                        f"{cmd_name}() without explicit acceleration parameter (a=). "
                        f"Robot default may exceed safe limits."
                    ),
                    severity=0.4,
                    location=f"line:{i}",
                    metadata={
                        "cwe": "CWE-20",
                        "command": cmd_name,
                        "missing": "acceleration",
                        "line": i,
                    },
                ))

    return violations
