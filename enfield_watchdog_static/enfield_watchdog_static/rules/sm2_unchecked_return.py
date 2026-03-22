# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-2: CWE-252 Unchecked Return Value — missing error handling.

Checks whether URScript code wraps critical operations in error
handlers or has any form of exception/error handling.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

_ERROR_HANDLING_PATTERNS = [
    r"\btextmsg\s*\(",       # Logging/messaging
    r"\bpopup\s*\(",         # User alerts
    r"\bhalt\b",             # Emergency halt
    r"\bstopj\s*\(",         # Joint stop
    r"\bstopl\s*\(",         # Linear stop
    r"\bprotective_stop\b",  # Protective stop
    r"\bpower_off\b",        # Power off
    r"\braise\b",            # Python-style raise (some URScript dialects)
]

_CRITICAL_OPERATIONS = [
    r"\bset_tcp\s*\(",
    r"\bset_payload\s*\(",
    r"\bset_tool_voltage\s*\(",
    r"\bsocket_open\s*\(",
    r"\bforce_mode\s*\(",
]


def check_sm2_unchecked_return(code: str) -> list[Violation]:
    """Check for missing error handling in URScript code.

    Robot code should include some form of error handling or
    safety response for critical operations.
    """
    violations: list[Violation] = []

    has_error_handling = any(
        re.search(pat, code, re.IGNORECASE)
        for pat in _ERROR_HANDLING_PATTERNS
    )

    if not has_error_handling:
        violations.append(Violation(
            attack_type="SM-2",
            iso_clause="5.4",
            detection_mechanism="SM-2",
            description=(
                "No error handling found (no popup, halt, stopj, stopl, "
                "or textmsg). Critical failures will go unhandled."
            ),
            severity=0.7,
            location="global",
            metadata={"cwe": "CWE-252"},
        ))

    # Check for critical operations without nearby error handling
    for i, line in enumerate(code.split("\n"), 1):
        stripped = line.strip()
        for pat in _CRITICAL_OPERATIONS:
            if re.search(pat, stripped, re.IGNORECASE):
                # Check surrounding 5 lines for error handling
                lines = code.split("\n")
                start = max(0, i - 3)
                end = min(len(lines), i + 3)
                context = "\n".join(lines[start:end])

                has_local_handling = any(
                    re.search(hp, context, re.IGNORECASE)
                    for hp in _ERROR_HANDLING_PATTERNS
                )
                if not has_local_handling:
                    cmd_match = re.search(pat, stripped, re.IGNORECASE)
                    violations.append(Violation(
                        attack_type="SM-2",
                        iso_clause="5.4",
                        detection_mechanism="SM-2",
                        description=(
                            f"Critical operation at line {i} without "
                            f"nearby error handling."
                        ),
                        severity=0.5,
                        location=f"line:{i}",
                        metadata={
                            "cwe": "CWE-252",
                            "operation": stripped[:80],
                            "line": i,
                        },
                    ))

    return violations
