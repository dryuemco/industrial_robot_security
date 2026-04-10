# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-3: CWE-693 Protection Mechanism Failure — safety interlock bypass.

Checks for patterns that disable or bypass safety mechanisms:
setting safety mode to reduced, disabling protective stop, etc.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

# Patterns that indicate deliberate safety mechanism bypass
_BYPASS_PATTERNS = [
    (r"\bset_safety_mode_transition_hardness\s*\(\s*[01]\s*\)",
     "set_safety_mode_transition_hardness set to minimum"),
    (r"\bunlock_protective_stop\s*\(",
     "Protective stop unlocked programmatically"),
    (r"\bset_watchdog\s*\(\s*-1\s*\)",
     "Watchdog timer disabled"),
    (r"\bsafeguard_reset\s*\(",
     "Safeguard reset called — may bypass safety state"),
    (r"\bset_reduced_mode\s*\(\s*False\s*\)",
     "Reduced mode explicitly disabled"),
    (r"#.*disable.*safety", "Comment suggests safety disabling intent"),
    (r"#.*bypass.*safety", "Comment suggests safety bypass intent"),
    (r"#.*ignore.*safety", "Comment suggests safety ignore intent"),
    (r"#.*skip.*safety", "Comment suggests safety skip intent"),
    (r"#.*no.*safety.*check", "Comment suggests skipping safety checks"),
]


def check_sm3_protection_failure(code: str) -> list[Violation]:
    """Check for patterns that disable or bypass safety mechanisms."""
    violations: list[Violation] = []

    for i, line in enumerate(code.split("\n"), 1):
        stripped = line.strip().lower()

        for pattern, desc in _BYPASS_PATTERNS:
            if re.search(pattern, line, re.IGNORECASE):
                violations.append(Violation(
                    attack_type="SM-3",
                    iso_clause="5.1.16",
                    detection_mechanism="SM-3",
                    description=f"Safety bypass: {desc}",
                    severity=0.9,
                    location=f"line:{i}",
                    metadata={
                        "cwe": "CWE-693",
                        "pattern": pattern,
                        "line": i,
                    },
                ))

    return violations
