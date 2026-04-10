# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-5: CWE-798 Hardcoded Credentials — hardcoded unsafe parameters.

In robot context, "credentials" maps to hardcoded unsafe values:
speeds above collaborative limits, excessive forces, or dangerous
tool parameters embedded as literal numbers rather than named
constants or configuration lookups.
"""

from __future__ import annotations

import re
from typing import Any

from enfield_watchdog_static.violation import Violation

# Thresholds for unsafe hardcoded values
_SPEED_COLLAB_LIMIT = 0.250   # m/s — ISO 10218-1:2025 clause 5.5.3 collaborative limit
_SPEED_FENCED_LIMIT = 0.500   # m/s — typical fenced max
_ACCEL_LIMIT = 2.5            # rad/s² — aggressive acceleration
_FORCE_LIMIT = 150.0          # N — ISO/TS 15066 contact force limit

# Patterns: movej/movel with literal v= or a= values
_SPEED_LITERAL_RE = re.compile(
    r"\b(?:movej|movel|movec)\s*\([^)]*\bv\s*=\s*([0-9]*\.?[0-9]+)",
    re.IGNORECASE,
)

_ACCEL_LITERAL_RE = re.compile(
    r"\b(?:movej|movel|movec)\s*\([^)]*\ba\s*=\s*([0-9]*\.?[0-9]+)",
    re.IGNORECASE,
)

# force_mode with high wrench values
_FORCE_LITERAL_RE = re.compile(
    r"\bforce_mode\s*\([^)]*\[([^\]]+)\]",
    re.IGNORECASE,
)


def check_sm5_hardcoded_values(code: str) -> list[Violation]:
    """Check for hardcoded parameter values exceeding safe thresholds."""
    violations: list[Violation] = []

    for i, line in enumerate(code.split("\n"), 1):
        stripped = line.strip()
        if stripped.startswith("#") or stripped.startswith("//"):
            continue

        # Check hardcoded speed values
        for match in _SPEED_LITERAL_RE.finditer(stripped):
            speed = float(match.group(1))
            if speed > _SPEED_COLLAB_LIMIT:
                violations.append(Violation(
                    attack_type="SM-5",
                    iso_clause="5.1.16",
                    detection_mechanism="SM-5",
                    description=(
                        f"Hardcoded speed {speed} exceeds collaborative "
                        f"limit {_SPEED_COLLAB_LIMIT} m/s"
                    ),
                    severity=round(
                        min((speed - _SPEED_COLLAB_LIMIT) / _SPEED_COLLAB_LIMIT, 1.0),
                        3,
                    ),
                    location=f"line:{i}",
                    metadata={
                        "cwe": "CWE-798",
                        "value": speed,
                        "limit": _SPEED_COLLAB_LIMIT,
                        "line": i,
                    },
                ))

        # Check hardcoded acceleration values
        for match in _ACCEL_LITERAL_RE.finditer(stripped):
            accel = float(match.group(1))
            if accel > _ACCEL_LIMIT:
                violations.append(Violation(
                    attack_type="SM-5",
                    iso_clause="5.1.16",
                    detection_mechanism="SM-5",
                    description=(
                        f"Hardcoded acceleration {accel} rad/s² exceeds "
                        f"limit {_ACCEL_LIMIT} rad/s²"
                    ),
                    severity=round(
                        min((accel - _ACCEL_LIMIT) / _ACCEL_LIMIT, 1.0), 3,
                    ),
                    location=f"line:{i}",
                    metadata={
                        "cwe": "CWE-798",
                        "value": accel,
                        "limit": _ACCEL_LIMIT,
                        "line": i,
                    },
                ))

    return violations
