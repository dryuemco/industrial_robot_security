# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""CANDIDATE corrected SM-5 rule — NOT registered in ALL_SECURITY_RULES.

This is a reviewable alternative to enfield_watchdog_static .rules.
sm5_hardcoded_values, addressing two over-counting modes that cause the active
SM-5 to flag every validated-safe baseline (41/41 false positives; see
scripts/sm5_audit.py):

  (A) UNIT CONFLATION   -- the active rule applies the 0.25 m/s TCP limit to
      movej v, which is a JOINT speed in rad/s. This rule applies the TCP limit
      only to movel/movec, and a separate rad/s limit to movej.
  (B) MODE INSENSITIVITY -- the active rule hardcodes 0.25 m/s regardless of the
      task's operating mode. This rule accepts a per-task `tcp_cap` (e.g.
      collaborative 0.25, fenced 0.40-0.45). When the cap is unknown it falls
      back to 0.25 (collaborative worst-case), preserving the conservative
      default for callers that do not supply a mode.

DECISION REQUIRED (Yunus + Georgios): adopting this rule changes the SM scoring
basis for the PREREGISTERED confirmatory experiments E1-E3 (H4-H6 are entirely
SM-driven). That requires re-running/re-scoring E1-E3 and an OSF amendment
(Georgios holds OSF authority). This module is provided so the decision is
informed and execution is ready; it is intentionally NOT wired into the
watchdog, so no preregistered result changes silently.
"""

from __future__ import annotations

import re

from enfield_watchdog_static.violation import Violation

# Mode-dependent TCP (Cartesian) speed limits, m/s.
TCP_LIMIT_COLLABORATIVE = 0.250   # ISO 10218-1:2025 clause 5.5.3
TCP_LIMIT_FENCED = 0.500          # typical fenced max
# Joint-space speed limit for movej, rad/s. UR5e wrist joints reach ~pi rad/s;
# a normal program move (~0.5 rad/s) must not fire. Flag only clearly excessive
# joint speeds.
JOINT_SPEED_LIMIT = 3.15          # rad/s (~UR5e max), separate units from TCP
_ACCEL_LIMIT = 2.5                # rad/s^2

_MOVEL_V = re.compile(r"\bmove[lc]\s*\([^)]*\bv\s*=\s*([0-9]*\.?[0-9]+)", re.I)
_MOVEJ_V = re.compile(r"\bmovej\s*\([^)]*\bv\s*=\s*([0-9]*\.?[0-9]+)", re.I)
_ACCEL = re.compile(r"\b(?:movej|movel|movec)\s*\([^)]*\ba\s*=\s*([0-9]*\.?[0-9]+)", re.I)


def check_sm5_corrected(code: str, tcp_cap: float | None = None) -> list[Violation]:
    """Mode-aware, unit-correct SM-5.

    Args:
        code: URScript source.
        tcp_cap: the task's TCP-speed cap (m/s). Defaults to the collaborative
            0.25 m/s when not supplied (conservative).
    """
    cap = TCP_LIMIT_COLLABORATIVE if tcp_cap is None else tcp_cap
    violations: list[Violation] = []

    for i, line in enumerate(code.split("\n"), 1):
        s = line.strip()
        if s.startswith("#") or s.startswith("//"):
            continue

        # Cartesian TCP speed (m/s) vs the task's mode cap.
        for m in _MOVEL_V.finditer(s):
            v = float(m.group(1))
            if v > cap + 1e-9:
                violations.append(Violation(
                    attack_type="SM-5", iso_clause="5.1.16",
                    detection_mechanism="SM-5",
                    description=(f"Hardcoded TCP speed {v} m/s exceeds "
                                 f"mode cap {cap} m/s"),
                    severity=round(min((v - cap) / max(cap, 1e-6), 1.0), 3),
                    location=f"line:{i}",
                    metadata={"cwe": "CWE-798", "value": v, "limit": cap,
                              "kind": "tcp_movel", "line": i},
                ))

        # Joint speed (rad/s) vs a joint limit — separate units from TCP.
        for m in _MOVEJ_V.finditer(s):
            v = float(m.group(1))
            if v > JOINT_SPEED_LIMIT + 1e-9:
                violations.append(Violation(
                    attack_type="SM-5", iso_clause="5.1.16",
                    detection_mechanism="SM-5",
                    description=(f"Hardcoded joint speed {v} rad/s exceeds "
                                 f"limit {JOINT_SPEED_LIMIT} rad/s"),
                    severity=round(min((v - JOINT_SPEED_LIMIT)
                                       / JOINT_SPEED_LIMIT, 1.0), 3),
                    location=f"line:{i}",
                    metadata={"cwe": "CWE-798", "value": v,
                              "limit": JOINT_SPEED_LIMIT,
                              "kind": "joint_movej", "line": i},
                ))

        # Acceleration (rad/s^2) — unchanged from the active rule.
        for m in _ACCEL.finditer(s):
            a = float(m.group(1))
            if a > _ACCEL_LIMIT + 1e-9:
                violations.append(Violation(
                    attack_type="SM-5", iso_clause="5.1.16",
                    detection_mechanism="SM-5",
                    description=(f"Hardcoded acceleration {a} rad/s^2 exceeds "
                                 f"limit {_ACCEL_LIMIT} rad/s^2"),
                    severity=round(min((a - _ACCEL_LIMIT) / _ACCEL_LIMIT, 1.0), 3),
                    location=f"line:{i}",
                    metadata={"cwe": "CWE-798", "value": a, "limit": _ACCEL_LIMIT,
                              "kind": "accel", "line": i},
                ))

    return violations
