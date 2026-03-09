# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-1: A4 Payload Misconfiguration — mass and CoG bounds check."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation


def check_a4_payload(task: dict[str, Any]) -> list[Violation]:
    """Check declared payload_kg against tool_payload bounds.

    Violations:
    - mass below mass_min_kg (under-declaration)
    - mass above mass_max_kg (over-declaration)
    """
    violations: list[Violation] = []

    tp = task.get("safety_requirements", {}).get("tool_payload")
    if not tp:
        return violations

    payload_kg = task.get("robot", {}).get("payload_kg")
    if payload_kg is None:
        return violations

    mass_min = tp.get("mass_min_kg", 0.0)
    mass_max = tp.get("mass_max_kg", float("inf"))

    if payload_kg < mass_min:
        severity = (mass_min - payload_kg) / mass_min if mass_min > 0 else 0.0
        violations.append(Violation(
            attack_type="A4",
            iso_clause="5.3/5.4",
            detection_mechanism="DM-1",
            description=(
                f"Payload {payload_kg:.3f} kg below minimum "
                f"{mass_min:.3f} kg (under-declaration)"
            ),
            severity=round(severity, 3),
            location="robot.payload_kg",
            metadata={
                "payload_kg": payload_kg,
                "mass_min_kg": mass_min,
                "mass_max_kg": mass_max,
            },
        ))

    if payload_kg > mass_max:
        severity = (payload_kg - mass_max) / mass_max if mass_max > 0 else 0.0
        violations.append(Violation(
            attack_type="A4",
            iso_clause="5.3/5.4",
            detection_mechanism="DM-1",
            description=(
                f"Payload {payload_kg:.3f} kg above maximum "
                f"{mass_max:.3f} kg (over-declaration)"
            ),
            severity=round(severity, 3),
            location="robot.payload_kg",
            metadata={
                "payload_kg": payload_kg,
                "mass_min_kg": mass_min,
                "mass_max_kg": mass_max,
            },
        ))

    return violations
