# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-1: A1 Speed Injection — check motion speeds against operating mode limit."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation

_MODE_LIMITS = {
    "collaborative": 250.0,
    "fenced": 500.0,
    "hybrid": 300.0,
}


def check_a1_speed(task: dict[str, Any]) -> list[Violation]:
    """Check all motion command speeds against max_tcp_speed_mm_s.

    Detection: compare each command's speed_mm_s against the task's
    safety_requirements.max_tcp_speed_mm_s (or mode default).
    """
    violations: list[Violation] = []

    mode = task.get("task", {}).get("operating_mode", "collaborative")
    safety = task.get("safety_requirements", {})
    limit = safety.get("max_tcp_speed_mm_s", _MODE_LIMITS.get(mode, 500.0))

    for cmd in task.get("motion_sequence", []):
        speed = cmd.get("speed_mm_s")
        if speed is not None and speed > limit:
            severity = (speed - limit) / limit if limit > 0 else 0.0
            violations.append(Violation(
                attack_type="A1",
                iso_clause="5.5.3",
                detection_mechanism="DM-1",
                description=(
                    f"Speed {speed:.1f} mm/s exceeds {mode} limit "
                    f"{limit:.1f} mm/s"
                ),
                severity=round(severity, 3),
                location=f"motion_sequence[{cmd['seq']}].speed_mm_s",
                metadata={
                    "speed_mm_s": speed,
                    "limit_mm_s": limit,
                    "operating_mode": mode,
                },
            ))

    return violations
