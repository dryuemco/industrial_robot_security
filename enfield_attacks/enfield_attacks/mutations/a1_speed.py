# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A1 — Speed Injection: inflate speed_mm_s beyond operating mode limit."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult


class A1SpeedInjection(Mutation):
    """Multiply all motion speeds by a factor that exceeds max_tcp_speed_mm_s.

    Strategy: set every speed_mm_s to ``max_tcp_speed_mm_s * multiplier``.
    Default multiplier 2.5× guarantees a clear violation.
    """

    attack_id = "A1"
    iso_clause = "5.6"
    description = "Speed Injection — TCP speed exceeds operating mode limit"

    def __init__(self, seed: int = 42, multiplier: float = 2.5) -> None:
        super().__init__(seed)
        self.multiplier = multiplier

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        limit = self._get_speed_limit(task)
        injected_speed = round(limit * self.multiplier, 1)
        mutated_fields: list[str] = []

        for cmd in task.get("motion_sequence", []):
            if "speed_mm_s" in cmd:
                cmd["speed_mm_s"] = injected_speed
                mutated_fields.append(f"motion_sequence[{cmd['seq']}].speed_mm_s")

        severity = (injected_speed - limit) / limit if limit > 0 else 0.0

        result = MutationResult(
            attack_type="A1",
            iso_clause=self.iso_clause,
            description=f"All speeds set to {injected_speed} mm/s (limit: {limit} mm/s)",
            mutated_fields=mutated_fields,
            severity_estimate=round(severity, 2),
            metadata={"injected_speed": injected_speed, "limit": limit, "multiplier": self.multiplier},
        )
        return task, result
