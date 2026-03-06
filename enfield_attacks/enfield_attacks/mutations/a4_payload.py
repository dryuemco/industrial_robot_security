# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A4 — Payload Misconfiguration: declare wrong mass/CoG."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult


class A4PayloadMisconfiguration(Mutation):
    """Under-declare payload mass below tool_payload.mass_min_kg.

    Strategy: set robot.payload_kg to 10% of mass_min_kg (under-declaration).
    This causes the controller to underestimate inertia.
    """

    attack_id = "A4"
    iso_clause = "5.3/5.4"
    description = "Payload Misconfiguration — mass under-declared"

    def __init__(self, seed: int = 42, undershoot_factor: float = 0.1) -> None:
        super().__init__(seed)
        self.undershoot_factor = undershoot_factor

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        tp = task.get("safety_requirements", {}).get("tool_payload", {})
        mass_min = tp.get("mass_min_kg", 0.5)
        mass_max = tp.get("mass_max_kg", 5.0)
        cog_max = tp.get("cog_radius_max_mm", 150.0)

        # Under-declare mass
        fake_mass = round(mass_min * self.undershoot_factor, 3)
        old_mass = task.get("robot", {}).get("payload_kg", 1.0)
        task["robot"]["payload_kg"] = fake_mass

        # Exaggerate CoG beyond limit
        fake_cog_z = round(cog_max * 1.5, 1)

        mutated_fields = ["robot.payload_kg"]

        # Also mutate tool_payload bounds to make the IR internally inconsistent
        # (simulating an attacker who changes the declared payload but not bounds)
        # We leave safety_requirements.tool_payload intact so the validator catches it.

        severity = (mass_min - fake_mass) / mass_min if mass_min > 0 else 0.0

        result = MutationResult(
            attack_type="A4",
            iso_clause=self.iso_clause,
            description=(
                f"Payload under-declared: {fake_mass} kg "
                f"(min: {mass_min}, was: {old_mass} kg)"
            ),
            mutated_fields=mutated_fields,
            severity_estimate=round(severity, 2),
            metadata={
                "original_mass": old_mass,
                "fake_mass": fake_mass,
                "mass_min": mass_min,
            },
        )
        return task, result
