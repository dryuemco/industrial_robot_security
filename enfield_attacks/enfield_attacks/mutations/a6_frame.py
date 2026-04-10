# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A6 — Frame Confusion: shift work object frame to corrupt resolved positions."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult


class A6FrameConfusion(Mutation):
    """Shift the work_object_specification uframe by a large offset.

    Strategy: add ``shift_mm`` to the X component of
    ``work_object_specification.uframe_translation_mm``. Also shift
    the corresponding frame in ``environment.frames`` to keep
    the IR internally consistent (the *declared* frame is wrong,
    simulating a real-world frame calibration attack).
    """

    attack_id = "A6"
    iso_clause = "5.7.4"
    description = "Frame Confusion — work object frame shifted"

    def __init__(self, seed: int = 42, shift_mm: float = 500.0) -> None:
        super().__init__(seed)
        self.shift_mm = shift_mm

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        mutated_fields: list[str] = []

        # Shift work_object_specification
        wos = task.get("work_object_specification")
        original_translation = None
        if wos and "uframe_translation_mm" in wos:
            original_translation = list(wos["uframe_translation_mm"])
            wos["uframe_translation_mm"][0] += self.shift_mm
            mutated_fields.append("work_object_specification.uframe_translation_mm")

        # Also shift the matching frame in environment.frames
        wobj_name = wos.get("wobj_name", "") if wos else ""
        frames = task.get("environment", {}).get("frames", {})
        # Try to find frame that matches wobj_name pattern
        for fname, fdata in frames.items():
            if fname == "world" or fname == "base_link":
                continue
            transform = fdata.get("transform")
            if transform and "position" in transform:
                transform["position"]["x"] += self.shift_mm
                mutated_fields.append(f"environment.frames.{fname}.transform.position.x")
                break  # Only shift first non-trivial frame

        severity = self.shift_mm  # mm displacement

        result = MutationResult(
            attack_type="A6",
            iso_clause=self.iso_clause,
            description=(
                f"Work object frame shifted +{self.shift_mm} mm in X "
                f"(was: {original_translation})"
            ),
            mutated_fields=mutated_fields,
            severity_estimate=severity,
            metadata={
                "shift_mm": self.shift_mm,
                "original_translation": original_translation,
                "wobj_name": wobj_name,
            },
        )
        return task, result
