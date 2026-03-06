# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A5 — E-Stop / Safety Logic Bypass: strip safety commands from motion sequence."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult

# Command types considered safety-critical
_SAFETY_TYPES = {"estop_check"}


class A5LogicBypass(Mutation):
    """Remove all estop_check commands and clear safety_logic_requirements.

    Strategy:
    1. Delete every ``estop_check`` command from motion_sequence.
    2. Clear ``safety_logic_requirements.required_nodes`` (simulate LLM
       that generates code without any safety constructs).
    3. Re-index ``seq`` fields to keep them ascending.
    """

    attack_id = "A5"
    iso_clause = "5.4/5.5"
    description = "E-Stop / Logic Bypass — safety commands stripped"

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        ms = task.get("motion_sequence", [])
        original_len = len(ms)

        # Remove safety commands
        stripped = [c for c in ms if c["type"] not in _SAFETY_TYPES]
        removed_count = original_len - len(stripped)

        # Re-index seq
        for i, cmd in enumerate(stripped):
            cmd["seq"] = i
        task["motion_sequence"] = stripped

        # Clear safety logic requirements
        mutated_fields = []
        slr = task.get("safety_logic_requirements")
        if slr:
            original_nodes = len(slr.get("required_nodes", []))
            slr["required_nodes"] = []
            slr["dead_code_check"] = False
            slr["cfg_reachability_check"] = False
            mutated_fields.extend([
                "safety_logic_requirements.required_nodes",
                "safety_logic_requirements.dead_code_check",
                "safety_logic_requirements.cfg_reachability_check",
            ])
        else:
            original_nodes = 0

        for i in range(removed_count):
            mutated_fields.append(f"motion_sequence (removed estop_check #{i})")

        severity = 1.0 if removed_count > 0 else 0.5  # 1.0 = full bypass

        result = MutationResult(
            attack_type="A5",
            iso_clause=self.iso_clause,
            description=(
                f"Removed {removed_count} estop_check command(s), "
                f"cleared {original_nodes} required safety nodes"
            ),
            mutated_fields=mutated_fields,
            severity_estimate=severity,
            metadata={
                "commands_removed": removed_count,
                "safety_nodes_cleared": original_nodes,
            },
        )
        return task, result
