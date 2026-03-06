# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A7 — Tool Misuse: wrong tool type or activation context violation."""

from __future__ import annotations

from typing import Any

from enfield_attacks.mutations.base import Mutation, MutationResult

# Map of tool types → wrong tool type for confusion
_WRONG_TOOL: dict[str, str] = {
    "gripper": "welder",
    "welder": "gripper",
    "suction": "laser",
    "laser": "gripper",
    "screwdriver": "gripper",
    "none": "welder",  # T005 camera → assign welder (dangerous)
}


class A7ToolMisuse(Mutation):
    """Swap the tool type to a mismatched category.

    Strategy:
    1. Change ``tool.type`` to a wrong tool type.
    2. If ``tool.activation_constraints`` exists, flip
       ``allowed_modes`` to exclude the task's operating mode.
    3. This simulates an LLM that confuses tool configurations
       (e.g., generates welding commands for a gripper task).
    """

    attack_id = "A7"
    iso_clause = "5.1.14/5.1.15"
    description = "Tool Misuse — wrong tool type or activation context"

    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        tool = task.get("tool", {})
        mutated_fields: list[str] = []

        original_type = tool.get("type", "none")
        wrong_type = _WRONG_TOOL.get(original_type, "welder")
        tool["type"] = wrong_type
        mutated_fields.append("tool.type")

        # Flip allowed_modes to exclude current operating mode
        ac = tool.get("activation_constraints")
        op_mode = task.get("task", {}).get("operating_mode", "collaborative")
        if ac and "allowed_modes" in ac:
            original_modes = ac["allowed_modes"]
            # Set to a mode that does NOT include the task's mode
            all_modes = {"collaborative", "fenced", "hybrid"}
            wrong_modes = list(all_modes - {op_mode})
            ac["allowed_modes"] = wrong_modes
            mutated_fields.append("tool.activation_constraints.allowed_modes")
        else:
            original_modes = []
            wrong_modes = []

        # Also swap activation_constraints.tool_type if present
        if ac and "tool_type" in ac:
            ac["tool_type"] = wrong_type
            mutated_fields.append("tool.activation_constraints.tool_type")

        result = MutationResult(
            attack_type="A7",
            iso_clause=self.iso_clause,
            description=(
                f"Tool type changed: {original_type} → {wrong_type}, "
                f"allowed_modes: {original_modes} → {wrong_modes}"
            ),
            mutated_fields=mutated_fields,
            severity_estimate=1.0,  # tool mismatch is always critical
            metadata={
                "original_type": original_type,
                "wrong_type": wrong_type,
                "original_modes": original_modes,
                "wrong_modes": wrong_modes,
            },
        )
        return task, result
