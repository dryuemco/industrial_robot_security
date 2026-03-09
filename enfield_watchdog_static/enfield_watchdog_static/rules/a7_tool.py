# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-6: A7 Tool Misuse — tool identity and activation context check."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation

# Expected tool_type mapping from tool.type → activation_constraints.tool_type
_EXPECTED_TOOL_TYPES = {
    "gripper": "gripper",
    "welder": "welding_arc",
    "suction": "suction",
    "laser": "laser",
    "screwdriver": "screwdriver",
    "none": None,
}


def check_a7_tool(task: dict[str, Any]) -> list[Violation]:
    """Check tool configuration for identity and context violations.

    Detection:
    1. Tool type mismatch: tool.type vs activation_constraints.tool_type
    2. Operating mode not in activation_constraints.allowed_modes
    """
    violations: list[Violation] = []

    tool = task.get("tool", {})
    tool_type = tool.get("type", "none")
    ac = tool.get("activation_constraints")

    if not ac:
        return violations

    op_mode = task.get("task", {}).get("operating_mode", "collaborative")

    # Check 1: tool identity
    declared_ac_type = ac.get("tool_type")
    expected_ac_type = _EXPECTED_TOOL_TYPES.get(tool_type)

    if (declared_ac_type is not None
            and expected_ac_type is not None
            and declared_ac_type != expected_ac_type):
        violations.append(Violation(
            attack_type="A7",
            iso_clause="5.1.14",
            detection_mechanism="DM-6",
            description=(
                f"Tool identity mismatch: tool.type='{tool_type}' expects "
                f"activation_constraints.tool_type='{expected_ac_type}' "
                f"but got '{declared_ac_type}'"
            ),
            severity=1.0,
            location="tool.activation_constraints.tool_type",
            metadata={
                "tool_type": tool_type,
                "expected_ac_type": expected_ac_type,
                "declared_ac_type": declared_ac_type,
            },
        ))

    # Check 2: operating mode allowed
    allowed_modes = ac.get("allowed_modes", [])
    if allowed_modes and op_mode not in allowed_modes:
        violations.append(Violation(
            attack_type="A7",
            iso_clause="5.1.15",
            detection_mechanism="DM-6",
            description=(
                f"Operating mode '{op_mode}' not in tool's "
                f"allowed_modes {allowed_modes}"
            ),
            severity=0.8,
            location="tool.activation_constraints.allowed_modes",
            metadata={
                "operating_mode": op_mode,
                "allowed_modes": allowed_modes,
            },
        ))

    return violations
