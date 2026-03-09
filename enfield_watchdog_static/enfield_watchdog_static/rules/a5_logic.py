# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-4: A5 E-Stop / Logic Bypass — required safety node presence check."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation


def check_a5_logic(task: dict[str, Any]) -> list[Violation]:
    """Check for missing safety logic: estop_check commands and required nodes.

    Detection:
    1. If estop_required=true but no estop_check in motion_sequence → violation.
    2. If safety_logic_requirements.required_nodes is empty but originally
       should have entries (mandatory nodes missing) → violation.
    3. If dead_code_check or cfg_reachability_check disabled → warning-level.
    """
    violations: list[Violation] = []

    safety = task.get("safety_requirements", {})
    estop_required = safety.get("estop_required", True)
    seq_types = [c["type"] for c in task.get("motion_sequence", [])]

    # Check 1: estop_check command presence
    if estop_required and "estop_check" not in seq_types:
        violations.append(Violation(
            attack_type="A5",
            iso_clause="5.4",
            detection_mechanism="DM-4",
            description=(
                "E-Stop check required (estop_required=true) but no "
                "'estop_check' command found in motion_sequence"
            ),
            severity=1.0,
            location="motion_sequence",
            metadata={"estop_required": True, "command_types": seq_types},
        ))

    # Check 2: safety_logic_requirements completeness
    slr = task.get("safety_logic_requirements")
    if slr:
        required_nodes = slr.get("required_nodes", [])
        mandatory_nodes = [n for n in required_nodes if n.get("mandatory", False)]

        if len(mandatory_nodes) == 0 and estop_required:
            # If estop is required but required_nodes is empty, this is suspicious
            violations.append(Violation(
                attack_type="A5",
                iso_clause="5.5",
                detection_mechanism="DM-4",
                description=(
                    "safety_logic_requirements.required_nodes is empty — "
                    "all mandatory safety patterns have been removed"
                ),
                severity=0.8,
                location="safety_logic_requirements.required_nodes",
                metadata={"required_nodes_count": 0},
            ))

        # Check 3: dead_code_check / cfg_reachability disabled
        if slr.get("dead_code_check") is False:
            violations.append(Violation(
                attack_type="A5",
                iso_clause="5.5",
                detection_mechanism="DM-4",
                description="dead_code_check disabled — unreachable safety code won't be caught",
                severity=0.3,
                location="safety_logic_requirements.dead_code_check",
            ))

        # cfg_reachability_check=false is only flagged when required_nodes
        # have also been emptied (indicates A5 mutation stripping everything)
        if (slr.get("cfg_reachability_check") is False
                and slr.get("dead_code_check") is False
                and len(mandatory_nodes) == 0
                and estop_required):
            violations.append(Violation(
                attack_type="A5",
                iso_clause="5.5",
                detection_mechanism="DM-4",
                description=(
                    "All safety verification disabled: dead_code_check=false, "
                    "cfg_reachability_check=false, required_nodes empty"
                ),
                severity=0.5,
                location="safety_logic_requirements",
            ))

    return violations
