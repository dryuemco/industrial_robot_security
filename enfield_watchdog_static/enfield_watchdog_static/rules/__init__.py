# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A1–A8 safety rules + SM-1–SM-7 security rules for static analysis."""

# --- Safety rules (Task IR level) ---
from enfield_watchdog_static.rules.a1_speed import check_a1_speed
from enfield_watchdog_static.rules.a2_zone import check_a2_zone
from enfield_watchdog_static.rules.a3_orientation import check_a3_orientation
from enfield_watchdog_static.rules.a4_payload import check_a4_payload
from enfield_watchdog_static.rules.a5_logic import check_a5_logic
from enfield_watchdog_static.rules.a6_frame import check_a6_frame
from enfield_watchdog_static.rules.a7_tool import check_a7_tool
from enfield_watchdog_static.rules.a8_prompt import check_a8_prompt

# --- Security rules (URScript code level) ---
from enfield_watchdog_static.rules.sm1_input_validation import check_sm1_input_validation
from enfield_watchdog_static.rules.sm2_unchecked_return import check_sm2_unchecked_return
from enfield_watchdog_static.rules.sm3_protection_failure import check_sm3_protection_failure
from enfield_watchdog_static.rules.sm4_unusual_conditions import check_sm4_unusual_conditions
from enfield_watchdog_static.rules.sm5_hardcoded_values import check_sm5_hardcoded_values
from enfield_watchdog_static.rules.sm6_missing_preamble import check_sm6_missing_preamble
from enfield_watchdog_static.rules.sm7_prompt_marker import check_sm7_prompt_marker

ALL_RULES = {
    "A1": check_a1_speed,
    "A2": check_a2_zone,
    "A3": check_a3_orientation,
    "A4": check_a4_payload,
    "A5": check_a5_logic,
    "A6": check_a6_frame,
    "A7": check_a7_tool,
    "A8": check_a8_prompt,
}

ALL_SECURITY_RULES = {
    "SM-1": check_sm1_input_validation,
    "SM-2": check_sm2_unchecked_return,
    "SM-3": check_sm3_protection_failure,
    "SM-4": check_sm4_unusual_conditions,
    "SM-5": check_sm5_hardcoded_values,
    "SM-6": check_sm6_missing_preamble,
    "SM-7": check_sm7_prompt_marker,
}

__all__ = [
    "ALL_RULES",
    "ALL_SECURITY_RULES",
    "check_a1_speed", "check_a2_zone", "check_a3_orientation",
    "check_a4_payload", "check_a5_logic", "check_a6_frame",
    "check_a7_tool", "check_a8_prompt",
    "check_sm1_input_validation", "check_sm2_unchecked_return",
    "check_sm3_protection_failure", "check_sm4_unusual_conditions",
    "check_sm5_hardcoded_values", "check_sm6_missing_preamble",
    "check_sm7_prompt_marker",
]
