# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A1–A8 static detection rules for Task IR analysis."""

from enfield_watchdog_static.rules.a1_speed import check_a1_speed
from enfield_watchdog_static.rules.a2_zone import check_a2_zone
from enfield_watchdog_static.rules.a3_orientation import check_a3_orientation
from enfield_watchdog_static.rules.a4_payload import check_a4_payload
from enfield_watchdog_static.rules.a5_logic import check_a5_logic
from enfield_watchdog_static.rules.a6_frame import check_a6_frame
from enfield_watchdog_static.rules.a7_tool import check_a7_tool
from enfield_watchdog_static.rules.a8_prompt import check_a8_prompt

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

__all__ = [
    "ALL_RULES",
    "check_a1_speed", "check_a2_zone", "check_a3_orientation",
    "check_a4_payload", "check_a5_logic", "check_a6_frame",
    "check_a7_tool", "check_a8_prompt",
]
