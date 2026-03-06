# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A1-A8 attack mutation functions for Task IR variant generation."""

from enfield_attacks.mutations.base import Mutation, MutationResult
from enfield_attacks.mutations.a1_speed import A1SpeedInjection
from enfield_attacks.mutations.a2_zone import A2ZonePenetration
from enfield_attacks.mutations.a3_orientation import A3OrientationAnomaly
from enfield_attacks.mutations.a4_payload import A4PayloadMisconfiguration
from enfield_attacks.mutations.a5_logic import A5LogicBypass
from enfield_attacks.mutations.a6_frame import A6FrameConfusion
from enfield_attacks.mutations.a7_tool import A7ToolMisuse
from enfield_attacks.mutations.a8_prompt import A8PromptInjection

MUTATIONS: dict[str, type[Mutation]] = {
    "A1": A1SpeedInjection,
    "A2": A2ZonePenetration,
    "A3": A3OrientationAnomaly,
    "A4": A4PayloadMisconfiguration,
    "A5": A5LogicBypass,
    "A6": A6FrameConfusion,
    "A7": A7ToolMisuse,
    "A8": A8PromptInjection,
}

__all__ = [
    "Mutation", "MutationResult", "MUTATIONS",
    "A1SpeedInjection", "A2ZonePenetration", "A3OrientationAnomaly",
    "A4PayloadMisconfiguration", "A5LogicBypass", "A6FrameConfusion",
    "A7ToolMisuse", "A8PromptInjection",
]
