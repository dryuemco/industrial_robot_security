"""Tests for enfield_llm.prompt_builder."""

import pytest

from enfield_llm.prompt_builder import (
    AdversarialType,
    PromptBuilder,
    PromptMode,
    SYSTEM_PROMPT_BASELINE,
    SYSTEM_PROMPT_SAFETY,
)


# Minimal Task IR fixture
SAMPLE_TASK_IR = {
    "task_id": "T001",
    "task_name": "Single object pick-place",
    "category": "pick_place",
    "description": "Pick object at position A and place at position B.",
    "robot": "UR5e",
    "operation_mode": "collaborative",
    "waypoints": [
        {"pose": [0.4, 0.2, 0.15, 3.14, 0, 0], "move_type": "movej"},
        {"pose": [0.4, 0.2, 0.05, 3.14, 0, 0], "move_type": "movel"},
        {"pose": [0.6, -0.1, 0.15, 3.14, 0, 0], "move_type": "movej"},
    ],
    "payload": {"mass_kg": 0.5},
    "tool": {"type": "gripper"},
    "safety_constraints": {
        "max_tcp_speed_mm_s": 250,
        "max_joint_accel_rad_s2": 1.4,
    },
    "safety_zones": [
        {"zone_id": "SZ001", "type": "collaborative"},
    ],
}


@pytest.fixture
def builder():
    return PromptBuilder()


class TestPromptModes:
    """Test that each mode produces valid prompt tuples."""

    def test_baseline_returns_tuple(self, builder):
        system, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert isinstance(system, str)
        assert isinstance(user, str)
        assert len(system) > 0
        assert len(user) > 0

    def test_baseline_uses_baseline_system_prompt(self, builder):
        system, _ = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert system == SYSTEM_PROMPT_BASELINE

    def test_safety_uses_safety_system_prompt(self, builder):
        system, _ = builder.build(SAMPLE_TASK_IR, PromptMode.SAFETY)
        assert system == SYSTEM_PROMPT_SAFETY
        assert "ISO 10218" in system
        assert "250 mm/s" in system

    def test_adversarial_requires_type(self, builder):
        with pytest.raises(ValueError, match="adversarial_type required"):
            builder.build(SAMPLE_TASK_IR, PromptMode.ADVERSARIAL)

    def test_adversarial_accepts_valid_type(self, builder):
        system, user = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_6_SPEC_AMBIGUITY,
        )
        assert isinstance(system, str)
        assert isinstance(user, str)


class TestUserPromptContent:
    """Test that user prompts contain expected task information."""

    def test_contains_task_id(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "T001" in user

    def test_contains_task_name(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "Single object pick-place" in user

    def test_contains_robot(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "UR5e" in user

    def test_contains_waypoints(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "movej" in user
        assert "movel" in user

    def test_contains_payload(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "0.5 kg" in user

    def test_contains_safety_zone(self, builder):
        _, user = builder.build(SAMPLE_TASK_IR, PromptMode.BASELINE)
        assert "SZ001" in user


class TestAdversarialPrompts:
    """Test that adversarial modes inject expected content."""

    def test_a6_1_mentions_speed(self, builder):
        _, user = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_1_SAFETY_OVERRIDE,
        )
        assert "maximum speed" in user.lower()

    def test_a6_2_modifies_system_prompt(self, builder):
        system, _ = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_2_CONTEXT_MANIPULATION,
        )
        assert "fully enclosed safety cell" in system

    def test_a6_4_injects_in_task(self, builder):
        _, user = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_4_INSTRUCTION_INJECTION,
        )
        assert "SYSTEM NOTE" in user

    def test_a6_7_claims_authority(self, builder):
        _, user = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_7_AUTHORITY_IMPERSONATION,
        )
        assert "safety officer" in user.lower()

    def test_a6_8_uses_hex(self, builder):
        _, user = builder.build(
            SAMPLE_TASK_IR,
            PromptMode.ADVERSARIAL,
            AdversarialType.A6_8_OBFUSCATION,
        )
        assert "0x03E8" in user

    def test_all_types_produce_output(self, builder):
        """Every A6.x type should produce non-empty prompts."""
        for attack_type in AdversarialType:
            system, user = builder.build(
                SAMPLE_TASK_IR,
                PromptMode.ADVERSARIAL,
                attack_type,
            )
            assert len(system) > 0, f"{attack_type} produced empty system"
            assert len(user) > 0, f"{attack_type} produced empty user"

    def test_get_all_adversarial_types(self, builder):
        types = builder.get_all_adversarial_types()
        assert len(types) == 8
