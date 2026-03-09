"""Tests for PR-H: IR → URScript Translator."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import json
import math
from pathlib import Path

import pytest

from enfield_translators.urscript_translator import IRToURScriptTranslator
from enfield_translators.urscript_utils import (
    joints_to_urscript,
    mm_s2_to_m_s2,
    mm_s_to_m_s,
    mm_to_m,
    pose_to_urscript,
    quaternion_to_axis_angle,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

TASKS_DIR = Path(__file__).resolve().parent.parent.parent / "enfield_tasks" / "ir" / "tasks"
BASELINE_FILES = sorted(TASKS_DIR.glob("T[0-9][0-9][0-9]_*.json"))


@pytest.fixture
def translator() -> IRToURScriptTranslator:
    return IRToURScriptTranslator()


@pytest.fixture
def t001() -> dict:
    with open(TASKS_DIR / "T001_pick_place_collab.json") as f:
        return json.load(f)


@pytest.fixture
def t002() -> dict:
    with open(TASKS_DIR / "T002_linear_weld_fenced.json") as f:
        return json.load(f)


@pytest.fixture(params=["T001", "T002", "T003", "T004", "T005"])
def any_baseline(request) -> dict:
    bf = [f for f in BASELINE_FILES if f.name.startswith(request.param)][0]
    with open(bf) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Unit conversion tests
# ---------------------------------------------------------------------------


class TestUnitConversions:

    def test_mm_to_m(self):
        assert mm_to_m(1000.0) == 1.0
        assert mm_to_m(250.0) == 0.25
        assert mm_to_m(0.0) == 0.0

    def test_mm_s_to_m_s(self):
        assert mm_s_to_m_s(250.0) == 0.25
        assert mm_s_to_m_s(500.0) == 0.5

    def test_mm_s2_to_m_s2(self):
        assert mm_s2_to_m_s2(1000.0) == 1.0
        assert mm_s2_to_m_s2(500.0) == 0.5


# ---------------------------------------------------------------------------
# Quaternion → axis-angle tests
# ---------------------------------------------------------------------------


class TestQuaternionConversion:

    def test_identity_quaternion(self):
        """Identity quaternion (0,0,0,1) → zero rotation."""
        rx, ry, rz = quaternion_to_axis_angle(0.0, 0.0, 0.0, 1.0)
        assert abs(rx) < 1e-6
        assert abs(ry) < 1e-6
        assert abs(rz) < 1e-6

    def test_180_about_y(self):
        """qx=0, qy=1, qz=0, qw=0 → 180° about Y → (0, π, 0)."""
        rx, ry, rz = quaternion_to_axis_angle(0.0, 1.0, 0.0, 0.0)
        assert abs(rx) < 1e-4
        assert abs(ry - math.pi) < 1e-4
        assert abs(rz) < 1e-4

    def test_180_about_z(self):
        """qx=0, qy=0, qz=1, qw=0 → 180° about Z → (0, 0, π)."""
        rx, ry, rz = quaternion_to_axis_angle(0.0, 0.0, 1.0, 0.0)
        assert abs(rx) < 1e-4
        assert abs(ry) < 1e-4
        assert abs(rz - math.pi) < 1e-4

    def test_90_about_z(self):
        """90° about Z: qx=0, qy=0, qz=sin(45°), qw=cos(45°)."""
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        rx, ry, rz = quaternion_to_axis_angle(0.0, 0.0, s, c)
        assert abs(rx) < 1e-4
        assert abs(ry) < 1e-4
        assert abs(rz - math.pi / 2) < 1e-4

    def test_near_zero_quaternion_safe(self):
        """Near-zero quaternion should not crash."""
        rx, ry, rz = quaternion_to_axis_angle(0.0, 0.0, 0.0, 0.0)
        assert abs(rx) < 1e-6
        assert abs(ry) < 1e-6
        assert abs(rz) < 1e-6

    def test_unnormalized_quaternion(self):
        """Unnormalized quaternion should still produce correct result."""
        rx, ry, rz = quaternion_to_axis_angle(0.0, 2.0, 0.0, 0.0)
        # Should normalize to (0,1,0,0) → 180° about Y
        assert abs(ry - math.pi) < 1e-4


# ---------------------------------------------------------------------------
# Pose / joints formatting tests
# ---------------------------------------------------------------------------


class TestFormatting:

    def test_pose_to_urscript_basic(self):
        s = pose_to_urscript(
            {"x": 400.0, "y": 0.0, "z": 300.0},
            {"qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0},
        )
        assert s.startswith("p[")
        assert "0.400000" in s  # x = 400mm → 0.4m
        assert "0.300000" in s  # z = 300mm → 0.3m

    def test_pose_no_orientation(self):
        s = pose_to_urscript({"x": 100.0, "y": 200.0, "z": 300.0})
        assert "0.0000, 0.0000, 0.0000]" in s  # zero rotation

    def test_joints_to_urscript(self):
        s = joints_to_urscript([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        assert s.startswith("[")
        assert "-1.5708" in s
        assert s.count(",") == 5


# ---------------------------------------------------------------------------
# Command translation tests
# ---------------------------------------------------------------------------


class TestCommandTranslation:

    def test_move_linear_generates_movel(self, translator, t001):
        script = translator.translate(t001)
        assert "movel(" in script
        assert "p[" in script

    def test_move_joint_generates_movej(self, translator, t001):
        script = translator.translate(t001)
        assert "movej(" in script

    def test_set_tool_gripper_generates_digital_out(self, translator, t001):
        script = translator.translate(t001)
        assert "set_digital_out(0, True)" in script  # gripper close
        assert "set_digital_out(0, False)" in script  # gripper open

    def test_wait_generates_sleep(self, translator, t001):
        script = translator.translate(t001)
        assert "sleep(" in script

    def test_estop_check_generates_guard(self, translator, t001):
        script = translator.translate(t001)
        assert "E-Stop" in script
        assert "halt" in script

    def test_comment_preserved(self, translator, t001):
        # T001 doesn't have comment commands, but T003 does
        with open(TASKS_DIR / "T003_palletize_hybrid.json") as f:
            t003 = json.load(f)
        script = translator.translate(t003)
        assert "Pallet grid" in script

    def test_set_digital_output(self, translator, t002):
        script = translator.translate(t002)
        # T002 has gas flow digital outputs
        assert "set_digital_out(1, True)" in script
        assert "set_digital_out(1, False)" in script

    def test_welder_tool_commands(self, translator, t002):
        script = translator.translate(t002)
        # T002 has arc ON/OFF via set_tool
        assert "arc ON" in script.lower() or "Welding arc ON" in script


# ---------------------------------------------------------------------------
# Full translation structure tests
# ---------------------------------------------------------------------------


class TestTranslationStructure:

    def test_header_present(self, translator, t001):
        script = translator.translate(t001)
        assert "# Task: T001" in script
        assert "ENFIELD IR→URScript Translator" in script
        assert "SIMULATION ONLY" in script

    def test_function_wrapping(self, translator, t001):
        script = translator.translate(t001)
        assert "def task_T001():" in script
        assert script.strip().endswith("task_T001()")

    def test_operating_mode_in_header(self, translator, t001):
        script = translator.translate(t001)
        assert "collaborative" in script

    def test_speed_converted_to_m_s(self, translator, t001):
        script = translator.translate(t001)
        # T001 max speed = 250mm/s → 0.250 m/s in header
        assert "0.250" in script

    def test_position_converted_to_meters(self, translator, t001):
        script = translator.translate(t001)
        # T001 pick position x=400mm → 0.400m
        assert "0.400000" in script

    def test_blend_radius_when_nonzero(self, translator, t001):
        script = translator.translate(t001)
        # T001 has blend_radius_mm=10 → r=0.0100
        assert "r=0.0100" in script

    def test_no_blend_radius_when_zero(self, translator, t001):
        # For fine positioning (blend=0), 'r=' should not appear on that line
        script = translator.translate(t001)
        lines = script.split("\n")
        fine_lines = [l for l in lines if "movel(" in l and "Descend to pick" not in l]
        # At least some movel lines should have r= (the ones with blend>0)
        r_lines = [l for l in lines if "movel(" in l and "r=" in l]
        no_r_lines = [l for l in lines if "movel(" in l and "r=" not in l]
        assert len(r_lines) > 0, "Should have some moves with blend radius"
        assert len(no_r_lines) > 0, "Should have some fine-positioning moves without r="


# ---------------------------------------------------------------------------
# All-tasks translation test
# ---------------------------------------------------------------------------


class TestAllTasks:

    def test_translates_without_error(self, translator, any_baseline):
        script = translator.translate(any_baseline)
        assert len(script) > 100
        task_id = any_baseline["task"]["id"]
        assert f"def task_{task_id}():" in script

    def test_has_movel_or_movej(self, translator, any_baseline):
        script = translator.translate(any_baseline)
        assert "movel(" in script or "movej(" in script

    def test_ends_with_entry_point(self, translator, any_baseline):
        script = translator.translate(any_baseline)
        task_id = any_baseline["task"]["id"]
        assert script.strip().endswith(f"task_{task_id}()")


# ---------------------------------------------------------------------------
# File I/O tests
# ---------------------------------------------------------------------------


class TestFileIO:

    def test_translate_file(self, translator):
        script = translator.translate_file(TASKS_DIR / "T001_pick_place_collab.json")
        assert "def task_T001():" in script

    def test_translate_to_file(self, translator, tmp_path):
        out = tmp_path / "T001.script"
        result = translator.translate_to_file(
            TASKS_DIR / "T001_pick_place_collab.json", out
        )
        assert result.exists()
        content = result.read_text()
        assert "movel(" in content

    def test_batch_translate(self, translator, tmp_path):
        out_dir = tmp_path / "urscript"
        manifest = translator.batch_translate(
            tasks_dir=str(TASKS_DIR),
            output_dir=str(out_dir),
        )
        assert len(manifest) == 5
        for entry in manifest:
            assert entry["output"].endswith(".script")
            assert (out_dir / entry["output"]).exists()

    def test_batch_manifest_written(self, translator, tmp_path):
        out_dir = tmp_path / "urscript"
        translator.batch_translate(str(TASKS_DIR), str(out_dir))
        manifest_file = out_dir / "manifest.json"
        assert manifest_file.exists()
        with open(manifest_file) as f:
            data = json.load(f)
        assert len(data) == 5


# ---------------------------------------------------------------------------
# Adversarial variant translation test
# ---------------------------------------------------------------------------


class TestAdversarialTranslation:
    """Verify that adversarial variants (from PR-G) also translate cleanly."""

    VARIANTS_DIR = Path(__file__).resolve().parent.parent.parent / "enfield_attacks" / "generated" / "variants"

    @pytest.fixture
    def variant_files(self):
        if not self.VARIANTS_DIR.exists():
            pytest.skip("Adversarial variants not found")
        return sorted(self.VARIANTS_DIR.glob("T*_A*_*.json"))

    def test_adversarial_variants_translate(self, translator, variant_files):
        """All 40 adversarial variants should translate without error."""
        errors = []
        for vf in variant_files:
            try:
                with open(vf) as f:
                    task = json.load(f)
                script = translator.translate(task)
                assert len(script) > 50, f"{vf.name}: script too short"
            except Exception as exc:
                errors.append(f"{vf.name}: {exc}")
        assert len(errors) == 0, f"Translation errors:\n" + "\n".join(errors)

    def test_adversarial_variant_count(self, variant_files):
        if len(variant_files) == 0:
            pytest.skip("Adversarial variants not found")
        assert len(variant_files) == 40


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


class TestEdgeCases:

    def test_empty_motion_sequence(self, translator):
        task = {
            "task": {"id": "T999", "name": "empty", "operating_mode": "collaborative"},
            "robot": {"adapter": "enfield_robots_ur5e", "model": "ur5e"},
            "motion_sequence": [{"seq": 0, "type": "comment", "comment_text": "nothing"}],
        }
        script = translator.translate(task)
        assert "def task_T999():" in script

    def test_unknown_command_type(self, translator):
        task = {
            "task": {"id": "T998", "name": "bad", "operating_mode": "fenced"},
            "robot": {"adapter": "enfield_robots_ur5e", "model": "ur5e"},
            "motion_sequence": [{"seq": 0, "type": "teleport", "label": "magic"}],
        }
        script = translator.translate(task)
        assert "WARNING: Unknown command type" in script

    def test_missing_orientation_defaults_to_zero(self, translator):
        task = {
            "task": {"id": "T997", "name": "no_orient", "operating_mode": "collaborative"},
            "robot": {"adapter": "enfield_robots_ur5e", "model": "ur5e"},
            "motion_sequence": [{
                "seq": 0, "type": "move_linear",
                "target_pose": {"position": {"x": 100.0, "y": 200.0, "z": 300.0}},
                "speed_mm_s": 100.0,
            }],
        }
        script = translator.translate(task)
        assert "movel(p[0.100000, 0.200000, 0.300000, 0.0000, 0.0000, 0.0000]" in script
