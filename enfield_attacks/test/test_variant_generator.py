"""Tests for PR-G attack variant generator — A1–A8 mutations."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import copy
import json
from pathlib import Path

import pytest

from enfield_attacks.mutations import MUTATIONS, MutationResult
from enfield_attacks.mutations.a1_speed import A1SpeedInjection
from enfield_attacks.mutations.a2_zone import A2ZonePenetration
from enfield_attacks.mutations.a3_orientation import A3OrientationAnomaly
from enfield_attacks.mutations.a4_payload import A4PayloadMisconfiguration
from enfield_attacks.mutations.a5_logic import A5LogicBypass
from enfield_attacks.mutations.a6_frame import A6FrameConfusion
from enfield_attacks.mutations.a7_tool import A7ToolMisuse
from enfield_attacks.mutations.a8_prompt import A8PromptInjection
from enfield_attacks.variant_generator import (
    batch_generate,
    generate_all_variants,
    generate_variant,
    generate_variant_id,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

TASKS_DIR = Path(__file__).resolve().parent.parent.parent / "enfield_tasks" / "ir" / "tasks"
BASELINE_FILES = sorted(TASKS_DIR.glob("T[0-9][0-9][0-9]_*.json"))
ALL_TASK_IDS = sorted({p.name.split("_")[0] for p in BASELINE_FILES})


@pytest.fixture
def t001() -> dict:
    with open(TASKS_DIR / "T001_pick_place_collab.json") as f:
        return json.load(f)


@pytest.fixture
def t002() -> dict:
    with open(TASKS_DIR / "T002_linear_weld_fenced.json") as f:
        return json.load(f)


@pytest.fixture
def t003() -> dict:
    with open(TASKS_DIR / "T003_palletize_hybrid.json") as f:
        return json.load(f)


@pytest.fixture
def t004() -> dict:
    with open(TASKS_DIR / "T004_multi_wp_obstacle_collab.json") as f:
        return json.load(f)


@pytest.fixture
def t005() -> dict:
    with open(TASKS_DIR / "T005_inspection_scan_collab.json") as f:
        return json.load(f)


@pytest.fixture(params=ALL_TASK_IDS)
def any_baseline(request) -> dict:
    bf = [f for f in BASELINE_FILES if f.name.startswith(request.param)][0]
    with open(bf) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Registry tests
# ---------------------------------------------------------------------------


class TestMutationRegistry:

    def test_all_8_attacks_registered(self):
        assert set(MUTATIONS.keys()) == {"A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"}

    def test_variant_id_format(self):
        assert generate_variant_id("T001", "A1") == "T001_A1"
        assert generate_variant_id("T005", "A8") == "T005_A8"


# ---------------------------------------------------------------------------
# A1: Speed Injection
# ---------------------------------------------------------------------------


class TestA1SpeedInjection:

    def test_speeds_exceed_limit(self, t001):
        m = A1SpeedInjection(multiplier=2.5)
        variant, result = m.safe_apply(t001)
        limit = t001["safety_requirements"]["max_tcp_speed_mm_s"]
        for cmd in variant["motion_sequence"]:
            if "speed_mm_s" in cmd:
                assert cmd["speed_mm_s"] > limit

    def test_result_metadata(self, t001):
        m = A1SpeedInjection()
        _, result = m.safe_apply(t001)
        assert result.attack_type == "A1"
        assert result.severity_estimate > 0
        assert len(result.mutated_fields) > 0

    def test_baseline_unchanged(self, t001):
        original = copy.deepcopy(t001)
        m = A1SpeedInjection()
        m.safe_apply(t001)
        assert t001 == original

    def test_all_tasks(self, any_baseline):
        m = A1SpeedInjection()
        variant, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A1"
        assert result.severity_estimate > 0


# ---------------------------------------------------------------------------
# A2: Zone Penetration
# ---------------------------------------------------------------------------


class TestA2ZonePenetration:

    def test_waypoint_outside_space(self, t001):
        m = A2ZonePenetration(margin_mm=200.0)
        variant, result = m.safe_apply(t001)
        assert result.attack_type == "A2"
        assert len(result.mutated_fields) > 0
        assert result.severity_estimate == 200.0

    def test_all_tasks(self, any_baseline):
        m = A2ZonePenetration()
        variant, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A2"


# ---------------------------------------------------------------------------
# A3: Orientation Anomaly
# ---------------------------------------------------------------------------


class TestA3OrientationAnomaly:

    def test_orientation_flipped(self, t001):
        m = A3OrientationAnomaly()
        variant, result = m.safe_apply(t001)
        assert result.attack_type == "A3"
        # T001 has upward cone → identity quat should be injected
        for cmd in variant["motion_sequence"]:
            tp = cmd.get("target_pose", {})
            if "orientation" in tp:
                # Should be identity or a known violating quaternion
                assert "qw" in tp["orientation"]

    def test_all_tasks(self, any_baseline):
        m = A3OrientationAnomaly()
        _, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A3"


# ---------------------------------------------------------------------------
# A4: Payload Misconfiguration
# ---------------------------------------------------------------------------


class TestA4PayloadMisconfiguration:

    def test_mass_under_declared(self, t001):
        m = A4PayloadMisconfiguration(undershoot_factor=0.1)
        variant, result = m.safe_apply(t001)
        mass_min = t001["safety_requirements"]["tool_payload"]["mass_min_kg"]
        assert variant["robot"]["payload_kg"] < mass_min
        assert result.attack_type == "A4"

    def test_all_tasks(self, any_baseline):
        m = A4PayloadMisconfiguration()
        variant, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A4"
        assert variant["robot"]["payload_kg"] < any_baseline["robot"]["payload_kg"]


# ---------------------------------------------------------------------------
# A5: Logic Bypass
# ---------------------------------------------------------------------------


class TestA5LogicBypass:

    def test_estop_removed(self, t001):
        m = A5LogicBypass()
        variant, result = m.safe_apply(t001)
        types = [c["type"] for c in variant["motion_sequence"]]
        assert "estop_check" not in types
        assert result.attack_type == "A5"

    def test_safety_nodes_cleared(self, t001):
        m = A5LogicBypass()
        variant, _ = m.safe_apply(t001)
        slr = variant.get("safety_logic_requirements", {})
        assert slr.get("required_nodes") == []

    def test_seq_reindexed(self, t001):
        m = A5LogicBypass()
        variant, _ = m.safe_apply(t001)
        seqs = [c["seq"] for c in variant["motion_sequence"]]
        assert seqs == list(range(len(seqs)))

    def test_all_tasks(self, any_baseline):
        m = A5LogicBypass()
        variant, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A5"


# ---------------------------------------------------------------------------
# A6: Frame Confusion
# ---------------------------------------------------------------------------


class TestA6FrameConfusion:

    def test_wobj_shifted(self, t001):
        m = A6FrameConfusion(shift_mm=500.0)
        variant, result = m.safe_apply(t001)
        wos = variant.get("work_object_specification", {})
        orig_wos = t001.get("work_object_specification", {})
        if "uframe_translation_mm" in wos:
            assert wos["uframe_translation_mm"][0] != orig_wos["uframe_translation_mm"][0]
        assert result.attack_type == "A6"

    def test_all_tasks(self, any_baseline):
        m = A6FrameConfusion()
        _, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A6"


# ---------------------------------------------------------------------------
# A7: Tool Misuse
# ---------------------------------------------------------------------------


class TestA7ToolMisuse:

    def test_tool_type_changed(self, t001):
        m = A7ToolMisuse()
        variant, result = m.safe_apply(t001)
        assert variant["tool"]["type"] != t001["tool"]["type"]
        assert result.attack_type == "A7"

    def test_modes_exclude_current(self, t001):
        m = A7ToolMisuse()
        variant, _ = m.safe_apply(t001)
        ac = variant.get("tool", {}).get("activation_constraints", {})
        if "allowed_modes" in ac:
            op_mode = t001["task"]["operating_mode"]
            assert op_mode not in ac["allowed_modes"]

    def test_all_tasks(self, any_baseline):
        m = A7ToolMisuse()
        variant, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A7"
        assert variant["tool"]["type"] != any_baseline["tool"]["type"]


# ---------------------------------------------------------------------------
# A8: Prompt Injection
# ---------------------------------------------------------------------------


class TestA8PromptInjection:

    def test_prompt_security_weakened(self, t001):
        m = A8PromptInjection()
        variant, result = m.safe_apply(t001)
        ps = variant.get("prompt_security", {})
        assert ps.get("perplexity_threshold", 0) > 9999
        assert ps.get("blocked_patterns") == []
        assert ps.get("encoding_scan") is False

    def test_combined_a1_a5(self, t001):
        m = A8PromptInjection(speed_multiplier=3.0)
        variant, result = m.safe_apply(t001)
        # A1: speeds exceeded
        limit = t001["safety_requirements"]["max_tcp_speed_mm_s"]
        for cmd in variant["motion_sequence"]:
            if "speed_mm_s" in cmd:
                assert cmd["speed_mm_s"] > limit
        # A5: no estop
        types = [c["type"] for c in variant["motion_sequence"]]
        assert "estop_check" not in types
        assert result.attack_type == "A8"
        assert "A1" in result.metadata["induced_attacks"]
        assert "A5" in result.metadata["induced_attacks"]

    def test_all_tasks(self, any_baseline):
        m = A8PromptInjection()
        _, result = m.safe_apply(any_baseline)
        assert result.attack_type == "A8"


# ---------------------------------------------------------------------------
# Integration: generate_variant() API
# ---------------------------------------------------------------------------


class TestGenerateVariant:

    def test_variant_id_stamped(self, t001):
        variant, _ = generate_variant(t001, "A1")
        assert variant["task"]["id"] == "T001_A1"

    def test_variant_name_prefixed(self, t001):
        variant, _ = generate_variant(t001, "A3")
        assert "[A3]" in variant["task"]["name"]

    def test_adversarial_tag_added(self, t001):
        variant, _ = generate_variant(t001, "A5")
        assert "adversarial" in variant["task"]["tags"]
        assert "a5" in variant["task"]["tags"]

    def test_all_attack_types(self, t001):
        for aid in MUTATIONS:
            variant, result = generate_variant(t001, aid)
            assert result.attack_type == aid
            assert variant["task"]["id"] == f"T001_{aid}"

    def test_invalid_attack_id(self, t001):
        with pytest.raises(KeyError):
            generate_variant(t001, "A9")


# ---------------------------------------------------------------------------
# Integration: generate_all_variants()
# ---------------------------------------------------------------------------


class TestGenerateAllVariants:

    def test_produces_8_variants(self, t001):
        results = generate_all_variants(t001)
        assert len(results) == 8

    def test_subset_generation(self, t001):
        results = generate_all_variants(t001, attack_ids=["A1", "A5"])
        assert len(results) == 2
        types = {r.attack_type for _, r in results}
        assert types == {"A1", "A5"}


# ---------------------------------------------------------------------------
# Integration: batch_generate()
# ---------------------------------------------------------------------------


class TestBatchGenerate:

    def test_produces_40_variants(self, tmp_path):
        output = tmp_path / "variants"
        manifest = batch_generate(
            tasks_dir=str(TASKS_DIR),
            output_dir=str(output),
            seed=42,
        )
        assert len(manifest) == len(list(Path(str(TASKS_DIR)).glob("T[0-9][0-9][0-9]_*.json"))) * 8
        # Check files exist
        json_files = list(output.glob("T*_A*_*.json"))
        assert len(json_files) == len(manifest)

    def test_manifest_written(self, tmp_path):
        output = tmp_path / "variants"
        batch_generate(tasks_dir=str(TASKS_DIR), output_dir=str(output))
        manifest_file = output / "manifest.json"
        assert manifest_file.exists()
        with open(manifest_file) as f:
            data = json.load(f)
        assert len(data) == len(list(Path(str(TASKS_DIR)).glob("T[0-9][0-9][0-9]_*.json"))) * 8

    def test_each_variant_is_valid_json(self, tmp_path):
        output = tmp_path / "variants"
        batch_generate(tasks_dir=str(TASKS_DIR), output_dir=str(output))
        for jf in output.glob("T*_A*_*.json"):
            with open(jf) as f:
                d = json.load(f)
            assert "task" in d
            assert "motion_sequence" in d

    def test_coverage_matrix(self, tmp_path):
        output = tmp_path / "variants"
        manifest = batch_generate(tasks_dir=str(TASKS_DIR), output_dir=str(output))
        # Every task × every attack
        pairs = {(e["baseline_id"], e["attack_type"]) for e in manifest}
        for tid in ALL_TASK_IDS:
            for aid in ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"]:
                assert (tid, aid) in pairs, f"Missing {tid}×{aid}"


# ---------------------------------------------------------------------------
# Determinism test
# ---------------------------------------------------------------------------


class TestDeterminism:

    def test_same_seed_same_output(self, t001):
        v1, r1 = generate_variant(t001, "A1", seed=42)
        v2, r2 = generate_variant(t001, "A1", seed=42)
        assert v1 == v2
        assert r1.description == r2.description

    def test_different_seed_may_differ(self, t001):
        # With current implementations seeds don't affect output
        # (mutations are deterministic by design), but the API supports it
        v1, _ = generate_variant(t001, "A1", seed=42)
        v2, _ = generate_variant(t001, "A1", seed=99)
        # Both should still produce valid mutations
        assert v1["task"]["id"] == "T001_A1"
        assert v2["task"]["id"] == "T001_A1"
