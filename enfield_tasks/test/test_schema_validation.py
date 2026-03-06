"""Tests for Task IR schema validation — PR-E."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import copy
import json
from pathlib import Path

import pytest

from enfield_tasks.validators.schema_validator import (
    TaskIRValidationResult,
    load_schema,
    validate_file,
    validate_task_ir,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

TASKS_DIR = Path(__file__).resolve().parent.parent / "ir" / "tasks"
SCHEMA_DIR = Path(__file__).resolve().parent.parent / "ir" / "schema"
T001_PATH = TASKS_DIR / "T001_pick_place_collab.json"
T002_PATH = TASKS_DIR / "T002_linear_weld_fenced.json"
T003_PATH = TASKS_DIR / "T003_palletize_hybrid.json"
T004_PATH = TASKS_DIR / "T004_multi_wp_obstacle_collab.json"
T005_PATH = TASKS_DIR / "T005_inspection_scan_collab.json"

ALL_TASK_PATHS = [T001_PATH, T002_PATH, T003_PATH, T004_PATH, T005_PATH]


@pytest.fixture
def t001_data() -> dict:
    """Load T001 task as dict."""
    with open(T001_PATH, "r") as f:
        return json.load(f)


@pytest.fixture
def t002_data() -> dict:
    with open(T002_PATH, "r") as f:
        return json.load(f)


@pytest.fixture
def t003_data() -> dict:
    with open(T003_PATH, "r") as f:
        return json.load(f)


@pytest.fixture
def t004_data() -> dict:
    with open(T004_PATH, "r") as f:
        return json.load(f)


@pytest.fixture
def t005_data() -> dict:
    with open(T005_PATH, "r") as f:
        return json.load(f)


@pytest.fixture
def schema() -> dict:
    """Load the schema."""
    return load_schema()


# ---------------------------------------------------------------------------
# Schema self-validation
# ---------------------------------------------------------------------------


class TestSchemaIntegrity:
    """Verify the schema file itself is well-formed."""

    def test_schema_file_exists(self):
        path = SCHEMA_DIR / "task_ir_v1.schema.json"
        assert path.exists(), f"Schema file not found: {path}"

    def test_schema_loads_without_error(self, schema):
        assert "$schema" in schema
        assert schema["title"] == "ENFIELD Task IR v1"

    def test_schema_version_const(self, schema):
        assert schema["properties"]["schema_version"]["const"] == "1.0.0"

    def test_required_top_level_fields(self, schema):
        required = schema["required"]
        for field in ["schema_version", "task", "robot", "environment", "motion_sequence"]:
            assert field in required

    def test_defs_present(self, schema):
        """Verify key $defs are present for A1-A8 coverage."""
        defs = schema.get("$defs", {})
        expected = [
            "halfspace",
            "orientation_cone",
            "tool_payload",
            "required_safety_node",
            "tool_activation_constraints",
            "safeguarded_space",
        ]
        for d in expected:
            assert d in defs, f"Missing $defs/{d}"


# ---------------------------------------------------------------------------
# T001 validation
# ---------------------------------------------------------------------------


class TestT001Validation:
    """Validate the T001 example task against the schema."""

    def test_t001_file_exists(self):
        assert T001_PATH.exists(), f"Example task not found: {T001_PATH}"

    def test_t001_schema_valid(self, t001_data):
        result = validate_task_ir(t001_data)
        assert result.valid, f"T001 should be valid:\n{result.summary()}"

    def test_t001_no_semantic_warnings(self, t001_data):
        result = validate_task_ir(t001_data)
        # T001 is designed as a clean baseline — no warnings expected
        assert len(result.semantic_warnings) == 0, (
            f"Unexpected warnings:\n{result.summary()}"
        )

    def test_t001_file_validation(self):
        result = validate_file(T001_PATH)
        assert result.valid

    def test_t001_task_metadata(self, t001_data):
        assert t001_data["task"]["id"] == "T001"
        assert t001_data["task"]["category"] == "pick_place"
        assert t001_data["task"]["operating_mode"] == "collaborative"

    def test_t001_has_safeguarded_space(self, t001_data):
        sg = t001_data["safety_requirements"]["safeguarded_space"]
        assert len(sg["halfspaces"]) == 6, "Box needs 6 halfspaces"

    def test_t001_has_orientation_cones(self, t001_data):
        cones = t001_data["safety_requirements"]["forbidden_orientation_cones"]
        assert len(cones) == 2

    def test_t001_has_tool_payload(self, t001_data):
        tp = t001_data["safety_requirements"]["tool_payload"]
        assert tp["mass_true_kg"] == 0.5
        assert tp["mass_min_kg"] < tp["mass_max_kg"]

    def test_t001_has_safety_logic_requirements(self, t001_data):
        slr = t001_data["safety_logic_requirements"]
        patterns = [n["pattern"] for n in slr["required_nodes"]]
        assert "estop_check" in patterns
        assert "error_handler" in patterns

    def test_t001_has_prompt_security(self, t001_data):
        ps = t001_data["prompt_security"]
        assert ps["perplexity_threshold"] > 0
        assert len(ps["blocked_patterns"]) > 0

    def test_t001_motion_sequence_has_estop(self, t001_data):
        types = [c["type"] for c in t001_data["motion_sequence"]]
        assert "estop_check" in types


# ---------------------------------------------------------------------------
# Negative tests — schema rejection
# ---------------------------------------------------------------------------


class TestSchemaRejection:
    """Verify the schema rejects invalid task data."""

    def test_missing_schema_version(self, t001_data):
        bad = copy.deepcopy(t001_data)
        del bad["schema_version"]
        result = validate_task_ir(bad)
        assert not result.valid

    def test_wrong_schema_version(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["schema_version"] = "2.0.0"
        result = validate_task_ir(bad)
        assert not result.valid

    def test_missing_task_id(self, t001_data):
        bad = copy.deepcopy(t001_data)
        del bad["task"]["id"]
        result = validate_task_ir(bad)
        assert not result.valid

    def test_invalid_task_id_format(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["task"]["id"] = "TASK1"  # Must be T###
        result = validate_task_ir(bad)
        assert not result.valid

    def test_invalid_operating_mode(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["task"]["operating_mode"] = "turbo"
        result = validate_task_ir(bad)
        assert not result.valid

    def test_missing_motion_sequence(self, t001_data):
        bad = copy.deepcopy(t001_data)
        del bad["motion_sequence"]
        result = validate_task_ir(bad)
        assert not result.valid

    def test_empty_motion_sequence(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["motion_sequence"] = []
        result = validate_task_ir(bad)
        assert not result.valid

    def test_invalid_command_type(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["motion_sequence"][0]["type"] = "teleport"
        result = validate_task_ir(bad)
        assert not result.valid

    def test_negative_speed(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["motion_sequence"][2]["speed_mm_s"] = -100.0
        result = validate_task_ir(bad)
        assert not result.valid

    def test_extra_top_level_field(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["hacker_field"] = "surprise"
        result = validate_task_ir(bad)
        assert not result.valid

    def test_halfspace_wrong_dimension(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["safety_requirements"]["safeguarded_space"]["halfspaces"][0]["normal"] = [1, 0]
        result = validate_task_ir(bad)
        assert not result.valid

    def test_orientation_cone_missing_axis(self, t001_data):
        bad = copy.deepcopy(t001_data)
        del bad["safety_requirements"]["forbidden_orientation_cones"][0]["axis"]
        result = validate_task_ir(bad)
        assert not result.valid


# ---------------------------------------------------------------------------
# Semantic warnings
# ---------------------------------------------------------------------------


class TestSemanticWarnings:
    """Verify semantic checks produce appropriate warnings."""

    def test_speed_exceeds_collab_limit(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["safety_requirements"]["max_tcp_speed_mm_s"] = 400.0  # > 250 for collab
        result = validate_task_ir(bad)
        assert result.valid  # schema-valid
        assert any("A1" in w for w in result.semantic_warnings)

    def test_waypoint_outside_safeguarded_space(self, t001_data):
        bad = copy.deepcopy(t001_data)
        # Move waypoint outside x_max=800
        bad["motion_sequence"][2]["target_pose"]["position"]["x"] = 900.0
        result = validate_task_ir(bad)
        assert result.valid
        assert any("A2" in w for w in result.semantic_warnings)

    def test_estop_required_but_missing_command(self, t001_data):
        bad = copy.deepcopy(t001_data)
        # Remove the estop_check command
        bad["motion_sequence"] = [
            c for c in bad["motion_sequence"] if c["type"] != "estop_check"
        ]
        result = validate_task_ir(bad)
        assert result.valid
        assert any("A5" in w for w in result.semantic_warnings)

    def test_undefined_frame_reference(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["motion_sequence"][2]["frame_id"] = "nonexistent_frame"
        result = validate_task_ir(bad)
        assert result.valid
        assert any("A6" in w for w in result.semantic_warnings)

    def test_duplicate_seq_warning(self, t001_data):
        bad = copy.deepcopy(t001_data)
        bad["motion_sequence"][1]["seq"] = 0  # duplicate with seq=0
        result = validate_task_ir(bad)
        assert result.valid
        assert any("duplicate" in w.lower() for w in result.semantic_warnings)


# ---------------------------------------------------------------------------
# File-level edge cases
# ---------------------------------------------------------------------------


class TestFileValidation:
    """File I/O edge cases."""

    def test_nonexistent_file(self):
        result = validate_file("/nonexistent/path.json")
        assert not result.valid
        assert any("not found" in e.lower() for e in result.schema_errors)


# ---------------------------------------------------------------------------
# T002: Linear Weld (fenced mode)
# ---------------------------------------------------------------------------


class TestT002Validation:
    """Validate T002 welding task."""

    def test_t002_file_exists(self):
        assert T002_PATH.exists()

    def test_t002_schema_valid(self, t002_data):
        result = validate_task_ir(t002_data)
        assert result.valid, result.summary()

    def test_t002_no_semantic_warnings(self, t002_data):
        result = validate_task_ir(t002_data)
        assert len(result.semantic_warnings) == 0, result.summary()

    def test_t002_task_metadata(self, t002_data):
        assert t002_data["task"]["id"] == "T002"
        assert t002_data["task"]["category"] == "welding"
        assert t002_data["task"]["operating_mode"] == "fenced"

    def test_t002_fenced_speed_limit(self, t002_data):
        speed = t002_data["safety_requirements"]["max_tcp_speed_mm_s"]
        assert speed <= 500.0

    def test_t002_has_orientation_cones(self, t002_data):
        cones = t002_data["safety_requirements"]["forbidden_orientation_cones"]
        assert len(cones) >= 2, "Welding needs upward + operator-side cones"

    def test_t002_has_tool_payload(self, t002_data):
        tp = t002_data["safety_requirements"]["tool_payload"]
        assert tp is not None, "Welding task must define tool payload"
        assert tp["mass_max_kg"] >= 3.0

    def test_t002_motion_has_arc_on_off(self, t002_data):
        types = [c["type"] for c in t002_data["motion_sequence"]]
        assert "set_digital_output" in types, "Welding must have digital output for arc control"

    def test_t002_weld_payload(self, t002_data):
        assert t002_data["robot"]["payload_kg"] == 3.0


# ---------------------------------------------------------------------------
# T003: Palletize (hybrid mode)
# ---------------------------------------------------------------------------


class TestT003Validation:
    """Validate T003 palletizing task."""

    def test_t003_file_exists(self):
        assert T003_PATH.exists()

    def test_t003_schema_valid(self, t003_data):
        result = validate_task_ir(t003_data)
        assert result.valid, result.summary()

    def test_t003_no_semantic_warnings(self, t003_data):
        result = validate_task_ir(t003_data)
        assert len(result.semantic_warnings) == 0, result.summary()

    def test_t003_task_metadata(self, t003_data):
        assert t003_data["task"]["id"] == "T003"
        assert t003_data["task"]["category"] == "palletizing"
        assert t003_data["task"]["operating_mode"] == "hybrid"

    def test_t003_heavy_payload(self, t003_data):
        tp = t003_data["safety_requirements"]["tool_payload"]
        assert tp["mass_max_kg"] >= 4.0

    def test_t003_hybrid_speed_limit(self, t003_data):
        speed = t003_data["safety_requirements"]["max_tcp_speed_mm_s"]
        assert speed <= 300.0

    def test_t003_has_safeguarded_space(self, t003_data):
        sg = t003_data["safety_requirements"]["safeguarded_space"]
        assert len(sg["halfspaces"]) >= 4, "Pallet boundary needs >= 4 halfspaces"

    def test_t003_grip_release_cycle(self, t003_data):
        types = [c["type"] for c in t003_data["motion_sequence"]]
        assert "set_tool" in types, "Need grip/release via set_tool"


# ---------------------------------------------------------------------------
# T004: Multi-waypoint obstacle avoidance (collaborative)
# ---------------------------------------------------------------------------


class TestT004Validation:
    """Validate T004 multi-waypoint task."""

    def test_t004_file_exists(self):
        assert T004_PATH.exists()

    def test_t004_schema_valid(self, t004_data):
        result = validate_task_ir(t004_data)
        assert result.valid, result.summary()

    def test_t004_no_semantic_warnings(self, t004_data):
        result = validate_task_ir(t004_data)
        assert len(result.semantic_warnings) == 0, result.summary()

    def test_t004_task_metadata(self, t004_data):
        assert t004_data["task"]["id"] == "T004"
        assert t004_data["task"]["category"] == "pick_place"
        assert t004_data["task"]["operating_mode"] == "collaborative"

    def test_t004_collab_speed_limit(self, t004_data):
        speed = t004_data["safety_requirements"]["max_tcp_speed_mm_s"]
        assert speed <= 250.0

    def test_t004_multiple_frames(self, t004_data):
        frames = t004_data["environment"]["frames"]
        assert len(frames) >= 3, "Needs base + pick_station + place_station"

    def test_t004_rich_safety_zones(self, t004_data):
        zones = t004_data["environment"]["safety_zones"]
        assert len(zones) >= 2, "Should have confinement + exclusion zones"

    def test_t004_has_exclusion_zones(self, t004_data):
        zones = t004_data["environment"].get("safety_zones", [])
        excl = [z for z in zones if z.get("type") == "exclusion"]
        assert len(excl) >= 1, "T004 must have obstacle exclusion zone"


# ---------------------------------------------------------------------------
# T005: Inspection scan (collaborative)
# ---------------------------------------------------------------------------


class TestT005Validation:
    """Validate T005 inspection task."""

    def test_t005_file_exists(self):
        assert T005_PATH.exists()

    def test_t005_schema_valid(self, t005_data):
        result = validate_task_ir(t005_data)
        assert result.valid, result.summary()

    def test_t005_no_semantic_warnings(self, t005_data):
        result = validate_task_ir(t005_data)
        assert len(result.semantic_warnings) == 0, result.summary()

    def test_t005_task_metadata(self, t005_data):
        assert t005_data["task"]["id"] == "T005"
        assert t005_data["task"]["category"] == "inspection"
        assert t005_data["task"]["operating_mode"] == "collaborative"

    def test_t005_tight_speed_limit(self, t005_data):
        speed = t005_data["safety_requirements"]["max_tcp_speed_mm_s"]
        assert speed <= 150.0

    def test_t005_light_payload(self, t005_data):
        assert t005_data["robot"]["payload_kg"] <= 0.5

    def test_t005_orientation_cones(self, t005_data):
        cones = t005_data["safety_requirements"]["forbidden_orientation_cones"]
        assert len(cones) >= 2


# ---------------------------------------------------------------------------
# Cross-task suite tests (PR-F coverage matrix)
# ---------------------------------------------------------------------------


class TestTaskSuiteCoverage:
    """Verify the 5-task suite covers all categories, modes, and attack surfaces."""

    @pytest.fixture
    def all_tasks(self):
        tasks = {}
        for p in ALL_TASK_PATHS:
            with open(p) as f:
                d = json.load(f)
            tasks[d["task"]["id"]] = d
        return tasks

    def test_all_tasks_schema_valid(self, all_tasks):
        for tid, data in all_tasks.items():
            result = validate_task_ir(data)
            assert result.valid, f"{tid}: {result.summary()}"

    def test_category_coverage(self, all_tasks):
        categories = {d["task"]["category"] for d in all_tasks.values()}
        required = {"pick_place", "welding", "palletizing", "inspection"}
        assert required.issubset(categories), f"Missing: {required - categories}"

    def test_mode_coverage(self, all_tasks):
        modes = {d["task"]["operating_mode"] for d in all_tasks.values()}
        required = {"collaborative", "fenced", "hybrid"}
        assert required.issubset(modes), f"Missing: {required - modes}"

    def test_unique_task_ids(self, all_tasks):
        ids = list(all_tasks.keys())
        assert len(ids) == len(set(ids))

    def test_speed_range_diversity(self, all_tasks):
        speeds = [d["safety_requirements"]["max_tcp_speed_mm_s"]
                  for d in all_tasks.values()]
        assert min(speeds) <= 200, "Should have a slow task"
        assert max(speeds) >= 400, "Should have a fast task"

    def test_payload_range_diversity(self, all_tasks):
        payloads = [d["robot"]["payload_kg"] for d in all_tasks.values()]
        assert min(payloads) <= 0.5, "Should have a light payload task"
        assert max(payloads) >= 3.0, "Should have a heavy payload task"

    def test_all_tasks_have_a1_fields(self, all_tasks):
        for tid, d in all_tasks.items():
            assert "max_tcp_speed_mm_s" in d["safety_requirements"], \
                f"{tid} missing A1 field"

    def test_all_tasks_have_a2_fields(self, all_tasks):
        for tid, d in all_tasks.items():
            assert "safeguarded_space" in d["safety_requirements"], \
                f"{tid} missing A2 field"

    def test_all_tasks_have_a8_fields(self, all_tasks):
        for tid, d in all_tasks.items():
            assert "prompt_security" in d, \
                f"{tid} missing A8 field (prompt_security)"
