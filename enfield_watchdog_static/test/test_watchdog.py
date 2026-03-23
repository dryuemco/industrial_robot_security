"""Tests for Static Watchdog — IR-level A1-A8 rule checker."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import json
from pathlib import Path

import pytest

from enfield_watchdog_static import StaticWatchdog, Violation, WatchdogReport
from enfield_watchdog_static.rules import ALL_RULES
from enfield_watchdog_static.rules.a1_speed import check_a1_speed
from enfield_watchdog_static.rules.a2_zone import check_a2_zone
from enfield_watchdog_static.rules.a3_orientation import check_a3_orientation
from enfield_watchdog_static.rules.a4_payload import check_a4_payload
from enfield_watchdog_static.rules.a5_logic import check_a5_logic
from enfield_watchdog_static.rules.a6_frame import check_a6_frame
from enfield_watchdog_static.rules.a7_tool import check_a7_tool
from enfield_watchdog_static.rules.a8_prompt import check_a8_prompt

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

TASKS_DIR = Path(__file__).resolve().parent.parent.parent / "enfield_tasks" / "ir" / "tasks"
VARIANTS_DIR = Path(__file__).resolve().parent.parent.parent / "enfield_attacks" / "generated" / "variants"

BASELINE_FILES = sorted(TASKS_DIR.glob("T[0-9][0-9][0-9]_*.json"))
VARIANT_FILES = sorted(
    f for f in VARIANTS_DIR.glob("T*_A*_*.json")
    if f.name != "manifest.json"
) if VARIANTS_DIR.exists() else []

# Dynamic counts — no hardcoding
BASELINE_COUNT = len(BASELINE_FILES)
VARIANT_COUNT = len(VARIANT_FILES)

# Extract task prefixes dynamically (e.g., ["T001", "T002", ..., "T015"])
TASK_PREFIXES = sorted({f.name.split("_")[0] for f in BASELINE_FILES})


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def wd() -> StaticWatchdog:
    return StaticWatchdog()


def _load(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


@pytest.fixture
def t001() -> dict:
    return _load(TASKS_DIR / "T001_pick_place_collab.json")


@pytest.fixture(params=TASK_PREFIXES)
def any_baseline(request) -> dict:
    bf = [f for f in BASELINE_FILES if f.name.startswith(request.param)][0]
    return _load(bf)


def _variant(task_prefix: str, attack: str) -> dict:
    """Load a specific variant file."""
    matches = [f for f in VARIANT_FILES
                if f.name.startswith(f"{task_prefix}_{attack}_")]
    if not matches:
        pytest.skip(f"Variant {task_prefix}_{attack} not found")
    return _load(matches[0])


# ---------------------------------------------------------------------------
# Rule registry
# ---------------------------------------------------------------------------


class TestRuleRegistry:

    def test_all_8_rules_registered(self):
        assert set(ALL_RULES.keys()) == {"A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"}

    def test_each_rule_is_callable(self):
        for aid, fn in ALL_RULES.items():
            assert callable(fn), f"{aid} rule is not callable"


# ---------------------------------------------------------------------------
# Baselines should be CLEAN (0 violations)
# ---------------------------------------------------------------------------


class TestBaselinesSafe:
    """All baseline tasks must pass with 0 violations."""

    def test_baseline_is_safe(self, wd, any_baseline):
        report = wd.analyze(any_baseline)
        assert report.safe, (
            f"{report.task_id} should be safe but got:\n{report.summary()}"
        )

    def test_baseline_runs_all_checks(self, wd, any_baseline):
        report = wd.analyze(any_baseline)
        assert report.checks_run == 8

    def test_baseline_summary_says_pass(self, wd, any_baseline):
        report = wd.analyze(any_baseline)
        assert "PASS" in report.summary()


# ---------------------------------------------------------------------------
# A1: Speed Injection detection
# ---------------------------------------------------------------------------


class TestA1Detection:

    def test_detects_speed_violation(self):
        variant = _variant("T001", "A1")
        violations = check_a1_speed(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A1" for v in violations)

    def test_severity_positive(self):
        variant = _variant("T001", "A1")
        violations = check_a1_speed(variant)
        assert all(v.severity > 0 for v in violations)

    def test_all_task_a1_variants_detected(self):
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A1")
            violations = check_a1_speed(variant)
            assert len(violations) > 0, f"{prefix}_A1 not detected"


# ---------------------------------------------------------------------------
# A2: Zone Penetration detection
# ---------------------------------------------------------------------------


class TestA2Detection:

    def test_detects_zone_violation(self):
        variant = _variant("T001", "A2")
        violations = check_a2_zone(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A2" for v in violations)

    def test_penetration_depth_positive(self):
        variant = _variant("T001", "A2")
        violations = check_a2_zone(variant)
        assert all(v.severity > 0 for v in violations)

    def test_all_task_a2_variants_detected(self):
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A2")
            violations = check_a2_zone(variant)
            assert len(violations) > 0, f"{prefix}_A2 not detected"


# ---------------------------------------------------------------------------
# A3: Orientation Anomaly detection
# ---------------------------------------------------------------------------


class TestA3Detection:

    def test_detects_orientation_violation(self):
        variant = _variant("T001", "A3")
        violations = check_a3_orientation(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A3" for v in violations)

    def test_angular_penetration_positive(self):
        variant = _variant("T001", "A3")
        violations = check_a3_orientation(variant)
        assert all(v.severity > 0 for v in violations)

    def test_most_tasks_with_cones_detected(self):
        """Most tasks with orientation cones should detect A3 variants."""
        detected = 0
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A3")
            violations = check_a3_orientation(variant)
            if len(violations) > 0:
                detected += 1
        # At least 80% of tasks should detect A3 (some may have no cones)
        assert detected >= int(len(TASK_PREFIXES) * 0.80), (
            f"Only {detected}/{len(TASK_PREFIXES)} tasks detected A3"
        )


# ---------------------------------------------------------------------------
# A4: Payload Misconfiguration detection
# ---------------------------------------------------------------------------


class TestA4Detection:

    def test_detects_payload_violation(self):
        variant = _variant("T001", "A4")
        violations = check_a4_payload(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A4" for v in violations)

    def test_all_task_a4_variants_detected(self):
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A4")
            violations = check_a4_payload(variant)
            assert len(violations) > 0, f"{prefix}_A4 not detected"


# ---------------------------------------------------------------------------
# A5: Logic Bypass detection
# ---------------------------------------------------------------------------


class TestA5Detection:

    def test_detects_missing_estop(self):
        variant = _variant("T001", "A5")
        violations = check_a5_logic(variant)
        estop_violations = [v for v in violations if "E-Stop" in v.description or "estop" in v.description.lower()]
        assert len(estop_violations) > 0

    def test_all_task_a5_variants_detected(self):
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A5")
            violations = check_a5_logic(variant)
            assert len(violations) > 0, f"{prefix}_A5 not detected"


# ---------------------------------------------------------------------------
# A6: Frame Confusion detection
# ---------------------------------------------------------------------------


class TestA6Detection:

    def test_detects_frame_deviation(self):
        variant = _variant("T002", "A6")
        violations = check_a6_frame(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A6" for v in violations)

    def test_most_task_a6_variants_detected(self):
        """A6 detection rate should be at least 75%."""
        detected = 0
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A6")
            violations = check_a6_frame(variant)
            if len(violations) > 0:
                detected += 1
        assert detected >= int(len(TASK_PREFIXES) * 0.75), (
            f"Only {detected}/{len(TASK_PREFIXES)} tasks detected A6"
        )


# ---------------------------------------------------------------------------
# A7: Tool Misuse detection
# ---------------------------------------------------------------------------


class TestA7Detection:

    def test_detects_tool_mismatch(self):
        variant = _variant("T001", "A7")
        violations = check_a7_tool(variant)
        assert len(violations) > 0
        assert any(v.attack_type == "A7" for v in violations)

    def test_most_task_a7_variants_detected(self):
        """A7 detection rate should be at least 80%."""
        detected = 0
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A7")
            violations = check_a7_tool(variant)
            if len(violations) > 0:
                detected += 1
        assert detected >= int(len(TASK_PREFIXES) * 0.80), (
            f"Only {detected}/{len(TASK_PREFIXES)} tasks detected A7"
        )


# ---------------------------------------------------------------------------
# A8: Prompt Injection detection
# ---------------------------------------------------------------------------


class TestA8Detection:

    def test_detects_weakened_prompt_security(self):
        variant = _variant("T001", "A8")
        violations = check_a8_prompt(variant)
        assert len(violations) > 0
        assert all(v.attack_type == "A8" for v in violations)

    def test_all_task_a8_variants_detected(self):
        for prefix in TASK_PREFIXES:
            variant = _variant(prefix, "A8")
            violations = check_a8_prompt(variant)
            assert len(violations) > 0, f"{prefix}_A8 not detected"


# ---------------------------------------------------------------------------
# A8 combo: also triggers A1 + A5
# ---------------------------------------------------------------------------


class TestA8Combo:
    """A8 variants inject A1 (speed) + A5 (estop removed). Full watchdog should catch all."""

    def test_a8_triggers_a1_and_a5_and_a8(self, wd):
        variant = _variant("T001", "A8")
        report = wd.analyze(variant)
        detected = {v.attack_type for v in report.violations}
        assert "A1" in detected, "A8 should induce A1 (speed violation)"
        assert "A5" in detected, "A8 should induce A5 (estop removed)"
        assert "A8" in detected, "A8 prompt security bypass should be detected"


# ---------------------------------------------------------------------------
# Orchestrator tests
# ---------------------------------------------------------------------------


class TestWatchdogOrchestrator:

    def test_analyze_returns_report(self, wd, t001):
        report = wd.analyze(t001)
        assert isinstance(report, WatchdogReport)
        assert report.task_id == "T001"

    def test_analyze_file(self, wd):
        report = wd.analyze_file(TASKS_DIR / "T001_pick_place_collab.json")
        assert report.task_id == "T001"
        assert report.source_file == "T001_pick_place_collab.json"

    def test_enabled_attacks_subset(self, t001):
        wd = StaticWatchdog(enabled_attacks=["A1", "A2"])
        report = wd.analyze(t001)
        assert report.checks_run == 2

    def test_batch_analyze_baselines(self, wd):
        reports = wd.batch_analyze(TASKS_DIR, pattern="T[0-9][0-9][0-9]_*.json")
        assert len(reports) == BASELINE_COUNT
        assert all(r.safe for r in reports)

    def test_batch_analyze_variants(self, wd):
        if not VARIANT_FILES:
            pytest.skip("Variants not found")
        reports = wd.batch_analyze(VARIANTS_DIR, pattern="T*_A*_*.json")
        assert len(reports) == VARIANT_COUNT
        flagged = [r for r in reports if not r.safe]
        assert len(flagged) >= int(VARIANT_COUNT * 0.75), (
            f"Expected >= {int(VARIANT_COUNT * 0.75)} flagged variants, got {len(flagged)}"
        )


# ---------------------------------------------------------------------------
# Report output
# ---------------------------------------------------------------------------


class TestReportOutput:

    def test_report_to_dict(self, wd, t001):
        report = wd.analyze(t001)
        d = report.to_dict()
        assert "task_id" in d
        assert "violations" in d
        assert d["safe"] is True

    def test_report_json_output(self, wd, tmp_path):
        reports = wd.batch_analyze(TASKS_DIR, pattern="T[0-9][0-9][0-9]_*.json")
        out = tmp_path / "report.json"
        wd.batch_report_json(reports, out)
        assert out.exists()
        with open(out) as f:
            data = json.load(f)
        assert data["total_files"] == BASELINE_COUNT
        assert data["safe_count"] == BASELINE_COUNT

    def test_variant_report_has_violations(self, wd, tmp_path):
        if not VARIANT_FILES:
            pytest.skip("Variants not found")
        reports = wd.batch_analyze(VARIANTS_DIR, pattern="T*_A*_*.json")
        out = tmp_path / "variant_report.json"
        wd.batch_report_json(reports, out)
        with open(out) as f:
            data = json.load(f)
        assert data["violation_count"] > 0


# ---------------------------------------------------------------------------
# Violation data model
# ---------------------------------------------------------------------------


class TestViolationModel:

    def test_violation_to_dict(self):
        v = Violation(
            attack_type="A1", iso_clause="5.6",
            detection_mechanism="DM-1",
            description="test", severity=1.5,
        )
        d = v.to_dict()
        assert d["attack_type"] == "A1"
        assert d["severity"] == 1.5

    def test_report_safe_when_empty(self):
        r = WatchdogReport(task_id="T999")
        assert r.safe
        assert r.violation_count == 0
        assert r.max_severity() == 0.0

    def test_report_not_safe_with_violations(self):
        r = WatchdogReport(task_id="T999", violations=[
            Violation("A1", "5.6", "DM-1", "test", 1.0),
        ])
        assert not r.safe
        assert r.violation_count == 1

    def test_violations_by_attack(self):
        r = WatchdogReport(task_id="T999", violations=[
            Violation("A1", "5.6", "DM-1", "speed", 1.0),
            Violation("A1", "5.6", "DM-1", "speed2", 0.5),
            Violation("A5", "5.4", "DM-4", "estop", 1.0),
        ])
        by_attack = r.violations_by_attack()
        assert len(by_attack["A1"]) == 2
        assert len(by_attack["A5"]) == 1

    def test_has_attack(self):
        r = WatchdogReport(task_id="T999", violations=[
            Violation("A1", "5.6", "DM-1", "test", 1.0),
        ])
        assert r.has_attack("A1")
        assert not r.has_attack("A2")


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


class TestEdgeCases:

    def test_minimal_task(self, wd):
        task = {
            "task": {"id": "T999", "name": "minimal", "operating_mode": "collaborative"},
            "robot": {"adapter": "enfield_robots_ur5e", "model": "ur5e"},
            "motion_sequence": [{"seq": 0, "type": "comment", "comment_text": "nothing"}],
        }
        report = wd.analyze(task)
        assert isinstance(report, WatchdogReport)
        assert report.checks_run == 8

    def test_no_safety_requirements(self, wd):
        task = {
            "task": {"id": "T999", "name": "bare", "operating_mode": "fenced"},
            "robot": {"adapter": "enfield_robots_ur5e", "model": "ur5e"},
            "motion_sequence": [{"seq": 0, "type": "move_joint", "speed_mm_s": 100.0,
                                  "target_joints": {"values": [0]*6}}],
        }
        report = wd.analyze(task)
        assert report.checks_run == 8


# ---------------------------------------------------------------------------
# Detection matrix: every attack type detected on at least one variant
# ---------------------------------------------------------------------------


class TestDetectionMatrix:
    """Verify the watchdog detects each A1-A8 across the full variant set."""

    def test_full_detection_matrix(self, wd):
        if not VARIANT_FILES:
            pytest.skip("Variants not found")

        detected_attacks: set[str] = set()
        for vf in VARIANT_FILES:
            task = _load(vf)
            report = wd.analyze(task, source_file=vf.name)
            for v in report.violations:
                detected_attacks.add(v.attack_type)

        expected = {"A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"}
        missing = expected - detected_attacks
        assert len(missing) == 0, f"Attacks not detected: {missing}"
