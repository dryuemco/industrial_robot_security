"""Tests for Experiment Runner — static watchdog CSV/JSON reports."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import csv
import json
from pathlib import Path

import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scripts.run_experiment import (
    run_experiment,
    _classify_file,
    _compute_summary,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
TASKS_DIR = REPO_ROOT / "enfield_tasks" / "ir" / "tasks"
VARIANTS_DIR = REPO_ROOT / "enfield_attacks" / "generated" / "variants"

# Dynamic counts — derived from actual files on disk
BASELINE_COUNT = len(list(TASKS_DIR.glob("T[0-9][0-9][0-9]_*.json")))
VARIANT_COUNT = len([
    f for f in VARIANTS_DIR.glob("T*_A*_*.json")
    if f.name != "manifest.json"
]) if VARIANTS_DIR.exists() else 0
TOTAL_SCENARIOS = BASELINE_COUNT + VARIANT_COUNT

# Task prefixes for matrix column count
TASK_PREFIXES = sorted({
    f.name.split("_")[0]
    for f in TASKS_DIR.glob("T[0-9][0-9][0-9]_*.json")
})


# ---------------------------------------------------------------------------
# File classifier tests
# ---------------------------------------------------------------------------


class TestClassifyFile:

    def test_baseline(self):
        m = _classify_file("T001_pick_place_collab.json")
        assert m["task_id"] == "T001"
        assert m["role"] == "baseline"
        assert m["attack_type"] == "none"

    def test_variant(self):
        m = _classify_file("T001_A1_pick_place_collab.json")
        assert m["task_id"] == "T001"
        assert m["role"] == "variant"
        assert m["attack_type"] == "A1"

    def test_variant_a8(self):
        m = _classify_file("T005_A8_inspection_scan_collab.json")
        assert m["task_id"] == "T005"
        assert m["attack_type"] == "A8"

    def test_unknown(self):
        m = _classify_file("random_file.json")
        assert m["role"] == "unknown"


# ---------------------------------------------------------------------------
# Full experiment run
# ---------------------------------------------------------------------------


class TestRunExperiment:

    @pytest.fixture(scope="class")
    def experiment(self, tmp_path_factory):
        out = tmp_path_factory.mktemp("results")
        summary = run_experiment(
            baselines_dir=TASKS_DIR,
            variants_dir=VARIANTS_DIR,
            output_dir=out,
        )
        return {"summary": summary, "output_dir": out}

    def test_csv_created(self, experiment):
        csv_path = experiment["output_dir"] / "verdicts.csv"
        assert csv_path.exists()

    def test_csv_row_count(self, experiment):
        csv_path = experiment["output_dir"] / "verdicts.csv"
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        assert len(rows) == TOTAL_SCENARIOS

    def test_csv_has_expected_columns(self, experiment):
        csv_path = experiment["output_dir"] / "verdicts.csv"
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            row = next(reader)
        expected = [
            "file", "task_id", "role", "injected_attack",
            "safe", "violation_count", "a1_detected", "a8_detected",
        ]
        for col in expected:
            assert col in row, f"Missing column: {col}"

    def test_csv_baselines_are_safe(self, experiment):
        csv_path = experiment["output_dir"] / "verdicts.csv"
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            baselines = [r for r in reader if r["role"] == "baseline"]
        assert all(r["safe"] == "True" for r in baselines)

    def test_csv_variants_mostly_flagged(self, experiment):
        csv_path = experiment["output_dir"] / "verdicts.csv"
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            variants = [r for r in reader if r["role"] == "variant"]
        flagged = sum(1 for r in variants if r["safe"] == "False")
        threshold = int(VARIANT_COUNT * 0.75)
        assert flagged >= threshold, f"Only {flagged}/{VARIANT_COUNT} variants flagged"

    def test_summary_json_created(self, experiment):
        json_path = experiment["output_dir"] / "summary.json"
        assert json_path.exists()

    def test_summary_baseline_stats(self, experiment):
        s = experiment["summary"]
        assert s["baselines"]["total"] == BASELINE_COUNT
        assert s["baselines"]["safe"] == BASELINE_COUNT
        assert s["baselines"]["false_positive_rate"] == 0.0

    def test_summary_variant_stats(self, experiment):
        s = experiment["summary"]
        assert s["variants"]["total"] == VARIANT_COUNT
        assert s["variants"]["flagged"] >= int(VARIANT_COUNT * 0.75)
        assert s["variants"]["overall_detection_rate"] >= 0.75

    def test_summary_per_attack_keys(self, experiment):
        s = experiment["summary"]
        for aid in ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"]:
            assert aid in s["per_attack"]
            assert "total" in s["per_attack"][aid]
            assert "detected" in s["per_attack"][aid]
            assert "detection_rate" in s["per_attack"][aid]

    def test_a1_fully_detected(self, experiment):
        s = experiment["summary"]
        assert s["per_attack"]["A1"]["detection_rate"] == 1.0

    def test_a5_fully_detected(self, experiment):
        s = experiment["summary"]
        assert s["per_attack"]["A5"]["detection_rate"] == 1.0

    def test_a8_fully_detected(self, experiment):
        s = experiment["summary"]
        assert s["per_attack"]["A8"]["detection_rate"] == 1.0

    def test_summary_has_metadata(self, experiment):
        s = experiment["summary"]
        assert "metadata" in s
        assert "timestamp" in s["metadata"]
        assert s["metadata"]["baseline_count"] == BASELINE_COUNT

    def test_detection_matrix_created(self, experiment):
        matrix_path = experiment["output_dir"] / "detection_matrix.csv"
        assert matrix_path.exists()

    def test_detection_matrix_shape(self, experiment):
        matrix_path = experiment["output_dir"] / "detection_matrix.csv"
        with open(matrix_path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        # Header + 8 attack rows
        assert len(rows) == 9
        # attack + N tasks + detection_rate
        expected_cols = 1 + len(TASK_PREFIXES) + 1
        assert len(rows[0]) == expected_cols

    def test_detection_matrix_header(self, experiment):
        matrix_path = experiment["output_dir"] / "detection_matrix.csv"
        with open(matrix_path) as f:
            reader = csv.reader(f)
            header = next(reader)
        assert header[0] == "attack"
        assert "T001" in header
        assert header[-1] == "detection_rate"


# ---------------------------------------------------------------------------
# Summary computation unit tests
# ---------------------------------------------------------------------------


class TestComputeSummary:

    def test_all_safe_baselines(self):
        rows = [
            {"role": "baseline", "safe": True, "injected_attack": "none",
             "a1_detected": False, "a2_detected": False, "a3_detected": False,
             "a4_detected": False, "a5_detected": False, "a6_detected": False,
             "a7_detected": False, "a8_detected": False},
        ]
        s = _compute_summary(rows)
        assert s["baselines"]["false_positive_rate"] == 0.0

    def test_perfect_detection(self):
        rows = [
            {"role": "variant", "safe": False, "injected_attack": "A1",
             "a1_detected": True, "a2_detected": False, "a3_detected": False,
             "a4_detected": False, "a5_detected": False, "a6_detected": False,
             "a7_detected": False, "a8_detected": False},
        ]
        s = _compute_summary(rows)
        assert s["per_attack"]["A1"]["detection_rate"] == 1.0

    def test_zero_variants(self):
        rows = [
            {"role": "baseline", "safe": True, "injected_attack": "none",
             "a1_detected": False, "a2_detected": False, "a3_detected": False,
             "a4_detected": False, "a5_detected": False, "a6_detected": False,
             "a7_detected": False, "a8_detected": False},
        ]
        s = _compute_summary(rows)
        assert s["variants"]["total"] == 0
        assert s["variants"]["overall_detection_rate"] == 0.0
