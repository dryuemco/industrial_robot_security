# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Mock-smoke for the --intervention orchestrator hook (additive).

Verifies the strategy module wires into run_ablation under the MockLLMClient:
class-id alignment, arm stamping on every record, the legacy path staying
unchanged when --intervention is absent, and (when the mock triggers a repair
turn) socratic recording its extra diagnosis call. Catches the wiring-bug
classes before any gateway time is spent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

from coverage_variant_driver import KNOWN_IDS  # noqa: E402
import run_coverage_ablation as ABL  # noqa: E402
import migration_index_analyzer as MIA  # noqa: E402
from pedagogical_interventions import CLASS_META  # noqa: E402

ABLATION = ["A1", "A5", "SM-2", "SM-4"]


def test_class_meta_ids_align_with_known_ids():
    # drift guard: every metadata/fragment key must be a real rule id
    assert set(CLASS_META) <= set(KNOWN_IDS)


def _run(tmp, intervention):
    out = tmp / ("mock_%s.jsonl" % (intervention or "legacy"))
    recs = ABL.run_ablation(
        ABLATION,
        families=["full"],
        tasks_filter="T001-T002",
        reps=1, max_retries=2, provider="mock", models=None,
        seed=42, max_tokens=512, timeout=60.0, feedback_mode="minimal",
        intervention=intervention, adversarial=None, out_jsonl=out,
    )
    assert recs and out.exists()
    return MIA.load_records(out)


def test_legacy_path_unchanged_when_intervention_absent(tmp_path):
    recs = _run(tmp_path, None)
    for r in recs:
        assert r.get("intervention") == "legacy_minimal"
        assert "diagnosis_calls" not in r


@pytest.mark.parametrize("iv", ["naive_minimal", "worked_example", "socratic"])
def test_intervention_runs_and_stamps_arm(tmp_path, iv):
    recs = _run(tmp_path, iv)
    assert recs
    for r in recs:
        assert r.get("intervention") == iv
        # dispatch must not corrupt the dual-channel scoring invariant
        assert set(r["full_violation_classes"]) == \
            set(r["safety_classes"]) | set(r["security_classes"])


def test_naive_minimal_is_byte_identical_to_legacy(tmp_path):
    # naive_minimal pins the legacy renderer; only the arm label differs
    legacy = _run(tmp_path, None)
    naive = _run(tmp_path, "naive_minimal")
    strip = lambda rs: [{k: v for k, v in r.items() if k != "intervention"}
                        for r in rs]
    assert strip(legacy) == strip(naive)


def test_socratic_records_diagnosis_when_a_repair_turn_occurs(tmp_path):
    recs = _run(tmp_path, "socratic")
    had_retry = max((r["iteration"] for r in recs), default=0) > 0
    if had_retry:  # only assertable if the mock actually drove a repair turn
        assert any(r.get("diagnosis_calls") for r in recs)
        for r in recs:
            for d in r.get("diagnosis_calls", []):
                assert d.get("prompt") and "iteration" in d
                assert d["prompt"].startswith("Your URScript:")
