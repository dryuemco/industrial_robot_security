# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Mock-smoke for pedagogical_interventions strategy logic (no LLM/lifter).

This is the strategy-layer gate. The separate, repo-side gate
(test_worked_example_fragments_pass_lifter) must run each worked-example
fragment through the real urscript_safety_lift + SM rules and assert zero
violations for its target class -- that one needs the rule API.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

from pedagogical_interventions import (  # noqa: E402
    build_intervention,
    INTERVENTION_NAMES,
    InterventionAction,
    worked_example_ready,
)

ABLATION = frozenset({"A1", "A5", "SM-2", "SM-4", "SM-5"})
IMPLEMENTED = ["naive_minimal", "naive_rich", "worked_example",
               "socratic", "productive_failure"]
STUBBED = ["scaffold_fade", "mastery_gate"]


@pytest.mark.parametrize("name", IMPLEMENTED)
def test_step_produces_wellformed_action(name):
    iv = build_intervention(name)
    for it in range(4):
        act = iv.step(it, [], ABLATION, "T001",
                      "movel(p[0.4,-0.2,0.3,0,3.14,0], a=1.2, v=0.9, r=0)")
        assert isinstance(act, InterventionAction)
        if not act.skip_feedback:
            diag = "MOCK DIAGNOSIS" if act.needs_diagnosis_call else None
            assert isinstance(act.render(diag), str)


def test_all_implemented_empty_violations_no_feedback():
    for name in IMPLEMENTED:
        iv = build_intervention(name)
        act = iv.step(0, [], frozenset(), "T001", "code")
        assert act.skip_feedback or act.render(None) == ""


def test_naive_minimal_lists_classes_and_is_short():
    act = build_intervention("naive_minimal").step(
        1, [], frozenset({"A1", "A5"}), "T", "c")
    body = act.render(None)
    assert "A1" in body and "A5" in body
    assert len(body) < 200  # genuinely minimal baseline


def test_naive_rich_includes_refs_and_hints():
    act = build_intervention("naive_rich").step(1, [], frozenset({"A1"}), "T", "c")
    body = act.render(None)
    assert "5.5.3" in body and "0.25 m/s" in body


def test_worked_example_never_emits_placeholder():
    act = build_intervention("worked_example").step(0, [], ABLATION, "T", "c")
    assert "PLACEHOLDER" not in act.render(None)


def test_worked_example_injects_real_fragment_for_a1():
    act = build_intervention("worked_example").step(
        0, [], frozenset({"A1"}), "T", "c")
    assert "v=0.25" in act.render(None)  # the safe exemplar, not a hint


def test_worked_example_ready_reports_only_real_fragments():
    ready = worked_example_ready()
    assert "A1" in ready
    assert (ABLATION - ready), "expected some classes to still lack fragments"


def test_socratic_requests_diagnosis_and_withholds_offending_line():
    code = "movel(p[0.4,-0.2,0.3,0,3.14,0], a=1.2, v=0.9, r=0)"
    act = build_intervention("socratic").step(1, [], frozenset({"A1"}), "T", code)
    assert act.needs_diagnosis_call and act.diagnosis_prompt
    assert "v=0.9" not in act.diagnosis_prompt          # must not reveal the line
    with pytest.raises(ValueError):
        act.render(None)                                # diagnosis is mandatory
    folded = act.render("A1: 'v=0.9' exceeds the 0.25 m/s limit.")
    assert "v=0.9" in folded and "correct" in folded.lower()


def test_productive_failure_free_then_structured():
    iv = build_intervention("productive_failure")
    assert iv.step(0, [], ABLATION, "T", "c").skip_feedback
    assert iv.step(1, [], ABLATION, "T", "c").skip_feedback
    late = iv.step(2, [], ABLATION, "T", "c")
    assert not late.skip_feedback and late.render(None)


@pytest.mark.parametrize("name", STUBBED)
def test_stub_raises_not_implemented(name):
    with pytest.raises(NotImplementedError):
        build_intervention(name).step(0, [], ABLATION, "T", "c")


def test_registry_matches_seven_canonical_names():
    assert set(INTERVENTION_NAMES) == {
        "naive_minimal", "naive_rich", "worked_example", "socratic",
        "scaffold_fade", "mastery_gate", "productive_failure",
    }


def test_unknown_intervention_raises():
    with pytest.raises(ValueError):
        build_intervention("nonexistent")
