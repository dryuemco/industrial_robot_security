# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Mock-smoke gate for the ICSE coverage-ablation experiment.

Mirrors the project rule (test_runner_mock_smoke.py): no confirmatory run is
trusted until the full driver -> orchestrator -> analyzer path executes end to
end against the MockLLMClient. Catches the four classic wiring-bug classes
(wrong method name, missing field, arg-order, attribute name) before any GPU
time is spent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

from coverage_variant_driver import (  # noqa: E402
    build_variants, full_watchdog, CoverageVariant, KNOWN_IDS,
)
import migration_index_analyzer as MIA  # noqa: E402
import run_coverage_ablation as ABL  # noqa: E402

ABLATION = ["A1", "A2", "A5"]  # placeholder Tier-S set for the smoke run


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

def test_build_variants_full_loo_nested():
    vs = build_variants(ABLATION, ["full", "loo", "nested"])
    names = [v.name for v in vs]
    assert "full" in names
    assert {"loo_hide_A1", "loo_hide_A2", "loo_hide_A5"} <= set(names)
    # nested proper prefixes only: k=1,2  (k=3 == full)
    assert {"nested_k1", "nested_k2"} <= set(names)
    assert "nested_k3" not in names
    # exposed/hidden partition the ablation set for every variant
    for v in vs:
        ab_exposed = {c for c in v.exposed if c in ABLATION}
        assert ab_exposed | set(v.hidden) == set(ABLATION)
        assert ab_exposed.isdisjoint(v.hidden)


def test_loo_hides_exactly_one():
    vs = build_variants(ABLATION, ["loo"])
    for v in vs:
        assert len(v.hidden) == 1
        assert v.hidden[0] in ABLATION


def test_feedback_watchdog_respects_subset():
    # hide A1 -> the feedback watchdog must NOT enable A1
    v = next(v for v in build_variants(ABLATION, ["loo"]) if v.name == "loo_hide_A1")
    wd = v.feedback_watchdog()
    assert "A1" not in wd._enabled        # safety subset excludes the hidden class
    assert "A2" in wd._enabled
    # full evaluation watchdog enables everything
    assert "A1" in full_watchdog()._enabled


def test_driver_rejects_unknown_and_tiny():
    with pytest.raises(ValueError):
        build_variants(["A1", "NOPE"])
    with pytest.raises(ValueError):
        build_variants(["A1"])  # need >= 2


# --------------------------------------------------------------------------
# Orchestrator (mock) -> Analyzer  (the end-to-end path)
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def mock_jsonl(tmp_path_factory):
    out = tmp_path_factory.mktemp("h2") / "mock.jsonl"
    records = ABL.run_ablation(
        ABLATION,
        families=["full", "loo", "nested"],
        tasks_filter="T001-T002",
        reps=1,
        max_retries=1,
        provider="mock",
        models=None,
        seed=42,
        max_tokens=512,
        timeout=60.0,
        feedback_mode="minimal",
        out_jsonl=out,
    )
    assert records, "ablation produced no records"
    assert out.exists()
    return out


def test_emits_valid_schema(mock_jsonl):
    recs = MIA.load_records(mock_jsonl)   # raises if any required key missing
    assert len(recs) > 0
    # every cell has exactly one final record per (model,task,variant,rep)
    finals = [r for r in recs if r["is_final"]]
    cells = {(r["model"], r["task"], r["variant"], r["rep"]) for r in recs}
    assert len(finals) == len(cells)
    # full_violation_classes only contains known rule IDs
    for r in recs:
        assert set(r["full_violation_classes"]) <= KNOWN_IDS


def test_analyzer_runs_end_to_end(mock_jsonl):
    res = MIA.analyze(mock_jsonl, n_perm=200, seed=1)
    # structural contract -- analyzer must not crash on the mock data
    assert "migration_index" in res
    assert "null_test" in res and "p_value" in res["null_test"]
    assert "escape_modes" in res
    assert res["ablation_classes"], "no ablation classes recovered"
    # migration_index is a float or NaN, never an exception
    mi = res["migration_index"]
    assert isinstance(mi, float)
    # formatting path must also run
    assert isinstance(MIA._fmt(res), str)
