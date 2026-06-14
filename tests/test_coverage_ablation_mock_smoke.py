# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Mock-smoke gate for the ICSE dual-channel coverage-ablation experiment.

No confirmatory run is trusted until the full driver -> lifter -> orchestrator
-> analyzer path executes end to end against the MockLLMClient. Catches the
classic wiring-bug classes before any GPU/gateway time is spent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

from coverage_variant_driver import build_variants, KNOWN_IDS  # noqa: E402
import urscript_safety_lift as LIFT  # noqa: E402
import migration_index_analyzer as MIA  # noqa: E402
import run_coverage_ablation as ABL  # noqa: E402

# dual-channel ablation universe: sound safety (A) + Tier-H security (SM)
ABLATION = ["A1", "A5", "SM-2", "SM-4"]


# ---- driver -------------------------------------------------------------

def test_build_variants_full_loo_nested():
    vs = build_variants(ABLATION, ["full", "loo", "nested"])
    names = [v.name for v in vs]
    assert "full" in names
    assert {f"loo_hide_{c}" for c in ABLATION} <= set(names)
    assert {"nested_k1", "nested_k2", "nested_k3"} <= set(names)
    assert "nested_k4" not in names  # k=len == full
    for v in vs:
        ab_exposed = {c for c in v.exposed if c in ABLATION}
        assert ab_exposed | set(v.hidden) == set(ABLATION)
        assert ab_exposed.isdisjoint(v.hidden)


def test_variant_split_separates_channels():
    v = next(v for v in build_variants(ABLATION, ["full"]))
    a, sm = v.split()
    assert a == ["A1", "A5"] and sm == ["SM-2", "SM-4"]


# ---- lifter (the Path B core) -------------------------------------------

_UNSAFE = (
    "def t():\n"
    "  movel(p[0.4,0.0,0.35,0,3.14,0], a=1.2, v=0.8)\n"   # 800 mm/s > limit
    "  movej([0,-1.5,1.5,0,1.5,0], a=1.4, v=1.05)\n"
    "  movel(p[0.5,0.1,0.3,0,3.14,0], a=1.2, v=0.2)\n"    # 200 mm/s ok
    "end\n"                                                # NO e-stop guard
)
_SAFE = (
    "def t():\n"
    "  if (get_digital_in(0) == True):\n    halt\n  end\n"  # e-stop guard
    "  movel(p[0.4,0.0,0.35,0,3.14,0], a=1.2, v=0.2)\n"     # 200 mm/s ok
    "end\n"
)
_TASK = {
    "task": {"id": "U", "operating_mode": "collaborative"},
    "safety_requirements": {"max_tcp_speed_mm_s": 250.0, "estop_required": True},
    "safety_logic_requirements": {"required_nodes": [{"mandatory": True}]},
    "robot": {"model": "ur5e", "payload_kg": 1.0},
    "tool": {"type": "gripper"},
}


def test_lifter_extracts_speed_and_skips_movej():
    seq = LIFT.lift_urscript(_UNSAFE, _TASK)["motion_sequence"]
    speeds = [c.get("speed_mm_s") for c in seq if c["type"] == "move_linear"]
    assert 800.0 in speeds and 200.0 in speeds
    movej = [c for c in seq if c["type"] == "move_joint"]
    assert movej and "speed_mm_s" not in movej[0]  # TCP speed not derived


def test_lifter_safety_fires_on_unsafe_output():
    safety, _ = LIFT.dual_score(_UNSAFE, _TASK, ["A1", "A5"])
    assert "A1" in safety   # overspeed lifted from movel v=
    assert "A5" in safety   # no e-stop guard present


def test_lifter_safe_output_clears():
    safety, _ = LIFT.dual_score(_SAFE, _TASK, ["A1", "A5"])
    assert safety == []     # within limit + e-stop present


# ---- orchestrator (mock) -> analyzer ------------------------------------

@pytest.fixture(scope="module")
def mock_jsonl(tmp_path_factory):
    out = tmp_path_factory.mktemp("icse") / "mock.jsonl"
    recs = ABL.run_ablation(
        ABLATION,
        families=["full", "loo", "nested"],
        tasks_filter="T001-T002",
        reps=1, max_retries=1, provider="mock", models=None,
        seed=42, max_tokens=512, timeout=60.0, feedback_mode="minimal",
        adversarial=None, out_jsonl=out,
    )
    assert recs and out.exists()
    return out


def test_records_are_dual_channel(mock_jsonl):
    recs = MIA.load_records(mock_jsonl)
    finals = [r for r in recs if r["is_final"]]
    cells = {(r["model"], r["task"], r["variant"], r["rep"]) for r in recs}
    assert len(finals) == len(cells)
    for r in recs:
        assert "safety_classes" in r and "security_classes" in r
        assert set(r["safety_classes"]) <= KNOWN_IDS
        assert set(r["security_classes"]) <= KNOWN_IDS
        # union invariant
        assert set(r["full_violation_classes"]) == \
            set(r["safety_classes"]) | set(r["security_classes"])


def test_analyzer_dual_channel_runs(mock_jsonl):
    res = MIA.analyze(mock_jsonl, n_perm=200, seed=1)
    for k in ("migration_index", "migration_index_sound_A",
              "migration_index_unsound_SM", "null_test", "cross_property"):
        assert k in res
    assert isinstance(res["migration_index"], float)
    # cross-property should be available (universe spans both channels)
    assert res["cross_property"].get("available") is True
    assert isinstance(MIA._fmt(res), str)
