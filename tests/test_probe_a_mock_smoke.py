# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""End-to-end mock smoke for scripts/probe_a.py (Track-2 Probe A).

Mirrors tests/test_runner_mock_smoke.py: load the script by path and run it
against MockLLMClient via provider='mock', exercising every glue path
(natural Stage-1/Stage-2, controlled present/absent, non-success handling)
before any Idun call. Pipeline integrity only -- mock output is canned, so no
breach/bucket VALUE is asserted; findings come from the frontier runs.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
for pkg in ("enfield_llm", "enfield_tasks", "enfield_watchdog_static", "scripts"):
    p = str(REPO_ROOT / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


probe_a = _load("probe_a_mod", "scripts/probe_a.py")
REQUIRED = {"arm", "model", "task_id", "rep", "status", "breach"}


@pytest.fixture
def tasks():
    return probe_a.R.load_tasks("T001,T002")


def test_runs_and_covers_arms(tasks, tmp_path):
    rows = probe_a.run_probe_a(tasks, reps=1, output_dir=tmp_path,
                               provider="mock", arms=("natural", "controlled"))
    assert len(rows) == 2 * (1 + 2) * 1
    assert {r["arm"] for r in rows} == {"natural", "controlled"}
    assert {r.get("variant") for r in rows if r["arm"] == "controlled"} == {"present", "absent"}


def test_row_schema_and_no_sham_breach(tasks, tmp_path):
    rows = probe_a.run_probe_a(tasks, reps=1, output_dir=tmp_path, provider="mock")
    for r in rows:
        assert REQUIRED <= set(r)
        assert (r["breach"] is not None) == (r["status"] == "success")


def test_at_least_one_success_reaches_oracle(tasks, tmp_path):
    rows = probe_a.run_probe_a(tasks, reps=1, output_dir=tmp_path, provider="mock")
    assert any(r["status"] == "success" and isinstance(r["breach"], bool) for r in rows)


def test_writer_emits_csv_and_summary(tasks, tmp_path):
    rows = probe_a.run_probe_a(tasks, reps=1, output_dir=tmp_path, provider="mock")
    summary = probe_a.write_results(rows, tmp_path)
    assert (tmp_path / "probe_a_results.csv").exists()
    assert (tmp_path / "probe_a_summary.json").exists()
    assert "natural" in summary and "controlled" in summary
