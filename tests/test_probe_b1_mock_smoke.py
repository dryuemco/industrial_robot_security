# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""End-to-end mock smoke for scripts/probe_b1.py (Track-2 Probe B1)."""
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


probe_b1 = _load("probe_b1_mod", "scripts/probe_b1.py")
REQUIRED = {"intent", "representation", "model", "task_id", "rep", "status",
            "protected", "breach"}
INTENTS = ("perf", "readability", "feature", "bugfix")


@pytest.fixture
def tasks():
    return probe_b1.R.load_tasks("T001,T002")


def test_runs_and_covers_intents(tasks, tmp_path):
    rows = probe_b1.run_probe_b1(tasks, reps=1, output_dir=tmp_path,
                                 provider="mock", intents=INTENTS,
                                 representations=("R4_guard",))
    assert len(rows) == 2 * len(INTENTS) * 1
    assert {r["intent"] for r in rows} == set(INTENTS)


def test_row_schema_and_no_sham_breach(tasks, tmp_path):
    rows = probe_b1.run_probe_b1(tasks, reps=1, output_dir=tmp_path, provider="mock",
                                 intents=INTENTS, representations=("R4_guard",))
    for r in rows:
        assert REQUIRED <= set(r)
        # protected/breach scored iff success
        assert (r["breach"] is not None) == (r["status"] == "success")
        assert (r["protected"] is not None) == (r["status"] == "success")


def test_bugfix_path_does_not_crash(tasks, tmp_path):
    rows = probe_b1.run_probe_b1(tasks, reps=1, output_dir=tmp_path, provider="mock",
                                 intents=("bugfix",), representations=("R4_guard",))
    assert len(rows) == 2 and all(r["intent"] == "bugfix" for r in rows)


def test_writer_emits_csv_and_summary(tasks, tmp_path):
    rows = probe_b1.run_probe_b1(tasks, reps=1, output_dir=tmp_path, provider="mock",
                                 intents=INTENTS, representations=("R4_guard",))
    summary = probe_b1.write_results(rows, tmp_path)
    assert (tmp_path / "probe_b1_results.csv").exists()
    assert (tmp_path / "probe_b1_summary.json").exists()
