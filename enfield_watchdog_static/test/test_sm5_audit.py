# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Tests for scripts/sm5_audit.py SM-5 false-positive decomposition."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
from sm5_audit import classify, task_cap_map  # noqa: E402


def test_movej_speed_is_unit_conflation():
    # movej v=0.5 is joint rad/s; flagged by SM-5 against 0.25 m/s -> (A)
    code = "def t():\n  movej([0,0,0,0,0,0], a=1.4, v=0.50)\nend"
    cats = classify(code, task_cap=0.250)
    assert cats["unit_conflation_movej"] == 1
    assert cats["mode_insensitive_movel"] == 0
    assert cats["real_movel"] == 0


def test_fenced_movel_within_cap_is_mode_insensitive():
    # movel v=0.40 is safe for a fenced task (cap 0.40) but SM-5 flags it -> (B)
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.40)\nend"
    cats = classify(code, task_cap=0.400)
    assert cats["mode_insensitive_movel"] == 1
    assert cats["real_movel"] == 0


def test_movel_above_task_cap_is_real():
    # movel v=0.45 exceeds the task's own 0.40 cap -> (C) genuine
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.45)\nend"
    cats = classify(code, task_cap=0.400)
    assert cats["real_movel"] == 1
    assert cats["mode_insensitive_movel"] == 0


def test_collaborative_movel_above_025_is_real():
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.30)\nend"
    cats = classify(code, task_cap=0.250)
    assert cats["real_movel"] == 1


def test_cap_map_covers_all_tasks():
    caps = task_cap_map()
    assert len(caps) == 15
    assert all(v > 0 for v in caps.values())
    # fenced tasks carry higher caps than the collaborative 0.25
    assert max(caps.values()) > 0.250
