# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Tests for the CANDIDATE corrected SM-5 rule (scripts/sm5_corrected.py).

Demonstrates that the corrected rule eliminates the 41/41 baseline false
positives while still catching genuine TCP-cap breaches. NOT a test of the
active watchdog (which is unchanged).
"""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
from sm5_corrected import check_sm5_corrected, JOINT_SPEED_LIMIT  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
URSCRIPT = REPO / "enfield_translators" / "generated" / "urscript"
_CAP = __import__("re").compile(r"Max TCP speed:\s*([0-9]*\.?[0-9]+)")


def _cap(code: str) -> float:
    m = _CAP.search(code)
    return float(m.group(1)) if m else 0.250


def test_corrected_rule_zero_on_all_safe_baselines():
    entries = json.load((URSCRIPT / "manifest.json").open())
    total = 0
    for e in entries:
        code = (URSCRIPT / e["output"]).read_text()
        total += len(check_sm5_corrected(code, tcp_cap=_cap(code)))
    assert total == 0, f"corrected SM-5 should be 0 on safe baselines, got {total}"


def test_movej_normal_speed_not_flagged():
    code = "def t():\n  movej([0,0,0,0,0,0], a=1.4, v=0.50)\nend"
    assert check_sm5_corrected(code, tcp_cap=0.250) == []


def test_movej_excessive_joint_speed_flagged():
    code = f"def t():\n  movej([0,0,0,0,0,0], a=1.4, v={JOINT_SPEED_LIMIT + 1})\nend"
    v = check_sm5_corrected(code, tcp_cap=0.250)
    assert len(v) == 1 and v[0].metadata["kind"] == "joint_movej"


def test_movel_above_collaborative_cap_flagged():
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.30)\nend"
    v = check_sm5_corrected(code, tcp_cap=0.250)
    assert len(v) == 1 and v[0].metadata["kind"] == "tcp_movel"


def test_movel_within_fenced_cap_not_flagged():
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.40)\nend"
    assert check_sm5_corrected(code, tcp_cap=0.400) == []


def test_movel_above_fenced_cap_flagged():
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.55)\nend"
    assert len(check_sm5_corrected(code, tcp_cap=0.400)) == 1


def test_default_cap_is_collaborative():
    code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.30)\nend"
    # no tcp_cap supplied -> defaults to 0.25 -> 0.30 flagged
    assert len(check_sm5_corrected(code)) == 1
