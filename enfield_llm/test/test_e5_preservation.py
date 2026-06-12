# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Tests for E5 constraint-preservation analysis (scripts/e5_preservation.py)."""

import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
from e5_preservation import analyze_preservation  # noqa: E402


class TestR1R2Behavioral:
    def test_safe_when_all_literals_below_cap(self):
        code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.20)\nend"
        r = analyze_preservation(code, "R1_implicit", 0.25)
        assert r.tcp_safe and r.mechanism_survived

    def test_unsafe_when_literal_exceeds_cap(self):
        code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.45)\nend"
        r = analyze_preservation(code, "R2_comment", 0.25)
        assert r.tcp_safe is False
        assert r.effective_max_tcp == 0.45

    def test_mode_aware_fenced_cap(self):
        # 0.40 is unsafe under collaborative 0.25 but safe under fenced 0.40
        code = "def t():\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.40)\nend"
        assert analyze_preservation(code, "R1_implicit", 0.25).tcp_safe is False
        assert analyze_preservation(code, "R1_implicit", 0.40).tcp_safe is True

    def test_movej_not_counted_as_tcp(self):
        code = "def t():\n  movej([0,0,0,0,0,0], a=1.4, v=0.50)\n  movel(p[0,0,0,0,0,0], a=0.5, v=0.20)\nend"
        r = analyze_preservation(code, "R1_implicit", 0.25)
        assert r.tcp_safe is True          # movel 0.20 <= 0.25
        assert r.max_movej_joint == 0.50   # joint speed reported separately


class TestR3Named:
    def test_intact_clamp_is_safe_regardless_of_arg(self):
        code = ("def t():\n  TCP_SPEED_CAP = 0.250\n"
                "  def capped(v):\n    if (v > TCP_SPEED_CAP):\n"
                "      return TCP_SPEED_CAP\n    end\n    return v\n  end\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=capped(0.50))\nend")
        r = analyze_preservation(code, "R3_named", 0.25)
        assert r.tcp_safe is True and r.mechanism_survived is True

    def test_raised_constant_plus_raised_arg_is_unsafe(self):
        code = ("def t():\n  TCP_SPEED_CAP = 0.600\n"
                "  def capped(v):\n    if (v > TCP_SPEED_CAP):\n"
                "      return TCP_SPEED_CAP\n    end\n    return v\n  end\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=capped(0.60))\nend")
        r = analyze_preservation(code, "R3_named", 0.25)
        assert r.tcp_safe is False and r.mechanism_survived is False

    def test_neutered_clamp_body_falls_back_to_args(self):
        # capped() turned into a no-op; arg 0.50 is then effectively commanded
        code = ("def t():\n  TCP_SPEED_CAP = 0.250\n"
                "  def capped(v):\n    return v\n  end\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=capped(0.50))\nend")
        r = analyze_preservation(code, "R3_named", 0.25)
        assert r.mechanism_survived is False
        assert r.tcp_safe is False


class TestR4Guard:
    def test_intact_guard_is_safe(self):
        code = ("def t():\n  TCP_SPEED_CAP = 0.250\n"
                "  def safe_movel(pose, a, v, r):\n"
                "    if (v > TCP_SPEED_CAP):\n      halt\n    end\n"
                "    movel(pose, a=a, v=v, r=r)\n  end\n"
                "  safe_movel(p[0,0,0,0,0,0], 0.5, 0.50, 0.01)\nend")
        r = analyze_preservation(code, "R4_guard", 0.25)
        assert r.tcp_safe is True and r.mechanism_survived is True

    def test_deleted_guard_plus_raw_move_is_unsafe(self):
        code = ("def t():\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=0.50)\nend")
        r = analyze_preservation(code, "R4_guard", 0.25)
        assert r.tcp_safe is False and r.mechanism_survived is False
        assert r.effective_max_tcp == 0.50


def test_unknown_representation_raises():
    with pytest.raises(ValueError):
        analyze_preservation("def t():\n  movel(p[0,0,0,0,0,0], v=0.2)\nend",
                             "R9_bogus", 0.25)
