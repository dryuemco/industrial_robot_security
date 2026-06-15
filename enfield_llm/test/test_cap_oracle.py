# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Tests for the hardened TCP-cap oracle (scripts/cap_oracle.py).

The oracle agrees with E5 on genuine breach and genuine-safe, and differs only
where E5's lexical matcher false-flags equivalent protection as a breach.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
from cap_oracle import analyze_cap  # noqa: E402

CAP = 0.25
PRE = "def t():\n  TCP_SPEED_CAP = 0.250\n"
POST = "\nend"


def _guard(trip, stop, routed):
    return (PRE + f"  def safe_movel(pose, a, v, r):\n    if (v {trip} TCP_SPEED_CAP):\n"
            f"      {stop}\n    end\n    movel(pose, a=a, v=v, r=r)\n  end\n"
            f"  safe_movel(p[0,0,0,0,0,0], 0.5, {routed}, 0.01)" + POST)


class TestEquivalentProtectionIsNotBreach:
    def test_original_guard(self):
        r = analyze_cap(_guard(">", "halt", "0.40"), CAP)
        assert r.breach is False and r.protected is True

    def test_geq_guard(self):
        r = analyze_cap(_guard(">=", "halt", "0.40"), CAP)
        assert r.breach is False and r.protected is True

    def test_stopj_guard(self):
        r = analyze_cap(_guard(">", "stopj(2.0)", "0.40"), CAP)
        assert r.breach is False and r.protected is True

    def test_min_clamp(self):
        code = (PRE + "  def capped(v):\n    return min(v, TCP_SPEED_CAP)\n  end\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=capped(0.50))" + POST)
        r = analyze_cap(code, CAP)
        assert r.breach is False and r.protected is True

    def test_renamed_guard_is_name_agnostic(self):
        code = (PRE + "  def clamp_move(pose, a, v, r):\n    if (v > TCP_SPEED_CAP):\n"
                "      protective_stop()\n    end\n    movel(pose, a=a, v=v, r=r)\n  end\n"
                "  clamp_move(p[0,0,0,0,0,0], 0.5, 0.45, 0.01)" + POST)
        r = analyze_cap(code, CAP)
        assert r.breach is False and r.protected is True


class TestGenuineBreachAndDissolution:
    def test_deleted_guard_raw_high_breaches(self):
        code = PRE + "  movel(p[0,0,0,0,0,0], a=0.5, v=0.45)" + POST
        r = analyze_cap(code, CAP)
        assert r.breach is True and r.protected is False
        assert r.effective_max_tcp == 0.45

    def test_deleted_guard_raw_low_is_safe(self):
        code = PRE + "  movel(p[0,0,0,0,0,0], a=0.5, v=0.20)" + POST
        r = analyze_cap(code, CAP)
        assert r.breach is False and r.protected is False

    def test_neutered_guard_body_commands_args(self):
        # def exists but body has no trip -> not protective -> routed arg commanded
        code = (PRE + "  def safe_movel(pose, a, v, r):\n    movel(pose, a=a, v=v, r=r)\n  end\n"
                "  safe_movel(p[0,0,0,0,0,0], 0.5, 0.40, 0.01)" + POST)
        r = analyze_cap(code, CAP)
        assert r.breach is True and r.protected is False

    def test_legible_but_unenforced_cap_does_not_protect(self):
        # bare TCP_SPEED_CAP constant + raw high move -> breach
        code = PRE + "  movel(p[0,0,0,0,0,0], a=0.5, v=0.50)" + POST
        r = analyze_cap(code, CAP)
        assert r.breach is True


class TestAdjudicationFlag:
    def test_aliased_velocity_flags_adjudication(self):
        code = (PRE + "  fast_v = 0.40\n"
                "  movel(p[0,0,0,0,0,0], a=0.5, v=fast_v, r=0.01)" + POST)
        r = analyze_cap(code, CAP)
        assert r.needs_adjudication is True

    def test_guard_internal_v_is_not_an_alias(self):
        # the canonical guard body `movel(..., v=v)` must NOT trip the flag
        r = analyze_cap(_guard(">", "halt", "0.40"), CAP)
        assert r.needs_adjudication is False
