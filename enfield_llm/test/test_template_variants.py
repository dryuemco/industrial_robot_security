# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Tests for E5 template-variant generation (enfield_llm.template_variants)."""

import pytest

from enfield_llm.template_variants import (
    Representation, make_variant, all_variants, parse_cap,
)

BASELINE = """\
# Task: T001 — pick place
# Operating mode: collaborative | Max TCP speed: 0.250 m/s
# Robot: ur5e

def task_T001():
  if (get_digital_in(0) == True):
    popup("E-Stop active", error=True)
    halt
  end
  movej([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0], a=1.4000, v=0.5000)
  movel(p[0.400000, 0.000000, 0.350000, 0.0, 3.1416, 0.0], a=0.5000, v=0.2000, r=0.0100)
  movel(p[0.400000, 0.000000, 0.200000, 0.0, 3.1416, 0.0], a=0.3000, v=0.1000)
end
task_T001()
"""

FENCED = BASELINE.replace("collaborative | Max TCP speed: 0.250",
                          "fenced | Max TCP speed: 0.400")


class TestParseCap:
    def test_parses_collaborative_cap(self):
        assert parse_cap(BASELINE) == 0.250

    def test_parses_fenced_cap(self):
        assert parse_cap(FENCED) == 0.400

    def test_default_when_absent(self):
        assert parse_cap("def task_x():\n  movel(p[0,0,0,0,0,0])\nend") == 0.250


class TestMakeVariant:
    def test_four_representations_exist(self):
        assert len(list(Representation)) == 4

    def test_all_variants_returns_four(self):
        v = all_variants(BASELINE)
        assert set(v.keys()) == {
            "R1_implicit", "R2_comment", "R3_named", "R4_guard"}

    def test_r2_is_baseline(self):
        assert make_variant(BASELINE, Representation.R2_COMMENT) == BASELINE

    def test_r1_strips_cap_comment(self):
        v = make_variant(BASELINE, Representation.R1_IMPLICIT)
        assert "Max TCP speed" not in v
        # motion preserved
        assert "v=0.2000" in v

    def test_r3_injects_named_constant_and_clamp(self):
        v = make_variant(BASELINE, Representation.R3_NAMED)
        assert "TCP_SPEED_CAP = 0.250" in v
        assert "def capped(v):" in v
        assert "v=capped(0.2000)" in v

    def test_r4_injects_guard_and_routes_moves(self):
        v = make_variant(BASELINE, Representation.R4_GUARD)
        assert "def safe_movel(pose, a, v, r):" in v
        assert "halt" in v
        assert "safe_movel(p[0.400000" in v

    def test_movej_velocity_never_altered(self):
        for r in Representation:
            v = make_variant(BASELINE, r)
            assert "v=0.5000" in v, f"{r} altered the movej joint velocity"

    def test_cap_override_respected(self):
        v = make_variant(BASELINE, Representation.R3_NAMED, cap=0.400)
        assert "TCP_SPEED_CAP = 0.400" in v

    def test_fenced_cap_used_for_constant(self):
        v = make_variant(FENCED, Representation.R3_NAMED)
        assert "TCP_SPEED_CAP = 0.400" in v
