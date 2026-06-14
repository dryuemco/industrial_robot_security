# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Fragment-compliance gate for worked-example interventions.

Each READY worked-example fragment must be compliant for ITS OWN class under the
real lifter + SM rules: injecting it as the "safe form" is only valid if the
fragment itself does not trigger the class it teaches. Placeholder classes are
skipped (they degrade to a hint and are never injected). Run before any
worked_example confirmatory run.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

from urscript_safety_lift import dual_violations  # noqa: E402
from pedagogical_interventions import (  # noqa: E402
    WORKED_EXAMPLE_FRAGMENTS, worked_example_ready,
)

# strictest-mode representative task: collaborative, 250 mm/s cap, estop required
_TASK = {
    "task": {"id": "WE", "operating_mode": "collaborative"},
    "safety_requirements": {"max_tcp_speed_mm_s": 250.0, "estop_required": True},
    "safety_logic_requirements": {"required_nodes": [{"mandatory": True}]},
    "robot": {"model": "ur5e", "payload_kg": 1.0},
    "tool": {"type": "gripper"},
}

READY = sorted(worked_example_ready())


def test_pilot_classes_have_real_fragments():
    # the pilot exercises the safety channel (A8.1 -> A1 + A5)
    assert {"A1", "A5"} <= set(READY)


@pytest.mark.parametrize("cls", READY)
def test_fragment_is_compliant_for_its_own_class(cls):
    frag = WORKED_EXAMPLE_FRAGMENTS[cls]
    safety, security = dual_violations(frag, _TASK, [cls])
    fired = {v.attack_type for v in safety} | {v.attack_type for v in security}
    assert cls not in fired, (
        "worked example for %s itself triggers %s -> the arm is confounded.\n"
        "fragment:\n%s" % (cls, cls, frag))
