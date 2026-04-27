# SPDX-License-Identifier: Apache-2.0
"""Offline test: T001 IR -> URScript structural sanity.

Runs without URSim or rclpy execution. Validates that the translator
output (which the publisher node would broadcast) has the structural
markers ur_robot_driver expects.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from enfield_translators.urscript_translator import IRToURScriptTranslator


REPO_ROOT = Path(__file__).resolve().parents[2]
T001_IR = (
    REPO_ROOT / 'enfield_tasks' / 'ir' / 'tasks'
    / 'T001_pick_place_collab.json'
)


@pytest.fixture(scope='module')
def t001_script() -> str:
    assert T001_IR.is_file(), f'Missing T001 IR: {T001_IR}'
    return IRToURScriptTranslator().translate_file(T001_IR)


def test_program_function_defined(t001_script: str) -> None:
    assert 'def task_T001():' in t001_script


def test_program_terminated(t001_script: str) -> None:
    # URScript program function must be closed with `end`.
    assert '\nend\n' in t001_script


def test_entry_point_invoked(t001_script: str) -> None:
    assert 'task_T001()' in t001_script


def test_contains_motion_primitives(t001_script: str) -> None:
    # T001 uses move_joint + move_linear -> movej + movel calls.
    assert 'movel(' in t001_script
    assert 'movej(' in t001_script


def test_unit_conversion_applied(t001_script: str) -> None:
    # Header reports max TCP speed in m/s (250 mm/s -> 0.250 m/s).
    assert '0.250 m/s' in t001_script


def test_nontrivial_size(t001_script: str) -> None:
    # 13-step motion sequence + header should yield >= 30 lines.
    assert t001_script.count('\n') >= 30


def test_simulation_only_warning_present(t001_script: str) -> None:
    # Header must carry the SIMULATION ONLY warning.
    assert 'SIMULATION ONLY' in t001_script
