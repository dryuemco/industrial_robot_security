# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the _load_script helper.

Validates the URScript loading dispatch: IR translation route vs
verbatim file read route. Covers argument validation (mutual exclusion,
required-one-of) and file existence checks.

Runs without rclpy or URSim (pure file I/O + translator integration).
"""
from __future__ import annotations

from pathlib import Path

import pytest

from enfield_urscript_runtime.urscript_publisher_node import _load_script


REPO_ROOT = Path(__file__).resolve().parents[2]
T001_IR = (
    REPO_ROOT / 'enfield_tasks' / 'ir' / 'tasks'
    / 'T001_pick_place_collab.json'
)


def test_load_script_neither_set() -> None:
    with pytest.raises(ValueError, match='required'):
        _load_script('', '')


def test_load_script_mutual_exclusion() -> None:
    with pytest.raises(ValueError, match='mutually exclusive'):
        _load_script('foo.json', 'bar.urscript')


def test_load_script_urscript_path_missing(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match='urscript_path'):
        _load_script('', str(tmp_path / 'nope.urscript'))


def test_load_script_task_ir_path_missing(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match='task_ir_path'):
        _load_script(str(tmp_path / 'nope.json'), '')


def test_load_script_urscript_path_reads_verbatim(tmp_path: Path) -> None:
    sample = (
        'def task_demo():\n'
        '  movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.5)\n'
        'end\n'
    )
    p = tmp_path / 'demo.urscript'
    p.write_text(sample)
    assert _load_script('', str(p)) == sample


def test_load_script_task_ir_path_translates() -> None:
    assert T001_IR.is_file(), f'Missing T001 IR fixture: {T001_IR}'
    script = _load_script(str(T001_IR), '')
    # Same structural markers as the existing translation offline test.
    assert 'def task_T001():' in script
    assert '\nend\n' in script
