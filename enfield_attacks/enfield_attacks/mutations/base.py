# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Abstract base class for Task IR mutations."""

from __future__ import annotations

import copy
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


@dataclass
class MutationResult:
    """Describes what a mutation did to a task."""

    attack_type: str
    iso_clause: str
    description: str
    mutated_fields: list[str] = field(default_factory=list)
    severity_estimate: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)


class Mutation(ABC):
    """Base class for A1–A8 attack mutations.

    Each subclass implements ``apply()`` which receives a deep copy
    of a baseline task dict and returns the mutated version plus a
    ``MutationResult`` describing the changes.
    """

    attack_id: str = ""
    iso_clause: str = ""
    description: str = ""

    def __init__(self, seed: int = 42) -> None:
        self.seed = seed

    @abstractmethod
    def apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        """Mutate *task* (already a deep copy) and return (mutated_task, result).

        Implementations MUST NOT modify the original dict passed in;
        the caller provides a ``copy.deepcopy`` but subclasses should
        still treat the input as owned.
        """
        ...

    def safe_apply(self, task: dict[str, Any]) -> tuple[dict[str, Any], MutationResult]:
        """Deep-copy then apply — safe entry point."""
        task_copy = copy.deepcopy(task)
        return self.apply(task_copy)

    # Helpers ---------------------------------------------------------------

    @staticmethod
    def _get_motion_commands(task: dict, cmd_type: str | None = None) -> list[dict]:
        """Return motion_sequence commands, optionally filtered by type."""
        cmds = task.get("motion_sequence", [])
        if cmd_type is not None:
            return [c for c in cmds if c["type"] == cmd_type]
        return cmds

    @staticmethod
    def _get_speed_limit(task: dict) -> float:
        """Return max_tcp_speed_mm_s from safety_requirements."""
        return task.get("safety_requirements", {}).get("max_tcp_speed_mm_s", 500.0)
