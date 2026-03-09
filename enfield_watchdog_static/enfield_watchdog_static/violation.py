# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Violation data model for static watchdog reports."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Violation:
    """A single safety violation detected by the static watchdog."""

    attack_type: str          # "A1" .. "A8"
    iso_clause: str           # e.g. "5.6"
    detection_mechanism: str  # "DM-1" .. "DM-7"
    description: str          # human-readable
    severity: float           # attack-type-specific metric
    location: str = ""        # e.g. "motion_sequence[2].speed_mm_s"
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "attack_type": self.attack_type,
            "iso_clause": self.iso_clause,
            "detection_mechanism": self.detection_mechanism,
            "description": self.description,
            "severity": self.severity,
            "location": self.location,
            "metadata": self.metadata,
        }


@dataclass
class WatchdogReport:
    """Aggregated report for one task file."""

    task_id: str
    source_file: str = ""
    violations: list[Violation] = field(default_factory=list)
    checks_run: int = 0

    @property
    def safe(self) -> bool:
        return len(self.violations) == 0

    @property
    def violation_count(self) -> int:
        return len(self.violations)

    def violations_by_attack(self) -> dict[str, list[Violation]]:
        result: dict[str, list[Violation]] = {}
        for v in self.violations:
            result.setdefault(v.attack_type, []).append(v)
        return result

    def has_attack(self, attack_id: str) -> bool:
        return any(v.attack_type == attack_id for v in self.violations)

    def max_severity(self) -> float:
        if not self.violations:
            return 0.0
        return max(v.severity for v in self.violations)

    def summary(self) -> str:
        if self.safe:
            return f"PASS — {self.task_id}: {self.checks_run} checks, 0 violations."
        lines = [
            f"FAIL — {self.task_id}: {self.violation_count} violation(s) "
            f"from {self.checks_run} checks:"
        ]
        for v in self.violations:
            lines.append(
                f"  [{v.attack_type}] {v.description} "
                f"(severity={v.severity:.2f}, ISO {v.iso_clause})"
            )
        return "\n".join(lines)

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "source_file": self.source_file,
            "safe": self.safe,
            "violation_count": self.violation_count,
            "checks_run": self.checks_run,
            "max_severity": self.max_severity(),
            "violations": [v.to_dict() for v in self.violations],
        }
