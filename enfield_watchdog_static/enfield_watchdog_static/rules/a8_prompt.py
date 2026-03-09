# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-7: A8 Prompt Injection — prompt_security configuration integrity check."""

from __future__ import annotations
from typing import Any
from enfield_watchdog_static.violation import Violation

# Thresholds indicating a weakened / bypassed prompt_security config
_PERPLEXITY_SUSPECT = 9999.0
_SAFETY_RATIO_SUSPECT = 0.01


def check_a8_prompt(task: dict[str, Any]) -> list[Violation]:
    """Check prompt_security for signs of bypass or weakening.

    Detection:
    1. perplexity_threshold set absurdly high (>= 9999)
    2. blocked_patterns emptied
    3. encoding_scan disabled
    4. min_safety_token_ratio near zero
    """
    violations: list[Violation] = []

    ps = task.get("prompt_security")
    if not ps:
        return violations

    # Check 1: perplexity threshold suspiciously high
    ppl = ps.get("perplexity_threshold", 50.0)
    if ppl >= _PERPLEXITY_SUSPECT:
        violations.append(Violation(
            attack_type="A8",
            iso_clause="5.3",
            detection_mechanism="DM-7",
            description=(
                f"perplexity_threshold={ppl:.0f} — suspiciously high, "
                f"adversarial suffix detection effectively disabled"
            ),
            severity=0.9,
            location="prompt_security.perplexity_threshold",
            metadata={"perplexity_threshold": ppl},
        ))

    # Check 2: blocked_patterns emptied
    patterns = ps.get("blocked_patterns", [])
    if isinstance(patterns, list) and len(patterns) == 0:
        violations.append(Violation(
            attack_type="A8",
            iso_clause="5.3",
            detection_mechanism="DM-7",
            description="blocked_patterns is empty — jailbreak pattern detection disabled",
            severity=0.7,
            location="prompt_security.blocked_patterns",
            metadata={"blocked_patterns_count": 0},
        ))

    # Check 3: encoding_scan disabled
    if ps.get("encoding_scan") is False:
        violations.append(Violation(
            attack_type="A8",
            iso_clause="5.3",
            detection_mechanism="DM-7",
            description="encoding_scan=false — Base64/ROT13 payload detection disabled",
            severity=0.5,
            location="prompt_security.encoding_scan",
        ))

    # Check 4: safety token ratio near zero
    ratio = ps.get("min_safety_token_ratio", 0.05)
    if ratio < _SAFETY_RATIO_SUSPECT:
        violations.append(Violation(
            attack_type="A8",
            iso_clause="5.3",
            detection_mechanism="DM-7",
            description=(
                f"min_safety_token_ratio={ratio} — context overflow "
                f"protection effectively disabled"
            ),
            severity=0.6,
            location="prompt_security.min_safety_token_ratio",
            metadata={"min_safety_token_ratio": ratio},
        ))

    return violations
