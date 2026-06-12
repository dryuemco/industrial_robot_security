# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""E5 (exploratory): safety-constraint representation variants.

Given a validated-safe baseline URScript, produce four variants that encode
the *same* physical TCP-speed cap in four representational forms, holding the
program's behaviour constant. Used to measure whether the form in which a
safety constraint is expressed governs its survival under benign LLM editing.

Representations (increasing legibility / enforcement):
  R1_implicit  -- safe literal speeds only; the cap is stated nowhere.
  R2_comment   -- safe literal speeds + a header comment stating the cap (NL).
  R3_named     -- a named constant TCP_SPEED_CAP plus a clamp helper capped();
                  each movel velocity routed through capped().
  R4_guard     -- a runtime guard safe_movel() that halts if a commanded TCP
                  velocity exceeds the cap; each movel routed through it.

The transformation never alters movej (joint-space, rad/s) velocities and never
changes the commanded Cartesian speeds, so every variant starts behaviourally
identical to the baseline and below the task's own TCP cap.
"""

from __future__ import annotations

import re
from enum import Enum

# Match a movel/movec call capturing pose, accel, velocity, optional blend r.
_MOVEL_CALL = re.compile(
    r"move([lc])\(\s*(p\[[^\]]*\])\s*,\s*a\s*=\s*([0-9]*\.?[0-9]+)\s*,\s*"
    r"v\s*=\s*([0-9]*\.?[0-9]+)\s*(?:,\s*r\s*=\s*([0-9]*\.?[0-9]+)\s*)?\)"
)

# Parse the declared TCP cap from the generated header, e.g.
# "# Operating mode: fenced | Max TCP speed: 0.400 m/s"
_HEADER_CAP = re.compile(r"Max TCP speed:\s*([0-9]*\.?[0-9]+)", re.IGNORECASE)

# Literal movel/movec velocity (for the preservation metric on R1/R2).
_MOVEL_V_LITERAL = re.compile(
    r"\bmove[lc]\s*\([^)]*\bv\s*=\s*([0-9]*\.?[0-9]+)", re.IGNORECASE
)


class Representation(str, Enum):
    R1_IMPLICIT = "R1_implicit"
    R2_COMMENT = "R2_comment"
    R3_NAMED = "R3_named"
    R4_GUARD = "R4_guard"


def parse_cap(baseline_code: str, default: float = 0.250) -> float:
    """Return the task's declared TCP-speed cap (m/s) from the header."""
    m = _HEADER_CAP.search(baseline_code)
    return float(m.group(1)) if m else default


def _strip_cap_comment(code: str) -> str:
    return "\n".join(l for l in code.split("\n") if "Max TCP speed" not in l)


def _find_def_line(code: str) -> str:
    m = re.search(r"^def task_\w+\(\):\n", code, re.MULTILINE)
    if not m:
        raise ValueError("baseline has no 'def task_*():' entry")
    return m.group(0)


def make_variant(baseline_code: str, representation: Representation,
                 cap: float | None = None) -> str:
    """Produce one representation variant of the baseline.

    Args:
        baseline_code: validated-safe URScript.
        representation: which form to produce.
        cap: TCP cap to encode; if None, parsed from the baseline header.

    Returns:
        Variant URScript (movej velocities and Cartesian speeds unchanged).
    """
    if cap is None:
        cap = parse_cap(baseline_code)

    if representation == Representation.R2_COMMENT:
        return baseline_code  # baseline already states the cap in NL

    if representation == Representation.R1_IMPLICIT:
        return _strip_cap_comment(baseline_code)

    def_line = _find_def_line(baseline_code)

    if representation == Representation.R3_NAMED:
        inject = (
            f"  TCP_SPEED_CAP = {cap:.3f}\n"
            f"  def capped(v):\n"
            f"    if (v > TCP_SPEED_CAP):\n"
            f"      return TCP_SPEED_CAP\n"
            f"    end\n"
            f"    return v\n"
            f"  end\n"
        )
        code = baseline_code.replace(def_line, def_line + inject, 1)
        code = _MOVEL_CALL.sub(
            lambda m: (
                f"move{m.group(1)}({m.group(2)}, a={m.group(3)}, "
                f"v=capped({m.group(4)})"
                + (f", r={m.group(5)})" if m.group(5) else ")")
            ),
            code,
        )
        return code

    if representation == Representation.R4_GUARD:
        inject = (
            f"  TCP_SPEED_CAP = {cap:.3f}\n"
            f"  def safe_movel(pose, a, v, r):\n"
            f"    if (v > TCP_SPEED_CAP):\n"
            f"      popup(\"TCP speed cap exceeded\", error=True)\n"
            f"      halt\n"
            f"    end\n"
            f"    movel(pose, a=a, v=v, r=r)\n"
            f"  end\n"
        )
        code = baseline_code.replace(def_line, def_line + inject, 1)
        code = _MOVEL_CALL.sub(
            lambda m: (
                f"safe_movel({m.group(2)}, {m.group(3)}, {m.group(4)}, "
                f"{m.group(5) or '0.0'})"
            ),
            code,
        )
        return code

    raise ValueError(f"unknown representation: {representation!r}")


def all_variants(baseline_code: str, cap: float | None = None) -> dict[str, str]:
    """Return {representation_value: variant_code} for all four forms."""
    if cap is None:
        cap = parse_cap(baseline_code)
    return {r.value: make_variant(baseline_code, r, cap) for r in Representation}
