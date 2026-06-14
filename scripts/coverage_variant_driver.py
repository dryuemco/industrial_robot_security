# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Coverage-variant driver for the H2 verifier-induced error-migration ablation.

The headline experiment of the SE (Track-2) paper manipulates *what the verifier
exposes to the repair loop* while a separate full watchdog *evaluates against the
complete rule set*. This realises the design invariant:

    evaluation oracle  >=  feedback oracle

i.e. a variant may hide a rule class from the loop's feedback, but every emitted
program is still scored against all classes so residual/migrated violations are
observable.

Variant families
----------------
  full     : all pinned classes exposed (control / boundary = empty).
  loo      : leave-one-out -- each variant hides exactly one class.
  nested   : growing exposed prefixes  Phi^(1) c Phi^(2) c ... c Phi^(k-1)
             (proper subsets only; the full prefix is the 'full' variant).

No detector is modified or added. Variants are pure configurations of the
existing StaticWatchdog via its enabled_attacks / enabled_security knobs.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "enfield_watchdog_static"))

from enfield_watchdog_static.watchdog import StaticWatchdog
from enfield_watchdog_static.rules import ALL_RULES, ALL_SECURITY_RULES

SAFETY_IDS = set(ALL_RULES)            # A1..A8  (IR-level)
SECURITY_IDS = set(ALL_SECURITY_RULES)  # SM-1..SM-7 (code-level)
KNOWN_IDS = SAFETY_IDS | SECURITY_IDS


@dataclass(frozen=True)
class CoverageVariant:
    """One verifier coverage configuration over the pinned ablation set."""

    name: str
    exposed: tuple[str, ...]   # classes the verifier FEEDS BACK to the loop
    hidden: tuple[str, ...]    # ablation_set - exposed (evaluated, never fed back)

    def split(self) -> tuple[list[str], list[str]]:
        """Split exposed IDs into (safety_ids, security_ids) for StaticWatchdog."""
        a = sorted(i for i in self.exposed if i in SAFETY_IDS)
        sm = sorted(i for i in self.exposed if i in SECURITY_IDS)
        return a, sm

    def feedback_watchdog(self) -> StaticWatchdog:
        """Watchdog that EXPOSES only this variant's classes (drives the loop).

        IMPORTANT: if a family is *empty* for one rule kind, we pass an explicit
        empty list (not None). StaticWatchdog treats None as 'enable all', so
        ``enabled_attacks=[]`` is required to mean 'expose no safety rule'.
        """
        a, sm = self.split()
        return StaticWatchdog(enabled_attacks=a, enabled_security=sm)


def full_watchdog() -> StaticWatchdog:
    """Evaluation oracle: the complete rule set (always measures everything)."""
    return StaticWatchdog()


def _validate(ablation_set: list[str]) -> list[str]:
    unknown = [c for c in ablation_set if c not in KNOWN_IDS]
    if unknown:
        raise ValueError(
            f"Unknown rule-class IDs {unknown}. Known: {sorted(KNOWN_IDS)}"
        )
    # dedupe, preserve order (order is load-bearing for the nested family)
    seen: set[str] = set()
    ordered: list[str] = []
    for c in ablation_set:
        if c not in seen:
            seen.add(c)
            ordered.append(c)
    if len(ordered) < 2:
        raise ValueError("Need >= 2 classes in the ablation set to vary coverage.")
    return ordered


def build_variants(
    ablation_set: list[str],
    families: Iterable[str] = ("full", "loo", "nested"),
) -> list[CoverageVariant]:
    """Build coverage variants over the pinned ablation (Tier-S) class list.

    Args:
        ablation_set: ordered list of provably-sound rule-class IDs to ablate
            over, e.g. ["A1", "A2", "A5"] (speed / zone / e-stop-control-flow).
            Order matters for the nested family (defines the prefix growth).
        families: which variant families to emit.

    Returns:
        Deterministic list of CoverageVariant (full, then LOO, then nested).

    Note:
        SM-* classes are Tier-H (heuristic, not provably sound) and should not
        normally appear in ``ablation_set`` for headline H2; keep them exposed
        as breadth context, or move them into ``ablation_set`` only for the
        H5 sound-vs-unsound contrast.
    """
    ablation_set = _validate(ablation_set)
    fam = set(families)
    full_set = tuple(ablation_set)
    out: list[CoverageVariant] = []

    if "full" in fam:
        out.append(CoverageVariant("full", full_set, ()))

    if "loo" in fam:
        for hide in ablation_set:
            exposed = tuple(c for c in ablation_set if c != hide)
            out.append(CoverageVariant(f"loo_hide_{hide}", exposed, (hide,)))

    if "nested" in fam:
        # proper prefixes only: k = 1 .. len-1 (k = len IS the 'full' variant)
        for k in range(1, len(ablation_set)):
            exposed = tuple(ablation_set[:k])
            hidden = tuple(ablation_set[k:])
            out.append(CoverageVariant(f"nested_k{k}", exposed, hidden))

    return out


def describe(variants: list[CoverageVariant]) -> str:
    lines = [f"{len(variants)} coverage variant(s):"]
    for v in variants:
        lines.append(
            f"  {v.name:16s} expose={list(v.exposed)!s:40s} hide={list(v.hidden)}"
        )
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--ablation-set", nargs="+", required=True,
        help="Pinned Tier-S rule-class IDs, e.g. A1 A2 A5",
    )
    p.add_argument(
        "--families", nargs="+", default=["full", "loo", "nested"],
        choices=["full", "loo", "nested"],
    )
    args = p.parse_args()
    vs = build_variants(args.ablation_set, args.families)
    print(describe(vs))
