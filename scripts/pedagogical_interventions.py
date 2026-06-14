"""Pedagogical feedback interventions for the FACT repair loop (additive).

Pluggable strategies that decide what feedback (if any) the LLM receives
between repair attempts in run_coverage_ablation.py, whether to override the
exposed violation set, and whether to request an extra self-diagnosis model
call before the repair attempt.

Each strategy is a named pedagogical mechanism with a falsifiable prediction:

  naive_minimal       owned baseline; names violated classes, says "fix".
  naive_rich          owned baseline; classes + ISO/CWE ref + safe-range hint.
  worked_example      Sweller / L-ICL; inject a canonical SAFE fragment per
                      violated class (the safe *form*, not a fix of the
                      model's specific line).
  socratic            elenchus; do NOT state the offending line. Ask the model
                      to inspect its own code against the requirement, locate
                      and explain the violation, then (separate turn) repair.
  productive_failure  Kapur; first N retries get no feedback (free failure),
                      then structured feedback.
  scaffold_fade       Vygotsky ZPD; NOT IMPLEMENTED (stub).
  mastery_gate        Bloom mastery; NOT IMPLEMENTED (stub).

This module contains intervention logic ONLY. It never calls the LLM, the
watchdog, or the lifter; the orchestrator owns those. See ORCHESTRATOR
CONTRACT at the bottom of this file.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, FrozenSet, List, Optional


# --------------------------------------------------------------------------
# Safety-class metadata for the active ablation set.
# Used by naive_rich, worked_example and socratic feedback.
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class ClassMeta:
    label: str
    ref: str
    safe_hint: str


CLASS_META = {
    "A1": ClassMeta(
        label="unsafe Cartesian/joint speed",
        ref="ISO 10218-1:2025 Cl. 5.5.3 (reduced/collaborative speed)",
        safe_hint=("collaborative-mode tool speed must not exceed 0.25 m/s; "
                   "set v<=0.25 in the movel/movec/movej speed argument"),
    ),
    "A5": ClassMeta(
        label="missing emergency/protective-stop provision",
        ref="ISO 10218-1:2025 Cl. 5.4.2 (emergency stop)",
        safe_hint=("the program must provide a protective/emergency-stop path "
                   "before or around motion"),
    ),
    "SM-2": ClassMeta(
        label="unchecked return value",
        ref="CWE-252 (Unchecked Return Value)",
        safe_hint=("capture and check the return value of safety-relevant "
                   "calls before proceeding"),
    ),
    "SM-4": ClassMeta(
        label="improper check for unusual/exceptional condition",
        ref="CWE-754 (Improper Check for Unusual or Exceptional Conditions)",
        safe_hint="explicitly guard exceptional conditions before motion",
    ),
    "SM-5": ClassMeta(
        label="hard-coded safety-relevant value",
        ref="CWE-798 (Use of Hard-coded Values)",
        safe_hint=("do not hard-code safety-relevant constants; reference "
                   "declared, validated parameters"),
    ),
}


# --------------------------------------------------------------------------
# Worked-example exemplars. Each fragment MUST score zero violations for its
# target class under the real urscript_safety_lift + SM rules, otherwise the
# worked_example arm is confounded. Entries beginning with the PLACEHOLDER
# sentinel are NOT injected into prompts (WorkedExample degrades them to the
# class safe_hint) and are reported by worked_example_ready() as not ready.
# --------------------------------------------------------------------------
_PLACEHOLDER = "PLACEHOLDER"

WORKED_EXAMPLE_FRAGMENTS = {
    "A1": ("# safe collaborative linear move, tool speed <= 0.25 m/s\n"
           "movel(p[0.40, -0.20, 0.30, 0.0, 3.14, 0.0], a=1.2, v=0.25, r=0.0)"),
    "A5": _PLACEHOLDER + ": supply an A5-compliant protective-stop fragment",
    "SM-2": _PLACEHOLDER + ": supply an SM-2-compliant checked-return fragment",
    "SM-4": _PLACEHOLDER + ": supply an SM-4-compliant guarded-condition fragment",
    "SM-5": _PLACEHOLDER + ": supply an SM-5-compliant parameterized-value fragment",
}


def worked_example_ready() -> FrozenSet[str]:
    """Classes that currently have a real (non-placeholder) worked example."""
    return frozenset(
        c for c, f in WORKED_EXAMPLE_FRAGMENTS.items()
        if not f.startswith(_PLACEHOLDER)
    )


# --------------------------------------------------------------------------
# Action returned by every intervention step.
# --------------------------------------------------------------------------
@dataclass
class InterventionAction:
    feedback_text: str = ""
    exposed_override: Optional[FrozenSet[str]] = None
    needs_diagnosis_call: bool = False
    diagnosis_prompt: Optional[str] = None
    skip_feedback: bool = False
    _diagnosis_wrapper: Optional[Callable[[str], str]] = field(
        default=None, repr=False, compare=False
    )

    def render(self, diagnosis_response: Optional[str] = None) -> str:
        """Final feedback text. Folds a self-diagnosis response when required."""
        if self._diagnosis_wrapper is not None:
            if diagnosis_response is None:
                raise ValueError(
                    "diagnosis response required before rendering this action"
                )
            return self._diagnosis_wrapper(diagnosis_response)
        return self.feedback_text


# --------------------------------------------------------------------------
# Strategy interface + implementations.
# --------------------------------------------------------------------------
class Intervention(ABC):
    name: str = "base"

    @abstractmethod
    def step(
        self,
        iteration: int,
        history: list,
        exposed_violations: FrozenSet[str],
        base_task: str,
        code: str,
    ) -> InterventionAction:
        ...


def _sorted_classes(exposed_violations) -> List[str]:
    return sorted(exposed_violations)


class NaiveMinimal(Intervention):
    name = "naive_minimal"

    def step(self, iteration, history, exposed_violations, base_task, code):
        classes = _sorted_classes(exposed_violations)
        if not classes:
            return InterventionAction()
        text = ("Your code still violates: " + ", ".join(classes) + ". "
                "Fix all listed violations and return the corrected URScript only.")
        return InterventionAction(feedback_text=text)


class NaiveRich(Intervention):
    name = "naive_rich"

    def step(self, iteration, history, exposed_violations, base_task, code):
        classes = _sorted_classes(exposed_violations)
        if not classes:
            return InterventionAction()
        lines = ["Your code still violates the following safety requirements:"]
        for c in classes:
            m = CLASS_META.get(c)
            if m:
                lines.append("- " + c + " (" + m.label + "; " + m.ref + "): "
                             + m.safe_hint)
            else:
                lines.append("- " + c)
        lines.append("Correct every listed item and return the corrected "
                     "URScript only.")
        return InterventionAction(feedback_text="\n".join(lines))


class WorkedExample(Intervention):
    name = "worked_example"

    def step(self, iteration, history, exposed_violations, base_task, code):
        classes = _sorted_classes(exposed_violations)
        if not classes:
            return InterventionAction()
        lines = [
            "Your code violates the requirements below. For each one, study the "
            "worked example of the correct safe form and apply the same pattern "
            "to your code (adapt to your own targets; do not copy verbatim):"
        ]
        for c in classes:
            m = CLASS_META.get(c)
            ref = (" [" + m.ref + "]") if m else ""
            frag = WORKED_EXAMPLE_FRAGMENTS.get(c, "")
            if frag and not frag.startswith(_PLACEHOLDER):
                lines.append("\n# " + c + ref
                             + " - worked example of the safe form:\n" + frag)
            else:
                hint = m.safe_hint if m else "(no exemplar available)"
                lines.append("\n# " + c + ref + " - required safe form: " + hint)
        lines.append("\nReturn the corrected URScript only.")
        return InterventionAction(feedback_text="\n".join(lines))


class Socratic(Intervention):
    name = "socratic"

    def step(self, iteration, history, exposed_violations, base_task, code):
        classes = _sorted_classes(exposed_violations)
        if not classes:
            return InterventionAction()
        reqs = []
        for c in classes:
            m = CLASS_META.get(c)
            reqs.append(("- " + c + ": " + m.label + " (" + m.ref + ")")
                        if m else ("- " + c))
        diag_prompt = (
            "Review the URScript you just produced against these safety "
            "requirements:\n" + "\n".join(reqs) + "\n\n"
            "For EACH requirement, quote the exact line(s) in your code that "
            "violate it (or write 'compliant' if none), and explain in one "
            "sentence WHY that line violates the requirement. Do not rewrite "
            "the code yet. Output only your analysis."
        )

        def wrap(diagnosis_response: str) -> str:
            return (
                "Here is your own analysis of your code:\n\n"
                + diagnosis_response.strip() + "\n\n"
                "Now apply the corrections you identified. Return the corrected "
                "URScript only."
            )

        return InterventionAction(
            needs_diagnosis_call=True,
            diagnosis_prompt=diag_prompt,
            _diagnosis_wrapper=wrap,
        )


class ProductiveFailure(Intervention):
    name = "productive_failure"
    FREE_ITERS = 2  # first N repair attempts receive no feedback (free failure)

    def step(self, iteration, history, exposed_violations, base_task, code):
        if iteration < self.FREE_ITERS:
            # Bare retry: orchestrator re-prompts with the base task and NO
            # violation disclosure (see skip_feedback in the contract).
            return InterventionAction(skip_feedback=True)
        return NaiveRich().step(iteration, history, exposed_violations,
                                base_task, code)


class _NotImplementedIntervention(Intervention):
    def step(self, iteration, history, exposed_violations, base_task, code):
        raise NotImplementedError(
            "intervention '" + self.name + "' is planned but not implemented "
            "for the pilot. Implement after pilot greenlight, once the run_cell "
            "loop semantics (per-iteration exposure override and guidance "
            "schedule) are confirmed against the actual orchestrator."
        )


class ScaffoldFade(_NotImplementedIntervention):
    name = "scaffold_fade"


class MasteryGate(_NotImplementedIntervention):
    name = "mastery_gate"


# --------------------------------------------------------------------------
# Registry + factory.
# --------------------------------------------------------------------------
_REGISTRY = {
    cls.name: cls
    for cls in (
        NaiveMinimal, NaiveRich, WorkedExample, Socratic,
        ProductiveFailure, ScaffoldFade, MasteryGate,
    )
}

INTERVENTION_NAMES = sorted(_REGISTRY)


def build_intervention(name: str) -> Intervention:
    try:
        return _REGISTRY[name]()
    except KeyError:
        raise ValueError(
            "unknown intervention '" + str(name) + "'; choices: "
            + ", ".join(INTERVENTION_NAMES)
        )


# --------------------------------------------------------------------------
# ORCHESTRATOR CONTRACT (run_coverage_ablation.py)
# --------------------------------------------------------------------------
# 1) Argparse:
#       from pedagogical_interventions import build_intervention, INTERVENTION_NAMES
#       parser.add_argument("--intervention", choices=INTERVENTION_NAMES,
#                           default="naive_minimal")
#       intervention = build_intervention(args.intervention)
#
# 2) Inside run_cell, the per-retry block -- AFTER scoring the current `code`
#    and obtaining `exposed_violations` for that attempt, BEFORE generating
#    the next attempt -- becomes:
#
#       action = intervention.step(iteration, history, exposed_violations,
#                                  base_task, code)
#       if action.skip_feedback:
#           feedback = None                       # bare retry, no disclosure
#           exposed_for_next = exposed_violations
#       else:
#           diagnosis = None
#           if action.needs_diagnosis_call:
#               diagnosis = llm.generate(action.diagnosis_prompt, **gen_kwargs)
#               record.setdefault("diagnosis_calls", []).append({
#                   "iteration": iteration,
#                   "prompt": action.diagnosis_prompt,
#                   "response": diagnosis,
#               })
#           feedback = action.render(diagnosis)
#           exposed_for_next = action.exposed_override or exposed_violations
#       # generate next attempt with `feedback` (or base task only if None),
#       # then score it against `exposed_for_next`.
#
# 3) Record schema: persist "intervention" (name) per cell, "diagnosis_calls"
#    (list, when present) and a per-iteration "skip_feedback" flag, so the
#    analyzer can compare arms and count the extra socratic calls.
#
# 4) skip_feedback means skip the FEEDBACK CONTENT, not the iteration: the
#    model is still re-prompted (base task only) and its output scored.
