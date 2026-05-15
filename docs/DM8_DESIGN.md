# DM-8 API-Type Checker — Design Document

**Status:** Design only. Implementation deferred to post-NTNU backlog.
**Owner:** `enfield_watchdog_static`
**Paper anchor:** §VI.L C1 (URSim Live-Execution Pilot — Static-Watchdog Complementarity Gap)
**Session:** S30 Commit C
**Last updated:** 2026-05-13

---

## 1. Problem Statement

The S27 URSim live-execution pilot (`enfield_urscript_runtime`, paper §VI.L) surfaced
a complementarity gap (C1) between the static watchdog (DM-1..7 + SM-1..7) and
URControl's runtime type enforcement. The motivating failure case is reproduced below
in canonical form:

```urscript
def main():
    target_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    movej(get_inverse_kin(target_q), a=1.4, v=1.05)
end
```

URControl rejects this program at runtime with a type error: `get_inverse_kin` requires
its first positional argument to be a `Pose` (`p[x, y, z, rx, ry, rz]`), not a 6-element
joint vector. The current static watchdog accepts the program because:

1. DM-1..7 check joint ranges, speed/accel limits, frame consistency, E-Stop presence,
   tool parameters, zone boundaries, and IR-level invariants — none of which inspect
   call-site argument types against the URScript built-in signature catalogue.
2. SM-1..7 check security/CWE patterns (input validation, hardcoded values, prompt
   injection markers, etc.) but do not perform type inference.
3. The IR layer is intentionally type-agnostic to remain vendor-neutral.

The gap is small in scope (a single class of URControl-detectable errors) but
material for the paper: §VI.L identifies four C1-class instances across the
Qwen2.5-Coder-32B URSim pilot, and §VII.B currently mentions the gap without naming
a closure path. DM-8 closes that gap on paper as a *candidate detection rule*; the
implementation lands post-NTNU.

---

## 2. Scope (Minimum Viable DM-8)

In scope:
- Argument-type checking for a fixed catalogue of safety-relevant URScript built-ins
  (movement primitives, kinematic conversions, pose algebra)
- Type inference over **literal** arguments and **locally-assigned variables** with
  a literal RHS (single-pass scope analysis)
- Three concrete inferred types: `Pose`, `JointVector`, `Scalar` (float/int unified)
- One DM-8 violation per mismatched call site

Out of scope (deferred or never):
- Inter-procedural type propagation through user-defined functions
- Heuristic disambiguation of generic `list` arguments
- Geometric validity checks (a Pose with reachable but unsafe orientation typechecks)
- URScript's `socket_*` and `rpc_*` family — out of motion-safety scope
- Integration with PolyScope program tree (script-only)

This scope is deliberately narrow so the implementation post-NTNU is bounded at a
few-hundred-line patch.

---

## 3. URScript Type Distinctions

URScript is dynamically typed at the language level but URControl enforces a small
set of structural constraints at API boundaries. The relevant distinctions for DM-8
are:

| Inferred type | Surface syntax | Examples |
|---|---|---|
| `Pose` | `p[...]` prefix, length 6 | `p[0.4, -0.1, 0.3, 0, 3.14, 0]` |
| `JointVector` | `[...]` without `p` prefix, length 6 of numerics | `[0.0, -1.57, 0.0, -1.57, 0.0, 0.0]` |
| `Scalar` | numeric literal | `1.4`, `0.25`, `-1.57` |
| `Unknown` | identifier without traceable assignment, or anything else | function call return values, params |

`Unknown` arguments are **not** flagged. Flagging Unknown would generate false
positives on every URScript program that uses subroutine return values. Conservative
rule: DM-8 fires only when the inferred type is known *and* mismatches the expected
signature slot.

---

## 4. Signature Catalogue

Source: Universal Robots URScript Manual e-Series 5.12 (`script-manual-e-series-sw-5.12.pdf`).
Initial catalogue (subset; expand as the URSim model matrix grows):

| Built-in | Arg 0 | Arg 1 | Arg 2 | Arg 3 | Notes |
|---|---|---|---|---|---|
| `movej` | `Pose` \| `JointVector` | `Scalar` (a) | `Scalar` (v) | `Scalar` (t/r) | Polymorphic on arg 0 |
| `movel` | `Pose` | `Scalar` | `Scalar` | `Scalar` | Linear in tool space |
| `movep` | `Pose` | `Scalar` | `Scalar` | `Scalar` | Process move |
| `movec` | `Pose` (via) | `Pose` (to) | `Scalar` | `Scalar` | Circular |
| `get_inverse_kin` | `Pose` | `JointVector` (qnear, optional) | `Scalar` | `Scalar` | **C1 fault site** |
| `get_forward_kin` | `JointVector` | `Pose` (tcp, optional) | — | — | |
| `pose_trans` | `Pose` | `Pose` | — | — | |
| `pose_inv` | `Pose` | — | — | — | |
| `pose_add` | `Pose` | `Pose` | — | — | Deprecated, still seen |
| `pose_sub` | `Pose` | `Pose` | — | — | |
| `point_dist` | `Pose` | `Pose` | — | — | |
| `servoc` | `Pose` | `Scalar` | `Scalar` | `Scalar` | |
| `servoj` | `JointVector` | `Scalar` | `Scalar` | `Scalar` | |
| `speedj` | `JointVector` | `Scalar` | `Scalar` | — | Joint-space speed |
| `speedl` | `Pose` | `Scalar` | `Scalar` | — | Tool-space speed |

The catalogue lives as a structured Python dictionary in
`enfield_watchdog_static/urscript_signatures.py` and is unit-tested against a small
fixture set of known-good and known-bad programs.

---

## 5. Implementation Outline

```
enfield_watchdog_static/
    urscript_signatures.py     # NEW: catalogue + type enum
    urscript_type_inference.py # NEW: literal + local-scope inference
    rules/
        dm8_api_type_checker.py  # NEW: AST visitor + violation emission
    static_analyzer.py         # MODIFIED: register DM-8 in default rule set
```

Algorithm sketch:

```
visit_program(ast):
    env = {}  # variable name -> inferred type or Unknown
    for stmt in ast.statements:
        if stmt is Assignment:
            env[stmt.lhs] = infer_literal_type(stmt.rhs)  # may be Unknown
        elif stmt is Call:
            check_call(stmt, env)

check_call(call, env):
    sig = SIGNATURES.get(call.name)
    if sig is None:
        return  # not in catalogue, no DM-8 opinion
    for i, arg in enumerate(call.args[:len(sig)]):
        inferred = infer_arg_type(arg, env)
        expected = sig[i]
        if inferred is Unknown:
            continue
        if not types_compatible(inferred, expected):
            emit_violation(DM8, call, i, inferred, expected)

infer_literal_type(node):
    if node is ListLiteral with prefix 'p' and len==6: return Pose
    if node is ListLiteral with no prefix and len==6 of Scalars: return JointVector
    if node is NumericLiteral: return Scalar
    return Unknown

infer_arg_type(node, env):
    if node is Identifier and node.name in env: return env[node.name]
    return infer_literal_type(node)
```

`types_compatible` handles `movej`'s polymorphic arg 0 (accepts both `Pose` and
`JointVector`).

---

## 6. Violation Payload

Following the existing DM-1..7 / SM-1..7 emission shape:

```python
Violation(
    rule_id="DM-8",
    severity="HIGH",         # URControl will reject at load time
    cwe=None,                # safety, not security
    iso_clause=None,         # type-system, not standards-mapped
    line=call.line,
    column=call.col,
    message=f"{call.name} arg {i} expects {expected}, inferred {inferred}",
    snippet=source_for(call),
    suggested_fix=fix_for(call.name, i, expected),
)
```

`suggested_fix` returns a short remediation string per built-in, e.g. for the
canonical C1 example:

> Wrap the joint vector in `get_forward_kin(...)` to convert to a Pose before
> calling `get_inverse_kin`, or pass the target Pose directly: `p[x, y, z, rx, ry, rz]`.

---

## 7. False-Positive Posture

DM-8 is positioned as a **conservative** detector: when type cannot be inferred
unambiguously, the rule stays silent. Two design decisions follow:

1. **No Unknown-flagging.** Programs that pass arguments through subroutines or
   library functions will not trigger DM-8 at all. This is acceptable because
   DM-8's role is to add coverage on the LLM-generated programs that prompted the
   gap, which empirically pass joint/pose literals or scope-local variables.
2. **No inter-procedural propagation.** Adding it would dramatically expand the
   surface area for FP and is out of scope for a candidate rule. If post-NTNU
   experiments show DM-8's TPR is bottlenecked by missed propagation, that is a
   v2 expansion candidate.

Expected FP rate on the 788-test corpus + 15-task LLM-output sample: ~0%. The
fixture set must include this expectation as a regression guard.

---

## 8. Paper Integration

Three paper-side edits are anchored to this design doc. Each lands in S30 Commit D
alongside the Lane 3 and TCS-sensitivity edits.

**§VI.L (C1 entry):**
Add one sentence at the end of the C1 paragraph:
> A candidate detection rule, DM-8 (API-type checker), is sketched in
> `docs/DM8_DESIGN.md` as a closure path for this gap; implementation is deferred
> to future work.

**§VII.B (Discussion — Limitations and Future Work):**
Add a paragraph naming DM-8 explicitly:
> One concrete extension is the candidate detection rule DM-8, an API-type
> checker that performs argument-type inference against the URScript built-in
> signature catalogue. DM-8 would close the complementarity gap identified in
> §VI.L C1 without requiring runtime execution, complementing rather than
> replacing URControl's load-time checks. A design specification is provided in
> the replication package (`docs/DM8_DESIGN.md`).

**§VIII (Conclusion):**
Strengthen the existing forward reference to enumerate DM-8 alongside the other
post-paper extensions.

---

## 9. Implementation Backlog (Post-NTNU)

Ordered, each item ~1-3h:

1. Implement `urscript_signatures.py` with the initial catalogue (15 built-ins).
2. Implement `urscript_type_inference.py` with literal + local-scope inference.
3. Implement `rules/dm8_api_type_checker.py` as an AST visitor.
4. Register DM-8 in `static_analyzer.py` default rule set.
5. Add unit tests: 1 known-good and 1 known-bad fixture per built-in (30 cases).
6. Add integration test: re-run the four C1-class Lane 3 URSim programs and
   confirm DM-8 fires on each.
7. Expand signature catalogue to cover any built-ins surfaced by post-NTNU
   experiments.
8. Optional v2: inter-procedural propagation if FP/TPR analysis justifies it.

---

## 10. Open Questions

- **OSF preregistration treatment.** DM-8 is exploratory and post-hoc; the design
  doc lands as a forward reference, not a preregistered analysis. No amendment
  needed. If post-NTNU implementation produces results worth claiming, file an
  Amendment 4 at that time.
- **Severity assignment.** Marked HIGH because URControl will reject at load
  time, but DM-8 violations are detection-only and do not block code release.
  Re-evaluate if downstream gating ever depends on watchdog severity.
- **Catalogue maintenance.** Future URScript version bumps may add/remove
  built-ins. The catalogue is small enough (~15 entries) that manual update at
  each Universal Robots SW major release is acceptable.

---

## 11. Acceptance Criteria for This Design Doc

- [x] Problem statement names C1 and the canonical failure case.
- [x] Scope explicitly bounded; out-of-scope items enumerated.
- [x] Signature catalogue source named (URScript Manual e-Series 5.12).
- [x] Algorithm pseudocode + module layout specified.
- [x] FP posture documented with regression-guard expectation.
- [x] Paper-side edit anchors specified for §VI.L, §VII.B, §VIII.
- [x] Implementation backlog enumerated and time-boxed.

Implementation acceptance criteria belong to the post-NTNU patch that builds DM-8;
they are out of scope for this document.
