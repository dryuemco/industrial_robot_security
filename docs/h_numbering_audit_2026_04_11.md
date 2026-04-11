# H-Numbering Drift Audit

**Date:** 2026-04-11
**Author:** Yunus Emre Cogurcu
**Trigger:** Week 10 TODO #12 (filed in commit 0ea0a27)
**Outcome:** Direction A — paper aligned to code + prereg
**Status:** Audit complete; remediation in progress

---

## Why this audit exists

The previous session filed Week 10 TODO #12 as a "bounded mechanical
cleanup" of H-numbering drift between three sources (paper, prereg,
code), with the assumption that the prereg already used paper-level
semantics and the code needed renaming to match.

On inspection that framing was inverted. The prereg and the code are
internally consistent. The paper is the source of the drift, and the
paper is also internally inconsistent: section V.E uses one set of
H4-H6 definitions, section VI.F-H uses a second (which happens to
match the prereg), section VII.B uses a third, and Appendix A uses a
fourth that mixes pre-Amendment and post-Amendment numbering and
lists a hypothesis ("H4 = Cochran's Q") that appears in no version
of the preregistration at all.

If #12 had been executed as filed — renaming `run_h4`, `run_h5`,
`run_h6` in `scripts/mcnemar_analysis.py` to match paper section
V.E — the result would have been a code/prereg divergence: the
analysis script would have computed values that did not match the
preregistered analysis plan. That is the kind of bug that survives
CI (the tests would still pass, because the tests check
mathematical correctness of each function, not which paper-level
hypothesis the function corresponds to) and would only surface in
review.

This document records the actual mapping, the chosen alignment
direction, and the work-breakdown for remediation. It is a one-time
audit artifact, not a permanent reference.

---

## Truth table

The H-numbering across all sources at the time of this audit
(HEAD `7c76025` on main):

| Source | H1 | H2 | H3 | H4 | H5 | H6 |
|---|---|---|---|---|---|---|
| **Prereg (original section 1.3)** | Watchdog >=40% reduction | Watchdog >=80% detection | FPR <=5% | — | — | — |
| **Prereg (Amendment 1, approved 2026-04-07)** | AST watchdog >=90% detection, <=5% FPR | — | — | Baseline rate >=30% (one-sided binomial; Wilson CI) | Adversarial >=50pp absolute (McNemar; Holm-Bonferroni) | Watchdog-in-loop >=40% relative (McNemar; Newcombe CI) |
| **Code: `scripts/mcnemar_analysis.py`** | — | — | — | `run_h4`: baseline rate >=`H4_THRESHOLD` (0.30), one-sided binomial | `run_h5`: `delta = rate_b - rate_a`, McNemar+Holm | `run_h6`: watchdog vs baseline+safety, `relative_change <= -H6_THRESHOLD`, McNemar+Holm |
| **Paper section V.E (line 287-289)** | — | — | — | Watchdog-in-loop >=30% relative (McNemar) | Adversarial >=50% **relative** (McNemar) | Cross-model heterogeneity (Cochran's Q) |
| **Paper section VI.F-H (line 375, 396, 412)** | — | — | — | Baseline rate >=30% (matches prereg) | Adversarial uplift A8.k | Watchdog WRR >=40% |
| **Paper section VII.B (line 504, 506)** | — | — | — | Watchdog-in-loop reduction | Adversarial uplift | Cross-model heterogeneity ("Cochran future work, not yet implemented" — stale claim) |
| **Paper Appendix A (line 547-550)** | LLM code >=30% violation | Adversarial >=50% | Watchdog >=40% | Cross-model Cochran's Q **(unregistered)** | — | — |
| **README + Appendix B reproducibility** | — | — | — | — | — | "624 tests" (stale, actual 702) |

### Key observations

1. **Prereg and code are mutually consistent.** `run_h4` / `run_h5` /
   `run_h6` compute exactly what Amendment 1 specifies. The code
   function names are correct.
2. **Paper section VI.F-H is also prereg-consistent.** The
   placeholder results section already uses Amendment 1 semantics.
   This is load-bearing: section VI does not need semantic
   rewriting, only header wording cleanup once section V.E is
   realigned.
3. **Paper section V.E has the wrong H-number assignments.** What
   section V.E labels H4 is what the prereg labels H6 (watchdog).
   What section V.E labels H6 (Cochran's Q) is not in the prereg
   at all.
4. **Paper Appendix A is the worst-affected section.** It mixes
   pre-Amendment numbering (H1-H3 with post-Amendment meanings) and
   lists "H4 = Cochran's Q" as preregistered, which is false in
   both prereg versions. A reviewer reading Appendix A first would
   conclude the paper claims preregistered status for an
   unregistered hypothesis.
5. **Paper section VII.B line 506 has a stale claim.** It states
   Cochran's Q "is listed as future work and is not yet implemented
   in `scripts/mcnemar_analysis.py`". Cochran's Q was implemented
   in commits `e4573ad` and `e918fb7` (Week 10 TODO #7) and is
   fully tested (`TestCrossModelCochranQ`, 13 tests).
6. **Paper section V.E H5 wording mismatch.** Section V.E says
   ">=50% relative" uplift; prereg and code use ">=50pp absolute".
   At low baseline rates these are very different thresholds.
7. **Test count is stale across paper Appendix B and section
   VII.B line 564.** "624 tests" appears in both; actual count at
   this audit is 702. Cascade per the project convention: README,
   CI badge, `open_science_release.md`, and `OSF_PREREGISTRATION.md`
   should be checked for the same number.

---

## Direction A: paper aligned to code + prereg

After reviewing the three remediation options (A: align paper to
prereg; B: file Amendment 2; C: hybrid keep Cochran exploratory),
**Direction A** is selected. Rationale:

- Lowest risk to the IEEE RA-L submission timeline (no OSF
  amendment cycle, no second Georgios review on the prereg).
- Prereg is the legally-binding artifact for the confirmatory
  claims. Aligning the paper to the prereg is always safer than
  aligning the prereg to the paper.
- Code does not need to change at the function level. Only the
  markdown report headers in `generate_markdown_report` need
  relabelling, because those headers are paper-facing strings,
  not function names.
- Cochran's Q stays in the paper, but as **exploratory** rather
  than confirmatory. It is reported descriptively alongside the
  H4-H6 confirmatory family, with explicit "not part of the
  preregistered confirmatory family" language.

### What changes (paper-side)

- **Section V.E:** Renumber so paper H4 = baseline rate (prereg
  H4), paper H5 = adversarial absolute pp uplift (prereg H5),
  paper H6 = watchdog relative reduction (prereg H6). Add an
  exploratory paragraph for Cochran's Q. Fix the H5 "relative"
  to "absolute pp" wording. Update the statistical-methods
  description for H4 from "McNemar paired" to "one-sided exact
  binomial; Wilson CI".
- **Section VI.F-H:** Already prereg-consistent in semantics;
  verify header text and decision-rule wording stay aligned with
  the new section V.E. Likely no semantic edit, only consistency
  check.
- **Section VII.B line 504:** Update H4/H5/H6 labels to match
  new section V.E.
- **Section VII.B line 506:** Remove the "Cochran's Q not yet
  implemented" claim. Replace with a statement that Cochran's Q
  is implemented and reported as exploratory (referencing section
  V.E's exploratory paragraph and section VI's results table).
- **Appendix A:** Rewrite. Show original prereg (H1-H3) and
  Amendment 1 (H4-H6) as separate clearly-labelled blocks. Remove
  the false "H4 = Cochran's Q preregistered" line. Add an
  explicit "Cochran's Q is reported as exploratory and is not
  part of the preregistered confirmatory family" sentence.
- **Section IV.C and other paper-wide H# references:** Grep-
  discover and realign. The paper-wide grep run during this audit
  is the authoritative scope.
- **Line ~291 "Zero refusals across all models and conditions":**
  Tag with a visible FIXME comment until the first confirmatory
  E1 run with the post-`7c76025` refusal classifier verifies it.
- **Test count cascade:** 624 to 702 across paper Appendix B,
  paper section VII.B line 564, README, CI badge,
  `open_science_release.md`, `OSF_PREREGISTRATION.md`. Single
  commit, no semantics.

### What changes (code-side)

- **`generate_markdown_report` headers in
  `scripts/mcnemar_analysis.py`:** The "## H6 Omnibus: Cross-Model
  Cochran's Q" header (introduced in `e918fb7`) and any other
  H#-labelled section header relabelled to match the new paper
  semantics. **Function names stay as `run_h4`, `run_h5`,
  `run_h6`** because they match the prereg.
- **Add a docstring at the top of `mcnemar_analysis.py`:**
  Document the convention "function names follow the
  preregistration H-numbering; markdown report headers follow the
  paper H-numbering; the two numberings happen to coincide under
  Direction A but the distinction is preserved for future-
  proofing."
- **Add a mapping invariant test:** A small test in
  `tests/test_mcnemar_analysis.py` that documents what each
  `run_h*` function computes, so a future rename cannot silently
  drift the semantics again.

### What does not change

- Code function names (`run_h4`, `run_h5`, `run_h6`) — they match
  prereg, no rename.
- Code statistical logic — already correct per Amendment 1.
- Prereg — neither original section 1.3 nor Amendment 1 are
  touched. No Amendment 2.
- The OSF deposit — no amendment, no clarification note for now.

---

## Out of scope for this audit

The following items are real but **deliberately excluded** from
the Direction A remediation, to keep #12 bounded:

1. **A6.* to A8.* code-level rename** in
   `llm_experiment_runner.py`, `prompt_builder`, mutation modules.
   Filed in the deferred follow-ups list. The paper already uses
   A8.* with a footnote explaining the legacy A6.* in code (paper
   line 112). This is purely editorial and does not affect any
   hypothesis.
2. **THREAT_MODEL.md A8.1-A8.6 vs A8.1-A8.8 range** at line ~91.
   Filed in deferred follow-ups.
3. **`attack_definitions_A5_A8.md` line ~611 prose** about ISO
   not addressing LLM code generation. Partially outdated by
   ISO 10218-1:2025 section 5.1.16. Filed in deferred follow-ups.
4. **Whether H1-H3 (original prereg) should still be reported in
   the paper.** The original prereg was about AST watchdog
   detection rate against an attack catalogue. The current paper
   focus is LLM violation rates. If H1-H3 are still reported,
   they stay confirmatory; if not, they become "deferred to
   follow-up work" in Appendix A. **This is a Georgios-boundary
   decision,** not a unilateral one. Flagged for the next
   supervisor sync; Direction A remediation can proceed without
   resolving this because Appendix A can be written conditionally
   on either answer.
5. **Whether to file a post-approval clarification note on OSF**
   describing the implementation details of the refusal
   classifier (commits `ad88356` + `7c76025`) and Cochran's Q's
   exploratory status. Optional, can be done after first
   confirmatory E1 run.

---

## Remediation phase plan

Phases referenced in the Week 10 session sequence after #12 is
opened:

| Phase | What | Risk | Done in commit |
|---|---|---|---|
| 0a | Fix `test_extracts_refusal_reason` regression + tie-breaker | low | `7c76025` |
| 1 | This audit doc | zero | (this commit) |
| 3 | Paper section V.E rewrite (H4/H5/H6 realign + H5 absolute + Cochran exploratory) | medium | pending |
| 3b | Paper section VI.F-H consistency check (semantic edit unlikely) | low | pending |
| 3c | Paper section VII.B line 504 + 506 fix | low | pending |
| 3d | Paper Appendix A rewrite | medium | pending |
| 3e | Test count cascade 624 to 702 | low | pending |
| 5 | `generate_markdown_report` headers + module docstring | low | pending |
| 6 | Mapping invariant test in `test_mcnemar_analysis.py` | low | pending |
| 7 | Line ~291 "zero refusals" FIXME | zero | pending |
| 8 | Close Week 10 TODO #12 + correct previous session's framing | zero | pending |

---

## References

- Week 10 TODO #12 entry in `docs/WEEK10_TODO.md` (filed in
  commit `0ea0a27`)
- OSF pre-registration: DOI [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2)
- OSF Amendment 1: filed 2026-04-06, approved 2026-04-07
- Paper draft: `paper/draft_v0.1.md` at HEAD `7c76025`
- Code: `scripts/mcnemar_analysis.py` at HEAD `7c76025`
