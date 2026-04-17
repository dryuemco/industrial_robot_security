# Brief: H5 outcome, B+C quick-wins, Amendment 2 submission request

**From:** Yunus Emre Cogurcu
**To:** Assoc. Prof. Georgios Spathoulas
**Date:** 2026-04-16 (Week 11/24)
**Repo HEAD:** `d28a4f0` on `origin/main`, 713 tests passing

---

## TL;DR

H5 is **not supported** under the preregistered decision rule, because a binary-flag ceiling (gate-passing baseline CVR = 0.988) leaves the +50 pp adversarial threshold mathematically unreachable. I took the full-disclosure writing path that you delegated; three follow-on writing commits landed today. **Requesting your go-ahead to submit OSF Amendment 2.**

---

## H5 result (E2, 2026-04-15, 315 calls, ~100 min runtime)

Design: 3 models × 15 tasks × 7 A8 conditions × 1 rep, paired per (model, task) to the E1 baseline condition (three reps, majority-vote).

- **Naive contrast** (all matched pairs): max per-model ΔCVR = **+26.67 pp** (DeepSeek-Coder-V2-16B / A8.5); max pooled = +6.67 pp (A8.5).
- **Gate-passing contrast** (both sides validate URScript): max per-model = **+33.33 pp** (DeepSeek / A8.1 & A8.4); max pooled = +4.76 pp (A8.1).
- All **42 per-model + 14 pooled** McNemar contrasts return Holm-adjusted p = 1.000.
- Per §V.E decision rule ("at least one cell rejects H0: ΔCVR < 0.50 after Holm–Bonferroni"): **not met under either contrast.**

Full tables in §VI.G of `paper/draft_v0.1.md`.

---

## Root cause: binary-flag ceiling

Gate-passing baseline CVR = **0.988** (§VI.F, Table III(a)-gate). Only 1.2 pp of ceiling room remained for any adversarial condition to produce a measurable binary increase, and the 100% ceiling is in fact reached by every gate-passing adversarial cell. The +50 pp preregistered threshold is mathematically unreachable in principle under the binary `has_violation` metric.

The NOT SUPPORTED verdict is therefore a **metric–data interaction under the ceiling**, not evidence that adversarial prompts leave safety unchanged.

Count-level descriptive means (§VI.G, Table IV(b)) sit below the ceiling and show a model-dependent uplift:

- Qwen2.5-Coder-32B / A8.4: 10.36 → 21.33 mean `total_violations` per call
- DeepSeek-Coder-V2-16B / A8.6: 9.50 → 13.80
- CodeLlama-34B: positive on A8.1, A8.2, A8.4, A8.7 but N ≤ 7 per cell, high variance

I have not applied a statistical test to these means since count-level testing was not preregistered. Reported as post-hoc descriptive only.

---

## Writing strategy (chosen while you were unavailable)

**Full disclosure + methodological transparency** rather than minimal preregistered-only reporting. Paper sections added/modified in Session 15:

- **§VI.G:** NOT SUPPORTED reported with both contrasts; Table IV(b) labelled "post-hoc, descriptive; not part of the H5 confirmatory test" with no p-values and no supported/not-supported verdict.
- **§VII.B.8 (new):** Binary ceiling + DM/SM asymmetry, proposing variant-IR protocol + multi-level metric as future work.
- **§V.C (new paragraph):** Protocol-level invariant clarified — Task IR held constant across conditions; DM pass reports 0 violations in 585/585 E1+E2 rows; combined (DM ∪ SM) signal reduces to SM.

Alternative minimal framing (~2 paragraphs, no ceiling discussion) was rejected as insufficient for a transparent IEEE RA-L submission. I am happy to revert if you prefer.

---

## Today's commits (Session 16, B+C quick-wins)

All three on `origin/main`, 713 tests still passing, PENDING DATA markers **3 → 2**:

| Commit | Summary |
|---|---|
| `c405242` | fix(paper): baseline Holm-adjusted p in VI.F corrected 0.002 → 0.004 (value was raw p mislabelled as Holm-adjusted; CSV Holm-adjusted = 0.003673 rounds to 0.004) |
| `ca2ba91` | docs(paper): §VI.J Cochran Q populated — Q = 12.60 (baseline), 20.00 (safety), 8.00 (adversarial-any); all REJECT H0 at family-wise α = 0.05 |
| `d28a4f0` | docs(paper): safety-prompt count-level effect added to VI.F — pooled gate-passing mean falls 10.07 → 6.38 (37% reduction); DeepSeek 61%, CodeLlama 68%, Qwen unchanged |

---

## Requests

1. **OSF Amendment 2 submission go-ahead.** Candidate is `d251c15`, filed locally 2026-04-07 in `docs/OSF_PREREGISTRATION.md`. Scope change: seven-subtype A8 family instead of preregistered eight (one variant dropped as specification drift, documented in §VII.B.6). Once OSF acknowledges, four locked artefacts (§V.E, §VI.G H5 decision sentence, Appendix A H5 row, prereg Amendment 1 H5 row) align in one follow-up commit.

2. **Writing-strategy confirmation.** Full disclosure (§VII.B.8, exploratory subsection in §VI.G) reflects my judgment; please flag if the framing needs adjustment before it propagates to the abstract, conclusion, and arXiv preprint.

3. **E3 design heads-up (for next meeting).** Running E3 under the same invariant-IR protocol would likely yield H6 NOT SUPPORTED for the same ceiling reason. Two options:
   - **(a)** Keep invariant-IR, frame H6 around count-level uplift instead of binary rate.
   - **(b)** Adopt variant-IR protocol that mutates the Task IR between conditions, exercising DM-1..7 properly.
   Your preference before the E3 run would avoid a second dead-end confirmatory data collection.

---

## Outstanding paper items

| Section | Status |
|---|---|
| §VI.F (H4) | Complete + today's count-level augmentation |
| §VI.G (H5) | Complete (NOT SUPPORTED + exploratory count) |
| §VI.H (H6) | Pending E3 data + design decision above |
| §VI.I (sensitivity) | Pending `scripts/mcnemar_analysis.py --sensitivity` flag review; starting next |
| §VI.J (Cochran) | Complete today |
| §VII.B.1–8 | Complete |
| §VII.A, §VII.C | Stable |

---

## Next session (my side, no blocker on you)

- Inspect `scripts/mcnemar_analysis.py --sensitivity` behaviour, populate §VI.I
- Session 16 closure commit to `docs/WEEK11_SPRINT.md`

Looking forward to your feedback on requests 1–3 at your convenience.

— Yunus
