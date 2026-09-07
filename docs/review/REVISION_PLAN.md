# Revision Plan — IEEE Access Manuscript Access-2026-32028

**Title:** Evaluating the Security of Open-Weight LLM-Generated Industrial Robot Code
**Decision (04 Sep 2026):** Reject with encouragement to resubmit (IEEE Access binary process).
One resubmission opportunity only; every reviewer concern must be addressed or rebutted.
**Reviewer 1:** minor revisions, positive. **Reviewer 2:** eight required items plus a longer list of gaps.
**Branch:** `review/access-r1`. **Paper source:** Overleaf export (`main_access.tex`, 62 refs, IEEE Access template); working copy to be placed under `paper/` on this branch.

Resubmission package (from the decision letter and checklist):
1. Response to reviewers (template attached to the letter): concern / response / action per item.
2. Highlighted PDF (all changes marked, yellow).
3. Clean manuscript as LaTeX + PDF.
4. No author-list change planned, so the byline-change form is not needed.

---

## 0. Evidence gathered before planning

These facts were verified against the repository and the shipped data. They shape the plan.

| # | Fact | Consequence |
|---|------|-------------|
| F1 | `results/E2_full/e2_results.csv` has 315 rows, `rep` = {1} only. 3 models × 15 tasks × 7 strategies × 1 rep = 315. | §V-C text "with three repetitions, for 315 generations" is wrong; Table 4 (45 cells) is right. (R2-3) |
| F2 | E1 baseline mean `total_violations`: 10.37 over the 83 gate-passing rows; 6.38 over all 135 rows with invalid/error rows counted as 0. | §VI-A uses the gate-passing denominator, §VI-B uses the all-rows denominator. Not an error in the data, an unreconciled denominator. (R2-3) |
| F3 | The 15 translator-generated URScript programs (`enfield_translators/generated/urscript/*.script`, deterministic, no LLM) fire the checker on 15/15: SM-4 (workspace checks) 15/15, SM-5 (hardcoded speed) 15/15, SM-6 (missing `set_tcp`/`set_payload`) 15/15; 5–9 firings per program. | A non-LLM comparator already exists in the repo and it **fails the checker too**. This is the honest answer to R2-1 and it also proves R2's point that part of 98.8% is rule design. (R2-1, R2 "98.8% partly artifact") |
| F4 | SM-5 applies the 0.25 m/s TCP limit to `movej(v=…)`, whose `v` is a joint speed in rad/s. `scripts/sm5_audit.py` and `scripts/sm5_corrected.py` already document this (unit conflation + mode insensitivity) and hold a candidate corrected rule, deliberately not wired in because it changes preregistered scoring. | Decision D1 below. The translator comparator's SM-5 firings are exactly this false positive. |
| F5 | Generated code is stored for every run: E1 260 files, E2 290, E3 330 (`results/*/code/`). | All re-scoring (corrected rules, comparator, per-strategy tables, trajectory quantification) is offline and cheap. No new LLM calls needed for the local models. |
| F6 | The validity gate is `URSCRIPT_CALL_PATTERN` in `enfield_llm/enfield_llm/code_parser.py`: a regex of ~20 URScript call names followed by `(`. No parser is involved. | R2-4 is correct; "grammar-anchored" / "parsed URScript grammar" must go. |
| F7 | Quantization: Qwen Q4_K_M, DeepSeek and CodeLlama Q4_0 (`docs/replication/MODEL_DIGESTS.txt` already flags the drift). | R2-7 confound is real and already documented internally; needs paper disclosure. |
| F8 | No figure-generation scripts exist in the repo; `figures/*.pdf` come from outside. | Figures 4 and 6 must be regenerated from scratch (matplotlib), which also fixes reproducibility. (R1-6, R2-7) |
| F9 | Full A8.1–A8.7 prompt templates exist in `prompt_builder.py` (`ADVERSARIAL_TEMPLATES`), plus `SYSTEM_PROMPT_BASELINE` / `SYSTEM_PROMPT_SAFETY`. | The "one full prompt per condition" appendix can be generated verbatim from code. (R2-6) |
| F10 | Task complexity scores exist (`results/task_complexity_scores.csv`, tertiles T_low/T_med/T_high) and E1 CSV carries `task_category`, `operating_mode`. | Task table and the complexity-stratified analysis can be produced from shipped data. (R2-6, R2 "complexity punted") |
| F11 | Ref 53 (Shukla et al.) is ISTAS 2025 with a DOI — published. Ref 54 (SCAFFOLD-CEGIS) is arXiv 2603.08520 (2026). | Check whether 54 has a venue; otherwise soften the sentence that leans on it. (R1-5) |
| F12 | Precision audit (Table 7): 50 firings, single rater, SM-3 and SM-7 not audited. | R1-3 and R2 "single rater" need a larger, two-rater audit with agreement statistic. |

---

## 1. Decisions the authors must make first

These change what gets built. Each blocks the items listed.

**D1 — Scoring basis for the revision (blocks R2-1, R2-3 partly, R1-3, Table 5/7).**
Options:
- (a) Keep the preregistered SM-5 as the confirmatory scoring, and add the corrected SM-5 (`sm5_corrected.py`) as a **sensitivity re-scoring** reported alongside. Report the translator comparator under both. No OSF amendment needed for confirmatory numbers; the corrected re-scoring is labelled non-preregistered. *Recommended: it is honest, cheap (offline), and directly answers R2's "rule-design artifact" without voiding the preregistration.*
- (b) Adopt the corrected SM-5 as the primary scoring. Requires re-scoring E1–E3, an OSF amendment (Georgios holds OSF authority), and rewriting every SM-driven number.

**D2 — Comparator set for R2-1.**
The 15 translator programs are the minimum. Recommended additions: (i) the 15 translator programs with the safety preamble the safety-prompted condition asks for (translator flag), which isolates SM-6; (ii) a small set of vendor/community URScript examples (UR script manual examples, `ur_robot_driver` example scripts) if licensing permits inclusion or at least scoring. Decide whether vendor examples are scored only (results reported) or also redistributed.

**D3 — Second rater for the precision audit (R1-3, R2 single-rater).**
Needs a co-author (or an external person) to independently label a stratified sample. Proposed: 20 firings per audited rule (SM-1, SM-2, SM-4, SM-5, SM-6, plus SM-3/SM-7 if they fire), two raters blind to each other, Cohen's κ reported. Who is rater 2?

**D4 — Title.**
R2-2 asks to rescope the title to code security. Current title already says "Security"; the abstract says "safety" twice in the first sentence. Proposed: keep title, move the scope sentence to the top of the abstract, and rename "motion-safety checks" framing in the abstract to "invariant task-specification checks (never fire by design)". Confirm whether a title tweak is wanted (e.g. adding "Static Code-Security Evaluation of …").

**D5 — Paper source in the repo.**
Put `main_access.tex` + figures under `paper/` on this branch so edits are versioned and diffable (needed for the highlighted PDF via `latexdiff`). The repo is public; decide whether `paper/` is merged to `main` or kept on the branch only. The `.eml` must not be committed (contains editor emails and a private submission URL); it is git-ignored on this branch.

---

## 2. Concern-by-concern plan

Status legend: `todo` / `doing` / `done` / `rebut` (counter-argue, no change) / `blocked(Dx)`.

### Reviewer 1

| ID | Concern | Action | Where | Effort | Status |
|----|---------|--------|-------|--------|--------|
| R1-1 | Abstract blurs confirmatory (3 models, prereg) with exploratory (frontier, 1 rep). 70% construct-removal reads like a finding. | Restructure abstract: confirmatory findings first, then one sentence explicitly labelled "exploratory, single repetition" for frontier/construct-removal. Remove the 70% figure from the abstract or attach "exploratory, n=1 per cell". | abstract; §I contributions | S | todo |
| R1-2 | Rule checker only validated on author-built variants; no independent validation. | Add to Future Work: independent validation against externally sourced violations (e.g. CVE-like URScript incidents, vendor safety bulletins). Cross-reference Appendix C caveat. The translator comparator (R2-1) also partly addresses this: it is independent of the variant generator. | §VII-D threats; §VIII | S | todo |
| R1-3 | Precision audit rests on 8–12 firings per rule, CI 55–99% for SM-5. | Enlarge audit (D3): ≥20 firings per rule, two raters, κ. Recompute Table 7 with tighter CIs. Add a main-text caveat sentence wherever "high-precision" is claimed if intervals remain wide. | Table 7; §VI-A | M | blocked(D3) |
| R1-4 | Scope condition (cybersecurity, not motion safety) buried in abstract. | Move to sentence 2 of the abstract and to the first paragraph of §I. Same fix serves R2-2. | abstract; §I | S | todo |
| R1-5 | Refs 53, 54 look like arXiv preprints; confirm status. | Ref 53 is ISTAS 2025 (DOI present): keep, fix formatting. Ref 54: check for a peer-reviewed version; if none, keep as arXiv and soften the dependent sentence in §II-D. | bibliography; §II-D | S | todo |
| R1-6 | Fig. 4 distinguishes four trajectory types by line style only; fails in grayscale. | Regenerate Fig. 4 with distinct markers + line styles + direct labels; verify in grayscale. Also quantify the regimes (see R2-M5). | `scripts/figures/fig_repair_trajectory.py` (new) | S | todo |

### Reviewer 2 — required items

| ID | Concern | Action | Where | Effort | Status |
|----|---------|--------|-------|--------|--------|
| R2-1 | Add a non-LLM comparator (~15 human/vendor URScript programs). | New script `scripts/comparator_urscript.py`: score (i) 15 translator programs, (ii) translator + safety preamble, (iii) vendor examples per D2, under preregistered rules and under corrected SM-5. New subsection §VI-A "Non-LLM comparator" with a table (per-rule firing counts, per-program violation count). State plainly that the reference translation fails SM-4/SM-6 by construction and SM-5 through the movej unit conflation, and that after the corrected SM-5 the comparator separates from the LLM outputs on the rules that remain. This turns 98.8% into an interpretable number. | new script; §VI-A; Table 5 | M | blocked(D1,D2) |
| R2-2 | Rescope title/abstract to code security; motion-safety half fires 0/585 by design. | Abstract and §I rewrite per R1-4/D4. In §IV-C rename "motion-safety pass" to make clear it is a specification check that is invariant across conditions, and say in the abstract that only the security pass carries signal. Consider dropping the motion-safety rows from the results narrative into a one-line protocol statement. | abstract; §I; §IV-C; §VI-A | S | blocked(D4) |
| R2-3a | §V-C arithmetic: "three repetitions, for 315 generations". | Change to "one repetition, for 315 generations (45 model-task cells × 7 strategies)". Explain why E2 has one rep (deterministic decoding; reps in E1 were identical). Check Table 4 and Appendix A consistency. | §V-C | S | todo |
| R2-3b | 10.37 vs 6.38 baseline means unreconciled. | State denominators explicitly: 10.37 = mean over 83 gate-passing baseline outputs; 6.38 = mean over all 135 with invalid outputs scored 0. Recompute the E2 contrast on the gate-passing subset so §VI-B uses the same basis as §VI-A, and report both. | §VI-A; §VI-B; Table 5 | S | todo |
| R2-4 | "Grammar-anchored" / "parsed URScript grammar" overstates a regex. | Replace throughout with "lexical, regex-based rule checker"; describe the gate precisely (list the call names or cite the regex); remove "parsed" and "grammar". Check §I, §IV, §IV-C, §VII, conclusion. | global | S | todo |
| R2-5 | Adversarial experiment has no table; per-strategy × per-model breakdown missing. | New Table: rows A8.1–A8.7, columns per model + pooled; cells: gate-pass count, mean violations (gate-passing), max severity, binary rate. Generated by `scripts/adversarial_table.py` from `E2_full`. Also report the binary-uplift test per strategy in the appendix. | §VI-B; new table | S | todo |
| R2-6a | No task table. | New Table: 15 tasks with ID, category, operating mode, tool, motion-command count, complexity tertile. From `enfield_tasks/ir/tasks` + `results/task_complexity_scores.csv`. | §IV-A | S | todo |
| R2-6b | No prompt text. | New Appendix: baseline system prompt, safety system prompt, one full user prompt (T001), and all seven A8 suffixes verbatim from `prompt_builder.py`. | new appendix | S | todo |
| R2-6c | No generated code listing. | New Appendix/figure: one annotated listing (before/after) showing the claimed phenomena: pseudo-code failing the gate; 200 mm/s emitted as 200 m/s; silent removal of the speed cap under repair. Pick from `results/E1_full/code` and `E3_full/code`; annotate rule firings in the margin. | new appendix | M | todo |
| R2-7a | Quantization confound (Q4_K_M vs Q4_0) on heterogeneity claim. | Disclose in §V-A (Table 3 already lists it) and add a sentence in §VI-D and threats: heterogeneity test cannot separate model identity from quantization. Optional: re-run Qwen at Q4_0 (1 model × 135 gens, cheap) to test sensitivity. | §V-A; §VI-D; §VII-D | S (+M optional) | todo |
| R2-7b | Remove trend guide from Fig. 6. | Regenerate Fig. 6 without the fitted line; show four points with exact-binomial CIs and n per point. | `scripts/figures/fig_construct_removal.py` (new) | S | todo |
| R2-8 | Cite IEC 62443, EU AI Act, MITRE CWE, ANSI/A3 R15.06-2025, URScript reference. | Add five bibitems and cite at first mention in §IV-D, §VII-D, §VII-F, §IV-C. | bibliography | S | todo |

### Reviewer 2 — further gaps ("Missing" and "Problems" lists)

| ID | Concern | Action | Where | Effort | Status |
|----|---------|--------|-------|--------|--------|
| R2-M1 | Detector validation circular. | Same as R1-2 + R2-1: comparator and future-work statement. Add an explicit "what the 114/120 does and does not show" sentence to Appendix C. | App. C; §VII-D | S | todo |
| R2-M2 | Manual audit single rater, no agreement statistic. | D3 / R1-3. | Table 7 | M | blocked(D3) |
| R2-M3 | Preregistered thresholds untestable. | Already framed as a lesson. Add one sentence: the confirmatory arm for H5 is reported as voided by ceiling, and the count/severity contrasts are labelled post hoc. No further change. | §VI-B | S | todo |
| R2-M4 | Repair-trajectory regimes never quantified for local models. | From `E3_full`: classify each of the 135 cells into the four regimes (repaired / oscillating / degraded-to-invalid / unchanged) by rule; report a small table; Fig. 4 shows real examples with counts in the legend. | §VI-C; Fig. 4 | M | todo |
| R2-M5 | Complexity-stratified analysis punted. | Run `scripts/complexity_correlation_analysis.py` on E1/E3; report the tertile-stratified rates and the (likely null) test in a short paragraph + appendix table. | §VI-D; appendix | S | todo |
| R2-M6 | Appendix B is a one-line stub. | Delete; move the pointer sentence into Appendix A. | App. B | S | todo |
| R2-M7 | Table 5 mixes confirmatory and exploratory rows with cryptic notes. | Split into two tables (confirmatory / additional). Expand "descr.", "ceiling demo" into full words. | Table 5 | S | todo |
| R2-M8 | Two highest-firing checks audit at 12.5% / 0% precision. | Already disclosed. Add the "restricted to high-precision rules" rate to Table 5 as its own row; after D1, report the corrected-SM-5 rate too. | Table 5; §VI-A | S | todo |

---

## 3. Execution order

Each step ends with a commit on `review/access-r1`; the status column above is updated in the same commit.

**Phase 0 — Setup (now)**
1. Ignore `docs/review/*.eml`; commit this plan.
2. Unzip Overleaf source into `paper/` (D5); confirm it builds locally (`pdflatex` or `latexmk`); commit as the pre-revision baseline tagged `paper-submitted-r0` so `latexdiff` can produce the highlighted PDF.

**Phase 1 — Data and analysis (repo code, no paper edits)**
3. `scripts/adversarial_table.py` → per-strategy × per-model table (R2-5) and the gate-passing E2 contrast (R2-3b).
4. `scripts/task_table.py` → task table (R2-6a).
5. `scripts/comparator_urscript.py` → non-LLM comparator under both scorings (R2-1, after D1/D2).
6. `scripts/e3_regimes.py` → trajectory regime classification (R2-M4).
7. Complexity-stratified analysis run (R2-M5).
8. Precision audit v2 tooling: `scripts/precision_audit_sample.py` to draw the stratified sample and blank labelling sheets for two raters; κ computation (R1-3, after D3).
9. Optional: Qwen Q4_0 sensitivity run (R2-7a) — needs the Ollama host.

**Phase 2 — Figures**
10. `scripts/figures/` with matplotlib sources for Fig. 4 (regimes, grayscale-safe) and Fig. 6 (no trend line, CIs). Regenerate any other figure whose data changed.

**Phase 3 — Paper text**
11. Abstract + §I rescoping (R1-1, R1-4, R2-2).
12. Terminology sweep "grammar-anchored"/"parsed" (R2-4).
13. §V-C arithmetic, §VI-A/B denominators, Table 5 split (R2-3, R2-M7, R2-M8).
14. New tables/appendices: task table, adversarial table, prompts, code listing, regimes, complexity (R2-5, R2-6, R2-M4, R2-M5).
15. Comparator subsection (R2-1) and precision audit v2 (R1-3).
16. Quantization disclosure, Fig. 6 caption, five citations, refs 53/54, Appendix B removal, future-work sentence (R2-7, R2-8, R1-2, R1-5, R2-M6).

**Phase 4 — Resubmission package**
17. `paper/response_to_reviewers.md` → filled from this plan (concern / response / action), exported to the IEEE template `.docx`.
18. `latexdiff` submitted vs revised → highlighted PDF; clean PDF; LaTeX source zip.
19. Walk the IEEE checklist (grammar, math formatting, reference formatting, ORCID metadata).

---

## 4. Open questions for the authors

- D1–D5 above.
- Is there a resubmission deadline in the author portal? The letter does not state one.
- Does the OSF registration need an addendum noting the corrected-SM-5 sensitivity analysis (even under D1-a), for transparency?
- Who runs the Ollama host for the optional Q4_0 run?
