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
| F13 | With the safety preamble added, the translator output clears SM-6 but SM-2 then fires 15/15: `set_tcp`/`set_payload` are "critical operations" that SM-2 wants wrapped by `popup`/`halt` within three lines. | Two checks pull in opposite directions; the rule set cannot be satisfied by the reference idiom without a contrived guard. Must be disclosed with the comparator. |
| F14 | Corrected SM-5 removes every SM-5 firing on translator output (15 → 0 files) but only 54 → 46 of 83 on LLM output; gate-passing violation rate stays 98.8%. Under the three high-precision checks plus corrected SM-5: 91.6% (E1) but 75.8% (frontier E1), substantive-safe 29/120. | Headline for local models is robust to rule choice; the frontier "100%" is not and needs a caveat. (`paper/tables/sensitivity_e1_*.tex`) |
| F15 | Temperature-zero repetitions are not identical: byte-identical output in 14/90 E1 cells, same binary verdict in 82/90, same violation count in 59/90; DeepSeek agrees on count in only 10/30 cells. | The paper's "nearly identical" claim holds at the verdict level only; report it (`rep_reproducibility.tex`) and keep the cell-level unit. |
| F16 | Construct removal by model size (E5) is not monotone: 0/30, 15/30 (50.0%), 13/29 (44.8%), 21/30 (70.0%). | §VI-E and the Fig. 6 caption say "monotone increase"; correct to "rises from 0% to 70% with a non-monotone middle". |
| F17 | All experiments used the task IR at 7c67b68 (= cc71721). The pre-approach edit of 2026-09-07 post-dates them. | Task-facing tables and prompts are generated from `paper/data/tasks_as_run/`; the replication appendix must say so. |
| F18 | E3 regimes for the 135 local cells: 78 drop out of validity (52 at retry 0), 57 stay valid, 0 reach zero; rule churn per valid step 0.26 cleared vs 0.29 introduced. | Quantifies Fig. 4 (`e3_regimes.tex`). |

---

## 1. Decisions the authors must make first

**Resolved 07 Sep 2026 (corresponding author):**
- D1 → (a): preregistered SM-5 stays confirmatory; corrected SM-5 reported as a non-preregistered sensitivity re-scoring.
- D2 → full set: 15 translator programs, translator + safety preamble, vendor/community URScript examples where licensing permits.
- D3 → human second rater to be recruited by the authors; a rater guide is provided in `docs/review/PRECISION_AUDIT_RATER_GUIDE.md`.
- D4 → keep the title; move the scope sentence to the top of the abstract.
- D5 → paper versioned under `paper/` on this branch; **branch stays local, is not pushed, and is not merged to `main`** while the repository is public.

Original decision text kept below for the record.


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
| R1-1 | Abstract blurs confirmatory (3 models, prereg) with exploratory (frontier, 1 rep). 70% construct-removal reads like a finding. | Restructure abstract: confirmatory findings first, then one sentence explicitly labelled "exploratory, single repetition" for frontier/construct-removal. Remove the 70% figure from the abstract or attach "exploratory, n=1 per cell". | abstract; §I contributions | S | done (abstract, contributions, Tables 6/7) |
| R1-2 | Rule checker only validated on author-built variants; no independent validation. | Add to Future Work: independent validation against externally sourced violations (e.g. CVE-like URScript incidents, vendor safety bulletins). Cross-reference Appendix C caveat. The translator comparator (R2-1) also partly addresses this: it is independent of the variant generator. | §VII-D threats; §VIII | S | done (App. D paragraph, threats paragraph) |
| R1-3 | Precision audit rests on 8–12 firings per rule, CI 55–99% for SM-5. | Enlarge audit (D3): ≥20 firings per rule, two raters, κ. Recompute Table 7 with tighter CIs. Add a main-text caveat sentence wherever "high-precision" is claimed if intervals remain wide. | Table 7; §VI-A | M | tooling done (sampler, guide, analyzer); awaiting rater labels |
| R1-4 | Scope condition (cybersecurity, not motion safety) buried in abstract. | Move to sentence 2 of the abstract and to the first paragraph of §I. Same fix serves R2-2. | abstract; §I | S | done |
| R1-5 | Refs 53, 54 look like arXiv preprints; confirm status. | Ref 53 is ISTAS 2025 (DOI present): keep, fix formatting. Ref 54: check for a peer-reviewed version; if none, keep as arXiv and soften the dependent sentence in §II-D. | bibliography; §II-D | S | done (ref 53 published; ref 54 preprint, softened) |
| R1-6 | Fig. 4 distinguishes four trajectory types by line style only; fails in grayscale. | Regenerated with distinct markers + line styles + direct labels, legend carries regime counts. | `scripts/figures/fig_repair_trajectory.py` | S | done (Fig. 5 regenerated, caption updated) |

### Reviewer 2 — required items

| ID | Concern | Action | Where | Effort | Status |
|----|---------|--------|-------|--------|--------|
| R2-1 | Add a non-LLM comparator (~15 human/vendor URScript programs). | New script `scripts/comparator_urscript.py`: score (i) 15 translator programs, (ii) translator + safety preamble, (iii) vendor examples per D2, under preregistered rules and under corrected SM-5. New subsection §VI-A "Non-LLM comparator" with a table (per-rule firing counts, per-program violation count). State plainly that the reference translation fails SM-4/SM-6 by construction and SM-5 through the movej unit conflation, and that after the corrected SM-5 the comparator separates from the LLM outputs on the rules that remain. This turns 98.8% into an interpretable number. | new script; §VI-A; Table 5 | M | done (Sec. VI-A comparator, Table 11, Fig. 4) |
| R2-2 | Rescope title/abstract to code security; motion-safety half fires 0/585 by design. | Abstract and §I rewrite per R1-4/D4. In §IV-C rename "motion-safety pass" to make clear it is a specification check that is invariant across conditions, and say in the abstract that only the security pass carries signal. Consider dropping the motion-safety rows from the results narrative into a one-line protocol statement. | abstract; §I; §IV-C; §VI-A | S | done (abstract, Sec. I, IV; title kept) |
| R2-3a | §V-C arithmetic: "three repetitions, for 315 generations". | Change to "one repetition, for 315 generations (45 model-task cells × 7 strategies)". Explain why E2 has one rep (deterministic decoding; reps in E1 were identical). Check Table 4 and Appendix A consistency. | §V-C | S | done |
| R2-3b | 10.37 vs 6.38 baseline means unreconciled. | State denominators explicitly: 10.37 = mean over 83 gate-passing baseline outputs; 6.38 = mean over all 135 with invalid outputs scored 0. Recompute the E2 contrast on the gate-passing subset so §VI-B uses the same basis as §VI-A, and report both. | §VI-A; §VI-B; Table 5 | S | done (both denominators defined and reported) |
| R2-4 | "Grammar-anchored" / "parsed URScript grammar" overstates a regex. | Replace throughout with "lexical, regex-based rule checker"; describe the gate precisely (list the call names or cite the regex); remove "parsed" and "grammar". Check §I, §IV, §IV-C, §VII, conclusion. | global | S | done (global sweep) |
| R2-5 | Adversarial experiment has no table; per-strategy × per-model breakdown missing. | New Table: rows A8.1–A8.7, columns per model + pooled; cells: gate-pass count, mean violations (gate-passing), max severity, binary rate. Generated by `scripts/adversarial_table.py` from `E2_full`. Also report the binary-uplift test per strategy in the appendix. | §VI-B; new table | S | done (Table 13, Sec. VI-B rewritten) |
| R2-6a | No task table. | New Table: 15 tasks with ID, category, operating mode, tool, motion-command count, complexity tertile. From `enfield_tasks/ir/tasks` + `results/task_complexity_scores.csv`. | §IV-A | S | done (Table 3) |
| R2-6b | No prompt text. | New Appendix: baseline system prompt, safety system prompt, one full user prompt (T001), and all seven A8 suffixes verbatim from `prompt_builder.py`. | new appendix | S | done (App. B) |
| R2-6c | No generated code listing. | New Appendix/figure: one annotated listing (before/after) showing the claimed phenomena: pseudo-code failing the gate; 200 mm/s emitted as 200 m/s; silent removal of the speed cap under repair. Pick from `results/E1_full/code` and `E3_full/code`; annotate rule firings in the margin. | new appendix | M | done (App. C) |
| R2-7a | Quantization confound (Q4_K_M vs Q4_0) on heterogeneity claim. | Disclose in §V-A (Table 3 already lists it) and add a sentence in §VI-D and threats: heterogeneity test cannot separate model identity from quantization. Optional: re-run Qwen at Q4_0 (1 model × 135 gens, cheap) to test sensitivity. | §V-A; §VI-D; §VII-D | S (+M optional) | threats text done; Q4_0 re-run in progress (X10) |
| R2-7b | Remove trend guide from Fig. 6. | Regenerate Fig. 6 without the fitted line; show four points with exact-binomial CIs and n per point. | `scripts/figures/fig_construct_removal.py` (new) | S | done (Fig. 7, captions, non-monotone wording) |
| R2-8 | Cite IEC 62443, EU AI Act, MITRE CWE, ANSI/A3 R15.06-2025, URScript reference. | Add five bibitems and cite at first mention in §IV-D, §VII-D, §VII-F, §IV-C. | bibliography | S | done (refs 63-67) |

### Reviewer 2 — further gaps ("Missing" and "Problems" lists)

| ID | Concern | Action | Where | Effort | Status |
|----|---------|--------|-------|--------|--------|
| R2-M1 | Detector validation circular. | Same as R1-2 + R2-1: comparator and future-work statement. Add an explicit "what the 114/120 does and does not show" sentence to Appendix C. | App. C; §VII-D | S | done |
| R2-M2 | Manual audit single rater, no agreement statistic. | D3 / R1-3. | Table 7 | M | tooling done; awaiting rater labels |
| R2-M3 | Preregistered thresholds untestable. | Already framed as a lesson. Add one sentence: the confirmatory arm for H5 is reported as voided by ceiling, and the count/severity contrasts are labelled post hoc. No further change. | §VI-B | S | done |
| R2-M4 | Repair-trajectory regimes never quantified for local models. | From `E3_full`: classify each of the 135 cells into the four regimes (repaired / oscillating / degraded-to-invalid / unchanged) by rule; report a small table; Fig. 4 shows real examples with counts in the legend. | §VI-C; Fig. 4 | M | done (Table 14 + text) |
| R2-M5 | Complexity-stratified analysis punted. | Run `scripts/complexity_correlation_analysis.py` on E1/E3; report the tertile-stratified rates and the (likely null) test in a short paragraph + appendix table. | §VI-D; appendix | S | done (Table 15 + text) |
| R2-M6 | Appendix B is a one-line stub. | Delete; move the pointer sentence into Appendix A. | App. B | S | done (App. B removed; provenance para in App. A) |
| R2-M7 | Table 5 mixes confirmatory and exploratory rows with cryptic notes. | Split into two tables (confirmatory / additional). Expand "descr.", "ceiling demo" into full words. | Table 5 | S | done (Tables 6/7) |
| R2-M8 | Two highest-firing checks audit at 12.5% / 0% precision. | Already disclosed. Add the "restricted to high-precision rules" rate to Table 5 as its own row; after D1, report the corrected-SM-5 rate too. | Table 5; §VI-A | S | done (Table 6 rows) |

---

## 3. Execution order

_Status 07 Sep 2026, evening: Phases 0-3 done except the two items that wait on external input (rater 2 labels; X10 run finishing). Phase 4 started: response letter drafted with placeholders. Revised manuscript compiles to 26 pages (submitted: 22)._

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

## 3a. Post-hoc task-IR change: assessment (asked by the corresponding author, 07 Sep 2026)

**Question.** The pre-approach `move_joint` added to eight tasks on 2026-09-07 (e6b5ffb) post-dates every experiment. Should the change be reverted and the comparisons redone, or should the experiments be re-run on the changed tasks?

**Assessment.** Neither. The stored results are not contaminated: every generation, verdict and log in `results/` was produced from the task IR at 7c67b68 (= cc71721 = tag `v1.0.0`), and that IR is preserved verbatim in `paper/data/tasks_as_run/`. The only things the edit could have contaminated were derived artefacts generated from the live directory (task table, prompts appendix, translator comparator, per-task caps); those now read the snapshot (cab136d). The edit does not change any safety verdict on the baselines (15/15 safe before and after) or the variant detection rate (114/120 before and after), because the added step is a joint move at the task's own speed inside the safeguarded space.

- Reverting would discard a genuine execution fix (URSim shoulder-singularity) and gain nothing, since the paper's numbers never depended on the live directory.
- Re-running E1–E3 on the changed tasks (~925 local generations, roughly 8–15 GPU-hours) would sever the link to the OSF preregistration, which registered the task set, and would make the revision a new study rather than a revised one. It is not what the reviewers asked for.

**Action.** Keep both states, and make the provenance explicit: the manuscript cites tag `v1.0.0` as the state under which all experiments ran, the replication appendix notes the later pre-approach refinement and that it is verdict-neutral, and the release accompanying the revision carries the `tasks_as_run` snapshot. Any *new* generation done for the revision (X10) uses the snapshot via `ENFIELD_TASKS_DIR=paper/data/tasks_as_run`.

## 3b. Additional low-cost analyses (offline, already computed unless marked)

All of these use stored outputs; none needs a new LLM call. Proposed for inclusion; the corresponding author decides which go into the manuscript.

| # | Analysis | What it answers | Status | Where |
|---|----------|-----------------|--------|-------|
| X1 | Non-LLM comparator under two scorings (translator, translator+preamble, 9 vendor/community programs) | R2-1; shows which rules are idiom-level (SM-4, SM-6) vs code-level | computed | `comparator_summary.tex` |
| X2 | Sensitivity of headline rates to rule set (5 variants) and gate (lexical vs structural), E1 and frontier | R2 "artefact" and "regex gate" critiques; bounds the headline | computed | `sensitivity_e1_baseline.tex`, `sensitivity_e1_frontier.tex` |
| X3 | Repetition reproducibility at five levels (text/status/binary/count/rules) | supports the cell-level unit; documents that greedy decoding is not bit-stable on the serving stack | computed | `rep_reproducibility.tex` |
| X4 | E3 trajectory regimes, all 135 cells, with rule churn (cleared vs introduced per step) | R2-M4; turns Fig. 4 from schematic into counts | computed | `e3_regimes.tex` |
| X5 | Per-strategy × per-model adversarial table on both denominators | R2-5, R2-3b; A8.4 is the only strategy with a large effect (+6.2 on gate-passing basis) | computed | `adversarial_table.tex` |
| X6 | Complexity-stratified rates and per-model Spearman ρ | R2-M5; null as expected (1 of 9 ρ nominally p<0.05, uncorrected) | computed | `complexity_table.tex` |
| X7 | Rule-interaction note: preamble satisfies SM-6 but trips SM-2 | explains part of the 98.8% as a rule-set property; strengthens the "measurement hazard" contribution | computed (F13) | text only |
| X8 | Frontier re-scoring under corrected SM-5 / high-precision set | caveats the frontier "100%" (drops to 75.8%) | computed | `sensitivity_e1_frontier.tex` |
| X9 | Precision audit v2, two raters, κ | R1-3, R2-M2 | tooling done; needs rater 2 | `audit_v2/` |
| X10 | Qwen at Q4_0 (90 generations: 15 tasks × 2 conditions × 3 reps) to test the quantization confound | R2-7a | in progress on the local RTX 5090 (user-space Ollama) | `scripts/review/quantization_compare.py` |
| X11 | Stricter-gate substantive-safe rate (structural gate) | complements X2; 38/130 pass, 0–1 substantive-safe | computed | in X2 tables |
| X12 | Per-model × per-rule firing heat-map (E1 baseline vs safety, plus translator and vendor rows) | visual replacement for Table 6 counts | computed | `fig_rule_heatmap.pdf` |

## 4. Open questions for the authors

- D1–D5 above.
- Is there a resubmission deadline in the author portal? The letter does not state one.
- Does the OSF registration need an addendum noting the corrected-SM-5 sensitivity analysis (even under D1-a), for transparency?
- Who runs the Ollama host for the optional Q4_0 run?
