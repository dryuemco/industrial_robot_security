# Round 3 (Access-2026-55984): changes to the paper

Branch `review/access-r3`, started from `review/access-r2` at `797aeb3` (the source of the
Access-2026-55984 submission). The response letter is written separately. Items A1 to A4
are Reviewer 1's analyses; V1 to V6 are corrections reported by the authors without a
reviewer request. B1 to B9 have not arrived and nothing here addresses them.

Every new number in the paper comes from a file under `analysis/round3/out/`; the source
column of the table below names it. The tables changed or added in this round are written
by `analysis/round3/s09_tables.py` from those files (`analysis/round3/run_all.sh` checks that
the files in `paper/tables/` equal its output). No code, data or preregistered scoring was
changed. The only file outside the paper that changed is the OSF mirror (V6).

## Commits

| Commit | Content |
|---|---|
| `fe24baf` | analysis: round-3 computations (`analysis/round3/`) |
| `6ad9b1d` | paper: abstract, Table 14 comparator, SM-6 lower bound (A1) |
| `9cfb029` | paper: cross-model test as heterogeneity in syntactic commitment (A2) |
| `1d35ec2` | paper: per-rule decomposition, rater B primary (A3, A4; V5) |
| `1dca8dd` | paper, docs: V1, V2, V4, V6 and the consistency edits listed below |
| this commit | docs: this file and the numeric-token comparison; Conclusion second finding aligned with the abstract; the analysis scripts `s00` to `s07` now read the paper and the OSF mirror at `797aeb3` from git, so the round-3 edits leave their committed outputs unchanged (`run_all.sh` reproduces every file under `analysis/round3/out/` byte for byte) |

`./scripts/run_tests.sh --fast` passed (919 tests) before each commit.

## Builds and page count

| PDF | Pages |
|---|---|
| r2 (`797aeb3`), rebuilt from the commit | 30 |
| r2 highlighted PDF as uploaded (`docs/review/submission_v9/02_highlighted_pdf/`) | 30 |
| r3 clean (`paper/main_access.pdf`) | 32 |
| r3 highlighted (`paper/diff/main_access_diff.pdf`) | 32 |

The highlighted PDF is built as in the last round: `paper/build_diff.sh 797aeb3` (latexdiff
against the submitted source, yellow additions and red struck-through deletions, and
`paper/mark_new_floats.py` for floats). Floats marked "Revised in this revision": Tables 6,
10, 14 and 17 and Figure 4; marked "New in this revision": Tables 13 and 19. The clean build
has the same five overfull boxes as the r2 build, all in text this round did not change, so
none is new.

Table numbers change after Table 12, because a new table enters as Table 13: the old Table 13
(sensitivity) is now Table 14, 14 (adversarial) is 15, 15 (repair regimes) is 16, 16
(quantization) is 17, 17 (complexity) is 18, the new registered cross-model table is 19,
and the old Tables 18, 19 and 20 are 20, 21 and 22. The reviewer's "Table 13" and "Table 16"
are now Tables 14 and 17.

## Changes

Line numbers refer to `paper/main_access.tex` in this commit.

| Section | Line | Item | Old | New | Source |
|---|---|---|---|---|---|
| Abstract | 47-68 | A1 | round-2 abstract ("1 to 9 of 135", "about twice as many rule firings") | the round-3 text, verbatim (248 words) | given |
| I-C, contributions | 174 | V4 | "1 of 135 at baseline" | "1 of the 135 baseline generations" | `out/r3/table6_fractions.csv` |
| V-E | 802-805 | A2 | "reported as an additional analysis only" | registered in the first amendment as exploratory, one per prompt condition, uncorrected p | `out/f1/f4_osf_cross_model.md` |
| VI, Table 6 | 875-897 | V4, A4, A2 | rates as percentages; rater B and adjudicated rows unlabelled; cross-model row "p=0.004 ... Holm-corrected" | every rate as k/N with its denominator; rater B row "(primary)", adjudicated row "(sensitivity)"; cross-model row relabelled, exact uncorrected p, gate-passing result; columns ragged-right and the value column widened from 0.17 to 0.19 column width (layout only) | `out/r3/table6_fractions.csv`, `out/a2/cochran_all.csv` |
| VI-A | 980-986 | V1 | "This single safe output is genuine code rather than an empty skeleton, since every gate-passing baseline generation contains motion commands." | registered 1/135 kept; the output passes the lexical gate only through a commented `stopl()`, writes motion without URScript call syntax, fails the structural gate; the motion-command flag is a substring search | `out/f1/f6_t009.md` |
| VI-A | 1096-1099 | A4 | adjudicated coincides with rater A; rater B "the conservative estimate" for readers who prefer it | adjudicated = rater A on all 100 items; rater B primary, adjudicated as sensitivity | `out/a4/precision.csv` |
| VI-A | 1104-1113 | A4 | (none) | SM-2 audit unit (program) and count unit (call site): 111 firings on 82 programs, 1.34 per program, 0.99 deduplicated; audit items 11 program-level (10 genuine for both raters) and 9 call-site (8 for A, 0 for B) | `out/r3/sm2_per_program.csv`, `out/a4/sm2_by_location.csv` |
| VI-A | 1132-1134 | A4 | "85% and 80% after adjudication (55% and 80% for rater B)" | rater B first, adjudicated in parentheses | `out/a4/precision.csv` |
| VI-A | 1138-1148 | A4, V4 | rater B and adjudicated restrictions without status; "9 rather than 1 of 130", "1 of 130", "1 of 135" | "the primary estimate", "the sensitivity analysis"; "of the 130 stored baseline outputs", "82 of 83", "1 of the 135 baseline generations" | `out/r3/table6_fractions.csv` |
| VI-A | 1155 | A4 | Table 10 with rater A, rater B and adjudicated columns | Table 10: rater B (primary), rater A = adjudicated (sensitivity), agreement and kappa; caption states the 100 of 100 identity | `out/a4/precision.csv` |
| VI-A | 1166-1170 | A3 | safety-prompt fall of the mean (10.37 to 6.39) not decomposed | fall of 3.98 by check: SM-1 2.32, SM-6 0.94, SM-5 0.32, SM-4 0.24, SM-2 0.15; 3.56 with corrected SM-5, where SM-5 rises by 0.11 | `out/r3/count_differences.csv` |
| VI-A, summary | 1186 | V4 | "1 of 135 (0.7%)" | "1 of the 135 baseline generations (0.7%)" | `out/r3/table6_fractions.csv` |
| VI-A, comparator | 1213-1227 | A3, V5 | "the count and the rule profile do separate generated code from the reference: ... 10.4 ... against 5.7 ... on which the reference never fires" | excess 4.64 of which SM-1 and SM-2 4.49 (rater B 45% and 50%); first-audit checks differ by 0.15 (5.88 against 5.73); corrected SM-5 excess 6.84, SM-1 and SM-2 4.49, remaining 2.35 from speed literals above the limit; "never fires" holds for the minimal translation, the preamble variant fires SM-2 on 15 of 15 | `out/a3/per_rule_means.csv`, `out/a3/differences.csv`, `out/r3/first_audit_rules.csv`, `out/a3/never_fires.md` |
| VI-A, Figure 4 caption | 1238 | V5 | "with the reference translation" | "with the minimal reference translation (without the preamble)"; the figure's comparison row is the minimal translator | `scripts/figures/fig_rule_heatmap.py` (row "Translator") |
| VI-A | 1246 | A3 | (none) | new Table 13: mean firings per program by check, LLM against both translations, with differences, preregistered and corrected SM-5 | `out/a3/per_rule_means.csv` |
| VI-A, sensitivity | 1258-1264 | V2, V4 | "at most 9 of 130 scored outputs ... leaves at most three of them violation-free" | six of the nine are pseudo-code without URScript call syntax and fail the structural gate; structural gate gives 0 to 3 of the 130 stored outputs | `out/f1/f7_sm6_only_safe.csv`, `out/a1/table13_with_translator.csv` |
| VI-A | 1272-1293 | A1 | (none) | new paragraph: all 74 flagged outputs lack set_tcp, 68 also set_payload; prompt gives payload, not TCP; safety prompt 68/83 to 14/108, Qwen 39/45 to 0/45, Qwen 45/45 still lack set_tcp, CodeLlama 18/18 and 10/10; translator min/pre 15 and 0 flagged; what the bound shows and does not show; SM-6 anchored to 5.1.14/5.1.15, no 5.1.16 anchor; rater B 18/20, Wilson [69.9, 97.2] | `out/f1/f1_sm6_groups.csv`, `out/f1/f2_safety_vs_baseline.csv`, `out/a1/prompt_fields.csv`, `out/a1/ir_tcp_payload.csv`, `out/a1/sm6_clause_map.md`, `out/a4/precision.csv` |
| VI-A | 1295 | A1, V4 | Table 13 (LLM only, "n=130 generations") | Table 14: reference translation columns (flagged of 15 and mean, min. / pre.) for every gate and rule set; caption "130 stored outputs of the 135 generations" | `out/a1/table13_with_translator.csv` |
| VI-B | 1313-1320 | A3 | adversarial count uplift not decomposed | A8.4: SM-5 +8.11 (7.35 corrected), SM-1 -2.01; pooled: SM-5 +2.04 (1.89 corrected), SM-1 -0.95; uplift remains under corrected SM-5 (5.45 and 1.08) | `out/r3/count_differences.csv`, `out/r3/count_check.csv` |
| VI-D | 1402-1421 | A2, V3 | "cross-model homogeneity test ... 12.60 (p of 0.004) ... 20.00 (p below 0.001), with multiplicity correction applied within this additional family only" | heterogeneity in syntactic commitment; registered in the first amendment, exploratory, uncorrected; exact p 0.001 and below 0.001; gate-passing: undefined at baseline (5 tasks), Q=6.00, p=0.111 under the safety prompt (8 tasks); gate-pass indicator Q=12.20 and 14.00 (p=0.001), 44 of 45 cells equal to the registered cell; pointer to Table 19 | `out/a2/cochran_all.csv`, `out/f1/f4_naive_vs_gatepass_cells.csv` |
| VI-D | 1455-1467 | A2, V3 | "uncorrected p = 0.013 and 0.020; Holm-corrected over the two tests, p = 0.026"; "stands at the binary level that it was registered for" | exact p 0.014 and 0.019; gate-passing at Q4_K_M undefined (6 tasks) and Q=4.00, p=0.333; gate-pass Q=8.67 and 6.22 (p=0.014 and 0.059); the registered test rejects at either level, no computable gate-passing test detects a difference | `out/a2/cochran_all.csv` |
| VI-D | 1474 | A2 | Table 16: Q and asymptotic p on the naive basis only | Table 17: naive, gate-passing and gate-pass indicator, tasks, Q and exact p for all six lines, naive rates | `out/f1/f5_table16_all_bases.csv`, `out/a2/cochran_all.csv` |
| VII-A | 1653-1658 | V2, V4, A1 (consistency) | "89.2% to 98.8% of baseline URScript generations ... only 1 to 9 programs in 135" | "of the gate-passing baseline outputs"; "1 to 9 of the 135 baseline generations (0 to 3 of the 130 stored outputs under the structural gate)"; one sentence that the comparator qualifies the comparison with general-purpose code | `out/a1/table13_with_translator.csv` |
| VII-D | 1781-1784 | A2 | "the binary heterogeneity survives at either uniform level" | "the registered test, a heterogeneity in syntactic commitment that no computable test on the gate-passing outputs reproduces, still rejects homogeneity at either uniform level" | `out/a2/cochran_all.csv` |
| Conclusion | 1854 | consistency with the abstract | "second of two identical generation runs" | "second of two generation runs" (the abstract dropped "identical"; the Qwen programs differ between runs) | given (abstract) |
| Conclusion | 1857-1865 | A3, V2, V4 | "1 to 9 of 135 ... part of this rate is a property of the rule set, while generated code still produces roughly twice as many rule firings" | lexical 1 to 9 of the 135 generations, structural 0 to 3 of the 130 stored outputs; rate largely reflects the rule set and the prompt; lower bound is the missing TCP declaration; most of the firing excess comes from SM-1 and SM-2 | `out/a3/differences.csv`, `out/a1/table13_with_translator.csv` |
| Conclusion | 1866-1867 | consistency with the abstract | "Scored naively it lets invalid pseudo-code pass as safe" | "Scored naively, and to a lesser degree under the registered validity gate, it lets ..." (abstract wording; V1, V2) | given (abstract) |
| Appendix A | 1961-1962 | A2 | run agreement on "the cross-model statistics (12.60 and 20.00)" | also identical on the gate-passing and gate-pass bases and for every test of Table 19 | `out/a2/cochran_all.csv` (run1 rows), `out/f1/f4_cochran_four_conditions.csv`, `out/f1/f4_cochran_adversarial_subtypes.csv` |
| Appendix A | 1996-2008 | A2 | (none) | new paragraph and Table 19: every registered cross-model test (baseline, safety, seven strategies, final repair attempt) on three bases; 192 of 192 and 57 of 57 gate-passing outputs violate | `out/f1/f4_*.csv`, `out/r3/gate_passing_violating.csv` |
| `docs/OSF_PREREGISTRATION.md` | end | V6 | no Amendment 1 analysis text | new section with eight verbatim excerpts of Amendment 1 (cross-model test, prompt conditions, cell aggregation, invalid pseudo-code) and a note on what the mirror does not reproduce | `analysis/round3/s10_osf_mirror.py`, `out/r3/osf_amendment1_excerpt.md` |

## V3: the round-2 Holm family, recorded

The round-2 Table 6 and VI-D gave p = 0.004 for the baseline cross-model test,
"Holm-corrected". That value came from `scripts/mcnemar_analysis.py`
(`run_cross_model_cochran_q`), whose Holm family has four tests (`out/a2/vid_four_condition_family.csv`):

| Condition in the code | What it is | Q | p (asymptotic) | Holm, four tests | Reported in round 2 |
|---|---|---|---|---|---|
| baseline | E1 neutral prompt | 12.60 | 0.0018 | 0.0037 | yes (0.004) |
| safety | E1 safety-prompted | 20.00 | 4.5e-5 | 1.4e-4 | yes (below 0.001) |
| adversarial_any | the seven E2 strategies pooled by OR per task-model cell | 8.00 | 0.0183 | 0.0183 | no |
| watchdog | final E3 repair attempt | 22.93 | 1.0e-5 | 4.2e-5 | no |

Run 1 gives the same values. The last two tests were computed and stored only in the
gitignored `results/stats*/cochran_results.csv` and `hypothesis_report.md`; they appear in no
tracked file other than the code and its tests (`analysis/round3/out/f1/f4_locations.csv`).
The pooled adversarial condition is not a registered condition; the registration names one
condition per strategy. The family is closed in round 3: the paper reports no Holm
correction for the cross-model test, as registered ("reported with p-value, no correction"),
gives exact permutation p values, and lists every registered test, including one per
strategy and the final repair attempt, in Table 19.

## Places on the impact list that did not change

| Place | Reason it stays |
|---|---|
| I-C, contributions, line 169 (seven clauses "including the new cybersecurity clause 5.1.16") | still accurate: SM-1, SM-3, SM-5, SM-7 and DM-7 map to 5.1.16; the A1 point concerns SM-6 only and is stated in VI-A |
| I-C, line 157, findings sentence ("generated code fails the static checks almost universally") | describes the measured rate; the qualification by the comparator is in the abstract, VI-A, VII-A and the Conclusion. See open items |
| V-E, line 837 (the cross-model test "operates at the task level") | still accurate for all three bases |
| Table 7, frontier "49.2% under SM-6 only" | the frontier run is outside A1 to A4 and was not recomputed; see open items |
| Table 9 (first audit) | a single-rater audit; the choice of primary rater does not apply |
| VI-A, line 1030 (first audit: TCP and payload "genuinely absent before motion") | still true; A1 adds why they are absent |
| VI-A, first-audit restriction (77 of 83, 92.8%) | unchanged numbers; Table 6 now shows them as k/N |
| Table 12 (comparator, programs firing per check) | its counts are correct; the per-rule means it lacked are the new Table 13 |
| Table 15 (was 14, adversarial) | its rates and means are unchanged; the decomposition by check is in the text of VI-A and VI-B |
| Table 20 (was 18, rule-to-clause mapping) | SM-6 rows (5.1.14, 5.1.15) agree with the code and the traceability file; VI-A now cites the table for the A1 point |
| Figure 4 image | the comparison row is the minimal translator, which the caption now names; the figure was not regenerated |
| Appendix A, "the single substantive-safe program (the same cell)" | still true in both runs |

## Numeric-token comparison

`docs/review/round3/numeric_tokens.csv` (from `analysis/round3/s11_numeric_tokens.py`)
compares every numeric token of `paper/main_access.tex` and its \input'ed tables at `797aeb3`
with this commit, block by block: file, new and old lines, section, tokens removed, tokens
added, item code and reason. Tokens present on both sides of a block cancel, so rewrapped
text does not appear. 32 blocks change at least one number, and every block carries an item
code (none unassigned).

## Open items (TODO)

1. Response letter: written by the authors. It needs the table renumbering above, the
   relabelling of the cross-model test, the removal of the Holm correction (V3) and the
   OSF mirror update (V6).
2. B1 to B9: not received; untouched.
3. Frontier run (Table 7, "49.2% under SM-6 only"): the SM-6 split into set_tcp and
   set_payload was not computed for the frontier models. Whether the A1 explanation carries
   over is not established; no number was written for it.
4. The I-C findings sentence keeps "fails the static checks almost universally". It is true
   as a measurement; whether to add the qualification there as well is an author decision.
5. 04_supplementary of the 55984 upload is on the Windows machine only; the copies here
   contain no cross-model output, but the uploaded copies were not checked.
6. Rerunning the round-2 generators (`scripts/review/sensitivity_rescoring.py`,
   `precision_audit_analyze.py`, `quantization_summary.py`) would restore the round-2
   tables; the round-3 tables come from `analysis/round3/s09_tables.py`.
7. The paper grows from 30 to 32 pages.
