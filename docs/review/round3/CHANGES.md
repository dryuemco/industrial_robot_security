# Round 3 (Access-2026-55984): changes to the paper

Branch `review/access-r3`, started from `review/access-r2` at `797aeb3` (the source of the
Access-2026-55984 submission). The response letter is written separately. Items A1 to A4
are Reviewer 1's analyses; V1 to V6 are corrections reported by the authors without a
reviewer request. B1 to B9 have not arrived and nothing here addresses them.

Every new number in the paper comes from a file under `analysis/round3/out/`; the source
column of the table below names it, and `analysis/round3/README.md` maps every new number and
table to its script and output file. The tables changed or added in this round are written
by `analysis/round3/s09_tables.py` from those files (`analysis/round3/run_all.sh` checks that
the files in `paper/tables/` equal its output). No data or preregistered scoring was changed.
Outside the paper, the OSF mirror changed (V6) and the round-2 table generators gained a
guard (below).

## Commits

| Commit | Content |
|---|---|
| `fe24baf` | analysis: round-3 computations (`analysis/round3/`) |
| `6ad9b1d` | paper: abstract, Table 13 comparator, SM-6 lower bound (A1) |
| `9cfb029` | paper: cross-model test as heterogeneity in syntactic commitment (A2) |
| `1d35ec2` | paper: per-rule decomposition, rater B primary (A3, A4; V5) |
| `1dca8dd` | paper, docs: V1, V2, V4, V6 and consistency edits |
| `f05c68a` | docs: first version of this file and of the numeric-token comparison; Conclusion second finding; analysis scripts read the r2 paper and OSF mirror from git |
| `5c2dbac` | paper: asymptotic Cochran p primary with exact p beside it; round-2 table numbers kept (per-rule means into Table 12, registered-test table to a new Appendix E as Table 21); I-C qualified; frontier SM-6 split computed (`s12_frontier.py`), paper unchanged for it |
| `c4f1ac1` | scripts, analysis: guard for the round-2 table generators; `run_all.sh` reproducible from a fresh clone; README mapping of numbers to sources |
| this commit | docs: this file and the numeric-token comparison updated |

`./scripts/run_tests.sh --fast` passed (919 tests) before each commit; `analysis/round3/test_round3.py`
has 15 tests, one of them for the generator guard.

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
10, 12, 13 and 16 and Figure 4; marked "New in this revision": Table 21. The clean build has
the same five overfull boxes as the r2 build, all in text this round did not change. Table 21,
a full-width float placed after the last table of the paper, appears at the top of the first
bibliography page.

## Table and figure numbers

Every table, figure, section, appendix and listing label of r2 has the same number in r3
(the `\newlabel` entries of the two `.aux` files, 69 labels compared). The only additions are
Appendix E (`app:crossmodel`) and Table 21 (`tab:crossmodel`). The reviewer's Table 13 and
Table 16 are still Tables 13 and 16.

## Changes

Line numbers refer to `paper/main_access.tex` in this commit.

| Section | Line | Item | Old | New | Source |
|---|---|---|---|---|---|
| Abstract | 47-68 | A1 | round-2 abstract ("1 to 9 of 135", "about twice as many rule firings") | the round-3 text, verbatim (248 words) | given |
| I-C, findings | 157-160 | A1 | "generated code fails the static checks almost universally" | adds that a non-LLM comparator shows much of this rate reflects the rule set and the prompt, and that its lower bound rests on a TCP declaration the prompt never specified, with a pointer to VI-A | `out/a1/table13_with_translator.csv`, `out/a1/prompt_fields.csv` |
| I-C, contributions | 176 | V4 | "1 of 135 at baseline" | "1 of the 135 baseline generations" | `out/r3/table6_fractions.csv` |
| V-E | 804-807 | A2 | "reported as an additional analysis only" | registered in the first amendment as exploratory, one per prompt condition, uncorrected p | `out/f1/f4_osf_cross_model.md` |
| VI, Table 6 | 877-899 | V4, A4, A2 | rates as percentages; rater B and adjudicated rows unlabelled; cross-model row "p=0.004 ... Holm-corrected" | every rate as k/N with its denominator; rater B "(primary)", adjudicated "(sensitivity)"; cross-model row relabelled, p=0.002 / p<0.001 uncorrected with exact p beside it, gate-passing result (undefined / Q=6.00, p=0.050, exact p=0.111); columns ragged-right and the value column widened (layout only) | `out/r3/table6_fractions.csv`, `out/a2/cochran_all.csv` |
| VI-A | 982-988 | V1 | "This single safe output is genuine code rather than an empty skeleton, since every gate-passing baseline generation contains motion commands." | registered 1/135 kept; the output passes the lexical gate only through a commented `stopl()`, writes motion without URScript call syntax, fails the structural gate; the motion-command flag is a substring search | `out/f1/f6_t009.md` |
| VI-A | 1098-1101 | A4 | adjudicated coincides with rater A; rater B "the conservative estimate" | adjudicated = rater A on all 100 items; rater B primary, adjudicated as sensitivity | `out/a4/precision.csv` |
| VI-A | 1106-1115 | A4 | (none) | SM-2 audit unit (program) and count unit (call site): 111 firings on 82 programs, 1.34 per program, 0.99 deduplicated; audit items 11 program-level (10 genuine for both raters) and 9 call-site (8 for A, 0 for B) | `out/r3/sm2_per_program.csv`, `out/a4/sm2_by_location.csv` |
| VI-A | 1134-1136 | A4 | "85% and 80% after adjudication (55% and 80% for rater B)" | rater B first, adjudicated in parentheses | `out/a4/precision.csv` |
| VI-A | 1140-1150 | A4, V4 | restrictions without status; "9 rather than 1 of 130", "1 of 130", "1 of 135" | "the primary estimate", "the sensitivity analysis"; "of the 130 stored baseline outputs", "82 of 83", "1 of the 135 baseline generations" | `out/r3/table6_fractions.csv` |
| VI-A, Table 10 | 1157 | A4 | rater A, rater B and adjudicated columns | rater B (primary), rater A = adjudicated (sensitivity), agreement and kappa; caption states the 100 of 100 identity | `out/a4/precision.csv` |
| VI-A | 1168-1172 | A3 | safety-prompt fall (10.37 to 6.39) not decomposed | fall of 3.98 by check: SM-1 2.32, SM-6 0.94, SM-5 0.32, SM-4 0.24, SM-2 0.15; 3.56 with corrected SM-5, where SM-5 rises by 0.11 | `out/r3/count_differences.csv` |
| VI-A, summary | 1188 | V4 | "1 of 135 (0.7%)" | "1 of the 135 baseline generations (0.7%)" | `out/r3/table6_fractions.csv` |
| VI-A, comparator | 1215-1229 | A3, V5 | "the count and the rule profile do separate generated code from the reference: ... 10.4 ... against 5.7 ... on which the reference never fires" | excess 4.64 of which SM-1 and SM-2 4.49 (rater B 45% and 50%); first-audit checks differ by 0.15 (5.88 against 5.73); corrected SM-5 excess 6.84, SM-1 and SM-2 4.49, remaining 2.35 from speed literals above the limit; "never fires" holds for the minimal translation, the preamble variant fires SM-2 on 15 of 15; points to the lower block of Table 12 | `out/a3/per_rule_means.csv`, `out/a3/differences.csv`, `out/r3/first_audit_rules.csv`, `out/a3/never_fires.md` |
| VI-A, Figure 4 caption | 1240 | V5 | "with the reference translation" | "with the minimal reference translation (without the preamble)"; the figure's comparison row is the minimal translator | `scripts/figures/fig_rule_heatmap.py` (row "Translator") |
| VI-A, Table 12 | 1246 | A3 | programs firing per check only | lower block: mean firings per program per check for both translations and the generated code, preregistered and corrected SM-5, and the differences LLM minus each translation; the upper block is unchanged | `out/a3/per_rule_means.csv` |
| VI-A, sensitivity | 1258-1264 | V2, V4 | "at most 9 of 130 scored outputs ... leaves at most three of them violation-free" | six of the nine are pseudo-code without URScript call syntax and fail the structural gate; structural gate gives 0 to 3 of the 130 stored outputs | `out/f1/f7_sm6_only_safe.csv`, `out/a1/table13_with_translator.csv` |
| VI-A | 1272-1293 | A1 | (none) | new paragraph: all 74 flagged outputs lack set_tcp, 68 also set_payload; prompt gives payload, not TCP; safety prompt 68/83 to 14/108, Qwen 39/45 to 0/45, Qwen 45/45 still lack set_tcp, CodeLlama 18/18 and 10/10; translator min/pre 15 and 0 flagged; what the bound shows and does not show; SM-6 anchored to 5.1.14/5.1.15, no 5.1.16 anchor; rater B 18/20, Wilson [69.9, 97.2] | `out/f1/f1_sm6_groups.csv`, `out/f1/f2_safety_vs_baseline.csv`, `out/a1/prompt_fields.csv`, `out/a1/ir_tcp_payload.csv`, `out/a1/sm6_clause_map.md`, `out/a4/precision.csv` |
| VI-A, Table 13 | 1295 | A1, V4 | LLM columns only, "n=130 generations" | reference translation columns (flagged of 15 and mean, min. / pre.) for every gate and rule set; caption "130 stored outputs of the 135 generations" | `out/a1/table13_with_translator.csv` |
| VI-B | 1313-1320 | A3 | adversarial count uplift not decomposed | A8.4: SM-5 +8.11 (7.35 corrected), SM-1 -2.01; pooled: SM-5 +2.04 (1.89 corrected), SM-1 -0.95; uplift remains under corrected SM-5 (5.45 and 1.08) | `out/r3/count_differences.csv`, `out/r3/count_check.csv` |
| VI-D | 1402-1427 | A2, V3 | "cross-model homogeneity test ... 12.60 (p of 0.004) ... 20.00 (p below 0.001), with multiplicity correction applied within this additional family only" | heterogeneity in syntactic commitment; registered in the first amendment, exploratory, uncorrected; p=0.002 and below 0.001 (exact 0.001 and below 0.001), with one sentence on why exact p is added; gate-passing: undefined at baseline (5 tasks), Q=6.00 under the safety prompt (8 tasks, 3 discordant; p=0.050, exact p=0.111); gate-pass indicator Q=12.20 and 14.00 (p=0.002 and below 0.001; exact 0.001); 44 of 45 cells equal to the registered cell; evidence for a violation difference among valid outputs described as weak; pointer to Table 21 in Appendix E | `out/a2/cochran_all.csv`, `out/f1/f4_naive_vs_gatepass_cells.csv` |
| VI-D | 1460-1474 | A2, V3 | "uncorrected p = 0.013 and 0.020; Holm-corrected over the two tests, p = 0.026"; "stands at the binary level that it was registered for" | p=0.013 and 0.020 (exact 0.014 and 0.019); gate-passing at Q4_K_M undefined (6 tasks) and Q=4.00, p=0.135 (exact 0.333); gate-pass Q=8.67 and 6.22 (p=0.013 and 0.045; exact 0.014 and 0.059); the registered test rejects at either level, the gate-pass indicator in every case (at Q4_K_M under the safety prompt only by the asymptotic p), gate-passing p=0.050 at the served levels and 0.135 at Q4_K_M | `out/a2/cochran_all.csv` |
| VI-D, Table 16 | 1481 | A2 | Q and asymptotic p on the naive basis only | naive, gate-passing and gate-pass indicator, each with Q, asymptotic p and exact p in separate columns; tasks for the gate-passing basis (15 on the other two); naive rates; the naive lines keep the round-2 p values (0.002, <0.001, 0.013, 0.020) | `out/f1/f5_table16_all_bases.csv`, `out/a2/cochran_all.csv` |
| VII-A | 1660-1665 | V2, V4, A1 (consistency) | "89.2% to 98.8% of baseline URScript generations ... only 1 to 9 programs in 135" | "of the gate-passing baseline outputs"; "1 to 9 of the 135 baseline generations (0 to 3 of the 130 stored outputs under the structural gate)"; one sentence that the comparator qualifies the comparison with general-purpose code | `out/a1/table13_with_translator.csv` |
| VII-D | 1788-1791 | A2 | "the binary heterogeneity survives at either uniform level" | "the registered test, a heterogeneity in syntactic commitment that the gate-passing outputs reproduce at most weakly, still rejects homogeneity at either uniform level" | `out/a2/cochran_all.csv` |
| Conclusion | 1861 | consistency with the abstract | "second of two identical generation runs" | "second of two generation runs" | given (abstract) |
| Conclusion | 1864-1872 | A3, V2, V4 | "1 to 9 of 135 ... part of this rate is a property of the rule set, while generated code still produces roughly twice as many rule firings" | lexical 1 to 9 of the 135 generations, structural 0 to 3 of the 130 stored outputs; rate largely reflects the rule set and the prompt; lower bound is the missing TCP declaration; most of the firing excess comes from SM-1 and SM-2 | `out/a3/differences.csv`, `out/a1/table13_with_translator.csv` |
| Conclusion | 1873-1874 | consistency with the abstract | "Scored naively it lets invalid pseudo-code pass as safe" | "Scored naively, and to a lesser degree under the registered validity gate, it lets ..." | given (abstract) |
| Appendix A | 1968-1969 | A2 | run agreement on "the cross-model statistics (12.60 and 20.00)" | also identical on the gate-passing and gate-pass bases and for every test of Table 21 | `out/a2/cochran_all.csv` (run1 rows), `out/f1/f4_cochran_four_conditions.csv`, `out/f1/f4_cochran_adversarial_subtypes.csv` |
| Appendix A | 2003-2004 | A2 | (none) | one sentence: every registered cross-model test is reported in Appendix E | given |
| Appendix E (new) | 2174-2188 | A2 | (none) | paragraph and Table 21: every registered cross-model test (baseline, safety, seven strategies, final repair attempt) on three bases with asymptotic and exact p; 192 of 192 and 57 of 57 gate-passing outputs violate | `out/f1/f4_*.csv`, `out/r3/gate_passing_violating.csv` |
| `docs/OSF_PREREGISTRATION.md` | end | V6 | no Amendment 1 analysis text | new section with eight verbatim excerpts of Amendment 1 (cross-model test, prompt conditions, cell aggregation, invalid pseudo-code) and a note on what the mirror does not reproduce | `analysis/round3/s10_osf_mirror.py`, `out/r3/osf_amendment1_excerpt.md` |

## The cross-model p values

The registered statistic is Cochran's Q; its asymptotic chi-square p with two degrees of freedom
is the primary, uncorrected p everywhere (VI-D, Table 6, VII-D, Table 16, Table 21). The exact
permutation p (full enumeration of the within-task arrangements, `analysis/round3/common.py`,
`cochran_exact`) is given beside it because several tests rest on few discordant tasks. The naive
12.60 and 20.00 lines keep their round-2 uncorrected p values (0.002 and below 0.001), as do the
Q4_K_M lines (0.013 and 0.020). Under the asymptotic p, the gate-passing safety-prompted test at the
served levels gives p = 0.050 (0.0498) on three discordant tasks, against an exact p of 0.111, and
the gate-pass indicator under the safety prompt at Q4_K_M gives p = 0.045 against an exact 0.059;
the text of VI-D and VII-D describes both cases with the two values.

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
and lists every registered test, one per strategy and the final repair attempt included, in
Table 21.

## Round-2 table generators

The round-2 generators of the four tables that `s09_tables.py` now writes
(`scripts/review/comparator_urscript.py`, `sensitivity_rescoring.py`,
`precision_audit_analyze.py`, `quantization_summary.py`) write their `.tex` through the new
`scripts/review/round3_guard.py`. When the target's first line names
`analysis/round3/s09_tables.py`, the round-2 version is written to `<name>.r2.tex` beside it
and a notice is printed; any other target is written as before. Checked by running the four
generators into a copy of `paper/tables/`: the four round-3 tables stayed byte-identical, and
three of the `.r2.tex` files equal the r2 tables. The fourth, `comparator_summary.r2.tex`, lacks
the two vendor rows because the comparator generator was run without its vendor input; that
concerns the invocation, not the guard. `analysis/round3/test_round3.py` tests the guard.

## Replication from a fresh clone

`git clone` of the local repository, `git checkout review/access-r3`, `bash
analysis/round3/run_all.sh`: exit 0, the five round-3 tables equal `paper/tables/`, 15 tests
pass, and `git status` shows no modified file, so every output under `analysis/round3/out/`
was recomputed to the committed bytes. Three steps need inputs outside version control and
print `SKIP` in a fresh clone, keeping their committed outputs: the comparison of
`data/confirmatory/` with the `results/` copies (`s00`), the search of `results/` and of the
earlier submission packages for F4 (`s07`), and the frontier analysis (`s12`, which reads
`results/E1_frontier/`). Two fixes made this hold: `s00` and `s07` no longer overwrite those
outputs when the inputs are absent, and the gate sources quoted in `out/f1/f6_t009.md` are read
at `797aeb3`, so the guard edit does not shift their line numbers.

## Supplementary package

`docs/review/submission_v10/04_supplementary/round3_analysis.zip` (510315 bytes, SHA-256
`76936739fc0c736f7756bc3230073c7ba78b23c95d534a3286f6f25b5cec2191`): the 128 tracked files under
`analysis/round3/` except `REPORT.md`, built deterministically from the committed files. Member
hashes are in `docs/review/submission_v10/MANIFEST.md`; the draft text for the portal's
supplementary description is in `docs/review/submission_v10/portal_notes.md`. The package
folder is gitignored like the earlier ones.

## Places on the impact list that did not change

| Place | Reason it stays |
|---|---|
| I-C, contributions, line 171 (seven clauses "including the new cybersecurity clause 5.1.16") | still accurate: SM-1, SM-3, SM-5, SM-7 and DM-7 map to 5.1.16; the A1 point concerns SM-6 only and is stated in VI-A |
| V-E, line 839 (the cross-model test "operates at the task level") | still accurate for all three bases |
| Table 7, frontier "49.2% under SM-6 only" | computed (`analysis/round3/s12_frontier.py`, `out/f2/`) and left unchanged, because the local pattern does not fully repeat; see below |
| Table 9 (first audit) | a single-rater audit; the choice of primary rater does not apply |
| VI-A, line 1032 (first audit: TCP and payload "genuinely absent before motion") | still true; A1 adds why they are absent |
| VI-A, first-audit restriction (77 of 83, 92.8%) | unchanged numbers; Table 6 now shows them as k/N |
| Table 14 (adversarial) | its rates and means are unchanged; the decomposition by check is in the text of VI-A and VI-B |
| Table 18 (rule-to-clause mapping) | SM-6 rows (5.1.14, 5.1.15) agree with the code and the traceability file; VI-A now cites the table for the A1 point |
| Figure 4 image | the comparison row is the minimal translator, which the caption now names; the figure was not regenerated |
| Appendix A, "the single substantive-safe program (the same cell)" | still true in both runs |

### Table 7: the frontier run

Source `analysis/round3/out/f2/` (`frontier_sm6_groups.csv`, `frontier_programs.csv`,
`frontier_prompts.json`). SM6-only flags 59 of the 120 frontier outputs (49.2%, reproducing
`paper/tables/sensitivity_e1_frontier.csv`). As in the local run, every flagged output lacks
`set_tcp` (52 lack only `set_tcp`, 7 lack both) and none lacks only `set_payload`, and the 120
logged requests (30 distinct prompts) mention a payload but never a TCP offset, `set_tcp` or a
tool offset; "TCP" occurs only in "Max TCP speed". Unlike the local run, 45 of the 120 outputs
declare `set_tcp` before the first motion call although the prompt gives no offset (Mistral
Large 18, gpt-oss 16, GLM 11, Devstral 0), and of the 30 with a literal pose only 3 match the
offset of the task representation. 16 Devstral outputs have no URScript motion call. The A1
sentence that the lower bound "does not show a weakness that the models could have avoided
with the inputs they were given" therefore does not carry over to the frontier models without
qualification, so no sentence was added at Table 7; the decision is left to the authors.

## Numeric-token comparison

`docs/review/round3/numeric_tokens.csv` (from `analysis/round3/s11_numeric_tokens.py`)
compares every numeric token of `paper/main_access.tex` and its \input'ed tables at `797aeb3`
with this commit, block by block: file, new and old lines, section, tokens removed, tokens
added, item code and reason. Tokens present on both sides of a block cancel, so rewrapped
text does not appear. 33 blocks change at least one number, and every block carries an item
code (none unassigned).

## Open items (TODO)

1. Response letter: written by the authors. It needs the relabelling of the cross-model test,
   the removal of the Holm correction (V3), the asymptotic and exact p, the new Appendix E and
   Table 21, the lower block of Table 12 and the OSF mirror update (V6).
2. B1 to B9: not received; untouched.
3. Before the push: `analysis/round3/REPORT.md` (Turkish summaries and wording about the
   working process) is in commit `fe24baf`; `out/a3/reviewer_check.csv` labels the authors'
   own derived values "user", as does the `s03_a3.py` docstring, and the README has a column
   headed "Brief item". These become public with the branch; what to do is the authors'
   decision.
4. Table 7 (frontier): see above; a sentence there needs the qualification that frontier
   models do declare a TCP, mostly with invented values.
5. The frontier run itself (`results/E1_frontier/`, with its code and prompt log) is outside
   version control, while Appendix A says the generated files of every run are in the
   replication package; `s12_frontier.py` cannot run from a fresh clone.
6. 04_supplementary of the 55984 upload (audit_v2.zip, confirmatory_run_comparison.zip) is on
   the Windows machine only; those copies were not checked.
7. The paper grows from 30 to 32 pages.
