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
Outside the paper, the OSF mirror changed (V6), the round-2 table generators gained a guard,
and the generation files of the additional runs were added under `data/` (below).

## Commits

The history of the branch was rewritten once (below, "History rewrite"); the hashes are those
after the rewrite.

| Commit | Content |
|---|---|
| `07b7e1b` | analysis: round-3 computations (`analysis/round3/`) |
| `35757a8` | paper: abstract, Table 13 comparator, SM-6 lower bound (A1) |
| `50cfeaa` | paper: cross-model test as heterogeneity in syntactic commitment (A2) |
| `255e78b` | paper: per-rule decomposition, rater B primary (A3, A4; V5) |
| `145b438` | paper, docs: V1, V2, V4, V6 and consistency edits |
| `13a2bda` | docs: first version of this file and of the numeric-token comparison; Conclusion second finding; analysis scripts read the r2 paper and OSF mirror from git |
| `51e14a1` | paper: asymptotic Cochran p with exact p beside it; round-2 table numbers kept (per-rule means into Table 12, registered-test table to a new Appendix E as Table 21); I-C qualified; frontier SM-6 split computed |
| `08ad502` | scripts, analysis: guard for the round-2 table generators; `run_all.sh` reproducible from a fresh clone; README mapping of numbers to sources |
| `08e2aa2` | docs: this file and the numeric tokens updated for that correction round |
| `fb6fa57` | paper, data: SM-6 sentence on declared values; exact p first on the post hoc bases; generation files of every reported run in `data/`, masked; Appendix A sentence |
| `3becd2e` | analysis: neutral labels in the analysis files |
| this commit | docs: this file and the numeric tokens; README scope line |

`./scripts/run_tests.sh --fast` passed (919 tests) before each commit;
`analysis/round3/test_round3.py` has 15 tests, one of them for the generator guard.

## Builds and page count

| PDF | Pages |
|---|---|
| r2 (`797aeb3`), rebuilt from the commit | 30 |
| r2 highlighted PDF as uploaded (`docs/review/submission_v9/02_highlighted_pdf/`) | 30 |
| r3 clean (`paper/main_access.pdf`) | 32 |
| r3 highlighted (`paper/diff/main_access_diff.pdf`) | 33 |

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
| VI, Table 6 | 877-899 | V4, A4, A2 | rates as percentages; rater B and adjudicated rows unlabelled; cross-model row "p=0.004 ... Holm-corrected" | every rate as k/N with its denominator; rater B "(primary)", adjudicated "(sensitivity)"; cross-model row relabelled, p=0.002 / p<0.001 uncorrected with exact p beside it; gate-passing result marked post hoc: undefined / Q=6.00, exact p=0.111 (asymptotic 0.050), 8 tasks, 3 discordant; columns ragged-right and the value column widened (layout only) | `out/r3/table6_fractions.csv`, `out/a2/cochran_all.csv` |
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
| VI-A | 1272-1297 | A1 | (none) | new paragraph: all 74 flagged outputs lack set_tcp, 68 also set_payload; prompt gives payload, not TCP; safety prompt 68/83 to 14/108, Qwen 39/45 to 0/45, Qwen 45/45 still lack set_tcp, CodeLlama 18/18 and 10/10; translator min/pre 15 and 0 flagged; SM-6 checks only that a declaration exists, not its value, so with no offset in the prompt it can be met only by writing a value not given: 3 of 83 local and 45 of 120 frontier outputs declare one before the first motion command, and of the literal poses 0 of 3 local and 3 of 28 frontier match the task's offset; SM-6 anchored to 5.1.14/5.1.15, no 5.1.16 anchor; rater B 18/20, Wilson [69.9, 97.2] | `out/f1/f1_sm6_groups.csv`, `out/f1/f2_safety_vs_baseline.csv`, `out/r3/tcp_values_summary.csv`, `out/a1/prompt_fields.csv`, `out/a1/ir_tcp_payload.csv`, `out/a1/sm6_clause_map.md`, `out/a4/precision.csv` |
| VI-A, Table 13 | 1299 | A1, V4 | LLM columns only, "n=130 generations" | reference translation columns (flagged of 15 and mean, min. / pre.) for every gate and rule set; caption "130 stored outputs of the 135 generations" | `out/a1/table13_with_translator.csv` |
| VI-B | 1317-1324 | A3 | adversarial count uplift not decomposed | A8.4: SM-5 +8.11 (7.35 corrected), SM-1 -2.01; pooled: SM-5 +2.04 (1.89 corrected), SM-1 -0.95; uplift remains under corrected SM-5 (5.45 and 1.08) | `out/r3/count_differences.csv`, `out/r3/count_check.csv` |
| VI-D | 1406-1432 | A2, V3 | "cross-model homogeneity test ... 12.60 (p of 0.004) ... 20.00 (p below 0.001), with multiplicity correction applied within this additional family only" | heterogeneity in syntactic commitment; registered in the first amendment, exploratory, uncorrected; p=0.002 and below 0.001 (exact 0.001 and below 0.001); the gate-passing and gate-pass analyses named post hoc, with the exact p first because they rest on few discordant tasks; gate-passing: undefined at baseline (5 tasks), Q=6.00 under the safety prompt (8 tasks, 3 discordant; exact p=0.111, asymptotic 0.050), does not reject homogeneity; gate-pass indicator Q=12.20 and 14.00 (exact 0.001; asymptotic 0.002 and below 0.001); 44 of 45 cells equal to the registered cell; pointer to Table 21 in Appendix E | `out/a2/cochran_all.csv`, `out/f1/f4_naive_vs_gatepass_cells.csv` |
| VI-D | 1466-1480 | A2, V3 | "uncorrected p = 0.013 and 0.020; Holm-corrected over the two tests, p = 0.026"; "stands at the binary level that it was registered for" | p=0.013 and 0.020 (exact 0.014 and 0.019); gate-passing at Q4_K_M undefined (6 tasks) and Q=4.00 (exact p=0.333, asymptotic 0.135); gate-pass Q=8.67 and 6.22 (exact 0.014 and 0.059, asymptotic 0.013 and 0.045); the registered test rejects at either level, the gate-pass indicator in every case except the safety prompt at Q4_K_M, and the gate-passing safety test does not reject at either level, on three and two discordant tasks | `out/a2/cochran_all.csv` |
| VI-D, Table 16 | 1487 | A2 | Q and asymptotic p on the naive basis only | naive (Q, p, exact p), gate-passing (tasks, Q, exact p, asym. p) and gate-pass indicator (Q, exact p, asym. p) in separate columns; naive rates; the naive lines keep the round-2 p values (0.002, <0.001, 0.013, 0.020) | `out/f1/f5_table16_all_bases.csv`, `out/a2/cochran_all.csv` |
| VII-A | 1666-1671 | V2, V4, A1 (consistency) | "89.2% to 98.8% of baseline URScript generations ... only 1 to 9 programs in 135" | "of the gate-passing baseline outputs"; "1 to 9 of the 135 baseline generations (0 to 3 of the 130 stored outputs under the structural gate)"; one sentence that the comparator qualifies the comparison with general-purpose code | `out/a1/table13_with_translator.csv` |
| VII-D | 1794-1797 | A2 | "the binary heterogeneity survives at either uniform level" | "the registered test still rejects homogeneity at either uniform level, a heterogeneity in syntactic commitment, while on the gate-passing outputs the test does not reject homogeneity (exact p) and rests on two or three discordant tasks" | `out/a2/cochran_all.csv` |
| Conclusion | 1868 | consistency with the abstract | "second of two identical generation runs" | "second of two generation runs" | given (abstract) |
| Conclusion | 1871-1879 | A3, V2, V4 | "1 to 9 of 135 ... part of this rate is a property of the rule set, while generated code still produces roughly twice as many rule firings" | lexical 1 to 9 of the 135 generations, structural 0 to 3 of the 130 stored outputs; rate largely reflects the rule set and the prompt; lower bound is the missing TCP declaration; most of the firing excess comes from SM-1 and SM-2 | `out/a3/differences.csv`, `out/a1/table13_with_translator.csv` |
| Conclusion | 1880-1881 | consistency with the abstract | "Scored naively it lets invalid pseudo-code pass as safe" | "Scored naively, and to a lesser degree under the registered validity gate, it lets ..." | given (abstract) |
| Appendix A | 1950-1953 | generation files | "The generation logs and generated URScript files for every run ... are provided in the same package." | adds "with local file paths and the inference endpoint masked"; the files of every reported run are now under `data/` (below) | `out/r3/import_manifest.csv`, `out/r3/import_masking.csv` |
| Appendix A | 1976-1977 | A2 | run agreement on "the cross-model statistics (12.60 and 20.00)" | also identical on the gate-passing and gate-pass bases and for every test of Table 21 | `out/a2/cochran_all.csv` (run1 rows), `out/f1/f4_cochran_four_conditions.csv`, `out/f1/f4_cochran_adversarial_subtypes.csv` |
| Appendix A | 2011-2012 | A2 | (none) | one sentence: every registered cross-model test is reported in Appendix E | given |
| Appendix E (new) | 2182-2198 | A2 | (none) | paragraph and Table 21: every registered cross-model test (baseline, safety, seven strategies, final repair attempt) on three bases, naive with the asymptotic p first, the post hoc bases with the exact p first; 192 of 192 and 57 of 57 gate-passing outputs violate | `out/f1/f4_*.csv`, `out/r3/gate_passing_violating.csv` |
| `docs/OSF_PREREGISTRATION.md` | end | V6 | no Amendment 1 analysis text | new section with eight verbatim excerpts of Amendment 1 (cross-model test, prompt conditions, cell aggregation, invalid pseudo-code) and a note on what the mirror does not reproduce | `analysis/round3/s10_osf_mirror.py`, `out/r3/osf_amendment1_excerpt.md` |

## The cross-model p values

The registered statistic is Cochran's Q. On the naive basis, the registered test, the asymptotic
chi-square p with two degrees of freedom is given first and the exact permutation p
(`analysis/round3/common.py`, `cochran_exact`, full enumeration of the within-task arrangements)
beside it; the naive 12.60 and 20.00 lines keep their round-2 uncorrected p values (0.002 and below
0.001), as do the Q4_K_M lines (0.013 and 0.020). The gate-passing and gate-pass analyses are post
hoc and rest on few discordant tasks, so on them the exact p comes first and the asymptotic p
beside it (VI-D, Table 6, VII-D, Table 16, Table 21, Appendix E). The gate-passing safety-prompted
test does not reject homogeneity at the served levels (exact p 0.111, asymptotic 0.050; three
discordant tasks) or at Q4_K_M (exact 0.333, asymptotic 0.135; two discordant tasks); the
gate-pass indicator under the safety prompt at Q4_K_M gives exact p 0.059 (asymptotic 0.045).

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

## The SM-6 sentence and the declared TCP values

`analysis/round3/s15_tcp_values.py` (`out/r3/tcp_values_summary.csv`, `tcp_values_programs.csv`).
Of the 83 gate-passing local baseline outputs, 7 contain a `set_tcp` call and 3 place it before
the first motion command (Qwen2.5-Coder-32B, task T012, repetitions 1 to 3, each
`set_tcp(p[0,0,0,0,0,0])` against an offset of 0.14 m in the task representation); none of the
6 literal poses matches the offset. Of the 120 frontier outputs, 48 contain a `set_tcp` call and
45 place it before the first motion command; 28 of those 45 write a literal pose and 3 of the 28
match the offset. The VI-A sentence uses the counts before the first motion command, which is
what SM-6 reads. The count of 9 local programs mentioned when this item was raised is the number
of local outputs with no SM-6 firing, not of outputs that declare a TCP.

## Generation files of the additional runs (`data/`)

`analysis/round3/s13_import_runs.py` copied 1577 files of the runs that the paper reports from the
runner's output directory `results/` (outside version control) into `data/exploratory/` and
`data/quantization_controls/`: the single-task probe (`smoke_test`), the frontier runs and their
preliminary probes, the enriched-feedback probe, the two temperatures, the
constraint-representation control and representation probes with their preliminary checks, and
the quantization controls (`data/exploratory/README.md`, `data/quantization_controls/README.md`).
Two macOS `.DS_Store` files were skipped. Not copied: `e1_pilot_session12` and `pilot_we_t001`
(not reported in the paper) and the URSim telemetry of the live-execution pilot (not a
generation run).

Before copying, every file was scanned for keys and tokens (none found), IP addresses, home
directories, user names and endpoints. Masking replaced only the matched substring, never inside
a prompt sent to a model or a model response (the script stops in that case):

| File | `<HOME>` | `<USER>` | `<IDUN_ENDPOINT>` |
|---|---|---|---|
| `data/exploratory/temp_robustness/run_temp0p2.log` | 1 | 0 | 181 |
| `data/exploratory/temp_robustness/run_temp0p7.log` | 1 | 0 | 181 |

Every other file is byte-identical to its source (`out/r3/import_manifest.csv`, both SHA-256
values per file); `s13_import_runs.py --check` recomputes the masked copies from `results/` and
finds 0 differences. The address 192.168.1.100 occurs in one frontier program
(`T014_mistralai_Mistral-Large-3-675B-Instruct-2512-NVFP4_baseline_rep1.urscript`, in
`E1_frontier`, `E3_frontier` and `e4_template_probe`) and in the `raw_response` field of the
corresponding log records only: the model wrote it into its program as a camera address, it is
in no prompt, request or client-written field, and it is kept unchanged.

After the copy, all analyses were rerun: `run_all.sh` reproduces every committed output, and
`analysis/round3/s14_exploratory_check.py` recomputes from `data/` the frontier sensitivity table
behind Table 7 (CSV and LaTeX equal to `paper/tables/sensitivity_e1_frontier.*`), the two
representation-probe figures (text equal to `paper/figures/`) and the frontier summaries (equal
to `docs/frontier_spotcheck/`); `s12_frontier.py` reads the frontier run from `data/` and gives
the committed `out/f2/` files.

## Trace cleanup

`out/a3/reviewer_check.csv` labels the authors' derived values "authors" (was "user"); the
`s03_a3.py` docstring and the README script-table header ("Item", was "Brief item") are neutral,
and the README's first line no longer refers to the analysis report. A search of
`analysis/round3/` for "user", "brief", "Claude", "chat" and for "prompt" outside the prompts
sent to the models leaves:

- "users": the user links of the OSF API records in `out/osf/*.json` (public registration data);
- "user name" and the `USER` column: the masking description in `s13_import_runs.py` and
  `out/r3/import_masking.csv`;
- "Claude": the OSF registration's own text in `out/osf/*.json` ("commercial LLM APIs (Claude,
  GPT-4o, Grok)");
- "chat": none;
- "prompt": only the model prompts, the `prompt_security` fields of the task representation and
  the SM-7 prompt-marker rule.

The item labels F1 to F7 in `s07_followup1.py` and in the README refer to the analysis requests
of this round; the analysis report that described them is no longer on the branch.

## History rewrite

`analysis/round3/REPORT.md` was removed from the history of `review/access-r3` only. The branch
had never been pushed (no remote-tracking ref, no upstream, and `git ls-remote origin` lists
only `main`, `review/access-r1` and `review/access-r2`). `git-filter-repo` was not installed, so
the built-in `git filter-branch --index-filter 'git rm --cached --ignore-unmatch
analysis/round3/REPORT.md' -- 797aeb3..review/access-r3` was used, with a message filter that
drops "the analysis report" from the first commit message; its `refs/original` backup ref was
deleted afterwards. Checks: no commit reachable from any ref touches the file; each of the 11
rewritten commits has the same patch as before apart from the file (same `git patch-id`); the
final tree differs from the old one only by the file. Old to new hashes: `fe24baf` `07b7e1b`,
`6ad9b1d` `35757a8`, `9cfb029` `50cfeaa`, `1d35ec2` `255e78b`, `1dca8dd` `145b438`, `f05c68a`
`13a2bda`, `5c2dbac` `51e14a1`, `c4f1ac1` `08ad502`, `08e7bfb` `08e2aa2`, `bf749a8` `fb6fa57`,
`b0e6d43` `3becd2e`. A bundle of the branch before the rewrite and a copy of the report are kept
outside version control in `r2workdrive/`.

## Round-2 table generators

The round-2 generators of the four tables that `s09_tables.py` now writes
(`scripts/review/comparator_urscript.py`, `sensitivity_rescoring.py`,
`precision_audit_analyze.py`, `quantization_summary.py`) write their `.tex` through
`scripts/review/round3_guard.py`. When the target's first line names
`analysis/round3/s09_tables.py`, the round-2 version is written to `<name>.r2.tex` beside it
and a notice is printed; any other target is written as before. Checked by running the four
generators into a copy of `paper/tables/`: the four round-3 tables stayed byte-identical, and
three of the `.r2.tex` files equal the r2 tables. The fourth, `comparator_summary.r2.tex`, lacks
the two vendor rows because the comparator generator was run without its vendor input; that
concerns the invocation, not the guard. `analysis/round3/test_round3.py` tests the guard.

## Replication from a fresh clone

`git clone` of the local repository, `git checkout review/access-r3`, `bash
analysis/round3/run_all.sh`: exit 0, the five round-3 tables equal `paper/tables/`, the
exploratory check passes, 15 tests pass, and `git status` shows no modified file, so every output
under `analysis/round3/out/` was recomputed to the committed bytes. Two steps need inputs outside
version control and print `SKIP` in a fresh clone, keeping their committed outputs: the
comparison of `data/` with the `results/` copies (`s00`, `s13 --check`) and the search of
`results/` and of the earlier submission packages for F4 (`s07`). The frontier analysis
(`s12`), the declared-TCP analysis (`s15`) and the exploratory check (`s14`) run from `data/`.

## Supplementary package

`docs/review/submission_v10/04_supplementary/round3_analysis.zip` (592356 bytes, SHA-256
`32485ba7f19678a2531d8bd9331fc29ae857e82c130b2defc813a2aa853208f7`): the 136 tracked files under
`analysis/round3/`, the same files as on the branch, built deterministically from the committed
files. Member hashes are in `docs/review/submission_v10/MANIFEST.md`; the draft text for the
portal's supplementary description is in `docs/review/submission_v10/portal_notes.md`. The
package folder is gitignored like the earlier ones.

## Places on the impact list that did not change

| Place | Reason it stays |
|---|---|
| Acknowledgment | unchanged; it says nothing about the use of AI tools (quoted in full in the open items) |
| I-C, contributions, line 171 (seven clauses "including the new cybersecurity clause 5.1.16") | still accurate: SM-1, SM-3, SM-5, SM-7 and DM-7 map to 5.1.16; the A1 point concerns SM-6 only and is stated in VI-A |
| V-E, line 839 (the cross-model test "operates at the task level") | still accurate for all three bases |
| Table 7, frontier "49.2% under SM-6 only" | computed (`s12_frontier.py`, `out/f2/`): every flagged frontier output lacks `set_tcp` and none lacks only `set_payload`, but 45 of 120 declare a TCP the prompt does not give; the VI-A sentence now covers both runs, so Table 7 itself needs no change |
| Table 9 (first audit) | a single-rater audit; the choice of primary rater does not apply |
| VI-A, line 1032 (first audit: TCP and payload "genuinely absent before motion") | still true; A1 adds why they are absent |
| VI-A, first-audit restriction (77 of 83, 92.8%) | unchanged numbers; Table 6 now shows them as k/N |
| Table 14 (adversarial) | its rates and means are unchanged; the decomposition by check is in the text of VI-A and VI-B |
| Table 18 (rule-to-clause mapping) | SM-6 rows (5.1.14, 5.1.15) agree with the code and the traceability file; VI-A now cites the table for the A1 point |
| Figure 4 image | the comparison row is the minimal translator, which the caption now names; the figure was not regenerated |
| Appendix A, "the single substantive-safe program (the same cell)" | still true in both runs |

## Numeric-token comparison

`docs/review/round3/numeric_tokens.csv` (from `analysis/round3/s11_numeric_tokens.py`)
compares every numeric token of `paper/main_access.tex` and its \input'ed tables at `797aeb3`
with this commit, block by block: file, new and old lines, section, tokens removed, tokens
added, item code and reason. Tokens present on both sides of a block cancel, so rewrapped
text does not appear. 33 blocks change at least one number, and every block carries an item
code (none unassigned).

## Open items (TODO)

1. Response letter: written by the authors. It needs the relabelling of the cross-model test,
   the removal of the Holm correction (V3), the p-value order (asymptotic first for the
   registered test, exact first for the post hoc bases), the new Appendix E and Table 21, the
   lower block of Table 12, the generation files now in `data/` and the OSF mirror update (V6).
2. B1 to B9: not received; untouched.
3. Acknowledgment, quoted in full, unchanged: "\textit{Formal Adversarial Testing of
   LLM-Generated Code for Industrial Robots} project has received funding from the European
   Union, via the oc4-2025-TES-02-44 issued and implemented by the ENFIELD project, under the
   grant agreement No 101120657." and "The preregistration, analysis plan, and amendment
   history for this study are available on the Open Science Framework (DOI
   10.17605/OSF.IO/VE5M2). The complete replication package, including the rule catalogue, task
   specifications, generation logs, and analysis scripts, is released under the Apache 2.0
   license." Nothing in it, or elsewhere in the paper, concerns the use of AI tools.
4. 04_supplementary of the 55984 upload (audit_v2.zip, confirmatory_run_comparison.zip) is on
   the Windows machine only; whether those copies are unchanged from the previous upload is to
   be checked there, and the portal note leaves their description as TODO.
5. The paper grows from 30 to 32 pages (the highlighted PDF from 30 to 33).
6. `results/stats*` (the gitignored Cochran output files cited for V3) and the earlier
   submission packages stay outside version control.
