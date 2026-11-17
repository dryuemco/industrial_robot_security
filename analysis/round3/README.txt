Analyses for the revision of Access-2026-55984

This folder recomputes every number that the revised manuscript adds or changes, from the data
committed in this repository. It covers Reviewer 1's items A1 to A4 and B1 to B9 and the other
numbers of the revised text. The scripts write under analysis/round3/out/, with two exceptions:
s09_tables.py writes paper/tables/ when it is run without --out-dir, and s13_import_runs.py
wrote the run copies in data/ once (with --check it only reads).

    bash analysis/round3/run_all.sh                # uses the cached OSF responses in out/osf/
    bash analysis/round3/run_all.sh --refresh-osf  # re-reads api.osf.io (read-only, no login)

A full run takes under a minute. It needs the repository's own Python packages (pandas, numpy,
scipy, statsmodels, pytest). No model is called, and the network is used only for the optional
OSF refresh.

Scripts

Each entry gives the script, then the review item or topic it covers, then its main outputs.

s00_provenance.py
    data source, gate definition
    out/provenance_*.csv, out/gate_equivalence.csv

s01_a1.py
    A1.1 to A1.5
    out/a1/

s02_a2.py
    A2.0 to A2.3, run 1
    out/a2/cochran_all.csv, verify_counts.csv, table16_check.csv

s06_osf.py
    A2.4
    out/osf/*.json, out/a2/osf_search.txt

s03_a3.py
    A3.1 to A3.5
    out/a3/

s04_a4.py
    A4.1 to A4.3
    out/a4/

s07_sm6_and_crossmodel.py
    SM-6 split by missing call, safety prompt, SM-6 audit by call, registered cross-model test
    and Table 16 on three bases, single safe output, SM6-only safe programs (runs after
    s02_a2.py and s06_osf.py)
    out/f1/

s08_paper_numbers.py
    per-rule decomposition of the Table 14 count contrasts, SM-2 per program
    out/r3/

s09_tables.py
    the paper tables written from out/ (see Paper tables below)
    paper/tables/ or --out-dir

s10_osf_mirror.py
    verbatim Amendment 1 excerpts for docs/OSF_PREREGISTRATION.md
    out/r3/osf_amendment1_excerpt.txt

s12_frontier.py
    SM-6 split and prompt check for the frontier run of Table 7
    out/f2/

s13_import_runs.py
    copies the reported runs from results/ into data/ with masking; --check recomputes and
    compares
    data/exploratory/, data/quantization_controls/, out/r3/import_*.csv

s14_exploratory_check.py
    recomputes the frontier sensitivity table, the representation figures and the frontier
    summaries from data/
    out/r3/exploratory_check.csv

s15_tcp_values.py
    set_tcp declarations and their values against the task's TCP offset, local and frontier
    out/r3/tcp_values_*.csv

s17_registered_h4_h6.py
    H4 to H6 in the registered form (H6 after one retry and over three); corrected SM-5 firings
    by kind; outputs over the registered token budget
    out/r3/registered_h4_h6.csv, out/r3/sm5_corrected_kinds.csv, out/r3/token_budget.csv

s18_e5_enforcement.py
    the representation probe read by function: class of every explicit-form output, commanded
    speed and breach of all 238 valid outputs, trend and paired tests
    out/e5/

s19_safety_and_repair.py
    binary rates under the safety prompt, repair under the preamble check and the programs that
    clear it, the input-validation clearances line by line, conservative interval, fixed-weight
    fall, excess by check against the translation with the preamble, registered sensitivity
    analyses, cross-model test after one retry
    out/r3/, out/a3/excess_by_check.csv

s16_b_items.py
    B1, B2, B3, B5, B7: outputs flagged per check by gate status, severity per check, the
    frontier models, cells per experiment family, breaches in the control probe
    out/b/

test_round3.py
    checks of the statistics helpers
    pytest

common.py holds the paths and the shared statistics (Wilson, Clopper-Pearson, Cohen's kappa,
exact Cochran Q, Holm). Gates, rule sets, the translator and the per-program scoring are
imported from scripts/review/sensitivity_rescoring.py and scripts/review/comparator_urscript.py,
so the definitions are the paper's own. out/flags.json records the consistency checks made on
the way (data identity, translator counts, reproduced tables).

The scripts s00 to s07 quote the paper and the OSF mirror as submitted for Access-2026-55984
(commit 3089121, common.R2_COMMIT). They read both from git into a temporary directory, so later
edits of paper/ and docs/OSF_PREREGISTRATION.md do not change their outputs.

Paper tables

s09_tables.py writes five tables of the paper, Tables 10, 12, 13, 16 and 21
(precision_audit_v2.tex, comparator_summary.tex, sensitivity_e1_baseline.tex,
quantization_summary.tex, crossmodel_registered.tex), from the CSV files under out/. Tables 15
and 17 are written by their own generators (scripts/review/e3_regimes.py,
scripts/review/complexity_table.py), and the other tables are part of paper/main_access.tex.
s09_tables.py is run as

    python3 analysis/round3/s09_tables.py                       # writes paper/tables/
    python3 analysis/round3/s09_tables.py --out-dir <directory>  # anywhere else

run_all.sh writes the five tables to out/tables/ and checks that they equal the files in
paper/tables/. Four other scripts can also write the first four tables
(scripts/review/comparator_urscript.py, sensitivity_rescoring.py, precision_audit_analyze.py,
quantization_summary.py). They write through scripts/review/round3_guard.py: when the first line
of the target names s09_tables.py, they write their own version to <name>.r2.tex beside it and
leave the table in place.

Where each number comes from

Sections and tables are those of the revised manuscript, and every path is under
analysis/round3/. Each entry gives the place in the paper, then the numbers, the script and the
output files.

Abstract; Table 13; VI-A sensitivity paragraph; VII-A; Conclusion
    89.2% to 98.8%; 1 to 9 of 135; 0 to 3 of 130 under the structural gate; reference
    translation flagged of 15 and mean, minimal and with preamble
    s01_a1.py
    out/a1/table13_with_translator.csv

Abstract; I-C; VI-A paragraph on the lower bound
    the prompt gives the payload and never the TCP offset; the task representation holds both
    s01_a1.py
    out/a1/prompt_fields.csv, out/a1/prompt_log_evidence.json, out/a1/ir_tcp_payload.csv

VI-A paragraph on the lower bound
    74 flagged, all without set_tcp, 68 also without set_payload; 68/83 to 14/108; Qwen 39/45 to
    0/45 and 45/45 without set_tcp; CodeLlama 18/18 and 10/10
    s07_sm6_and_crossmodel.py
    out/f1/f1_sm6_groups.csv, out/f1/f2_safety_vs_baseline.csv

VI-A paragraph on the lower bound
    SM-6 clauses 5.1.14 and 5.1.15
    s01_a1.py
    out/a1/sm6_clause_map.txt

VI-A paragraph on the lower bound
    3 of 83 local and 45 of 120 frontier outputs declare a TCP before the first motion call; 0
    of 3 and 3 of 28 literal poses match the task's offset
    s15_tcp_values.py
    out/r3/tcp_values_summary.csv

VI-A; Table 10
    rater B and adjudicated precision, Wilson and Clopper-Pearson intervals, agreement, kappa,
    adjudicated = rater A on 100 of 100
    s04_a4.py
    out/a4/precision.csv

VI-A, SM-2 unit
    111 firings on 82 programs, 1.34 and 0.99 per program
    s08_paper_numbers.py
    out/r3/sm2_per_program.csv

VI-A, SM-2 unit
    11 program-level items (10 and 10 genuine), 9 call-site items (8 and 0)
    s04_a4.py
    out/a4/sm2_by_location.csv

VI-A, single safe output
    commented stopl(), structural gate, substring motion flag
    s07_sm6_and_crossmodel.py, s04_a4.py
    out/f1/f6_t009.txt, out/a4/gate_calls.csv

VI-A, safety-prompt contrast by check
    3.98, 2.32, 0.94, 0.32, 0.24, 0.15; 3.56 and 0.11 corrected
    s08_paper_numbers.py
    out/r3/count_differences.csv

VI-A, comparator; Table 12 lower block; Conclusion
    per-rule means and differences: 10.37, 5.73, 4.64, 4.49, 5.88, 6.84, 9.84, 3.00, 2.35, 2.99,
    0.35, 0.29
    s03_a3.py, s08_paper_numbers.py
    out/a3/per_rule_means.csv, out/a3/differences.csv, out/r3/first_audit_rules.csv

VI-A, comparator
    preamble translation fires SM-2 on 15 of 15
    s03_a3.py
    out/a3/never_fires.txt

VI-A, sensitivity paragraph
    six of the nine SM6-only safe outputs are pseudo-code
    s07_sm6_and_crossmodel.py
    out/f1/f7_sm6_only_safe.csv

VI-B, adversarial contrast by check
    8.11, 7.35, 2.01, 2.04, 1.89, 0.95, 5.45, 1.08
    s08_paper_numbers.py
    out/r3/count_differences.csv, out/r3/count_check.csv

Table 6
    every rate as k/N
    s08_paper_numbers.py
    out/r3/table6_fractions.csv

Table 6; VI-D; VII-D; Table 16
    cross-model Q, asymptotic and exact p, tasks, on the naive, gate-passing and gate-pass
    bases, all three quantization variants, both runs
    s02_a2.py
    out/a2/cochran_all.csv

VI-D
    44 of 45 baseline cells equal on the naive and gate-pass bases
    s07_sm6_and_crossmodel.py
    out/f1/f4_naive_vs_gatepass_cells.csv

Appendix A; Appendix E; Table 21
    every registered cross-model test on three bases, run 1 = run 2
    s07_sm6_and_crossmodel.py
    out/f1/f4_cochran_four_conditions.csv, out/f1/f4_cochran_adversarial_subtypes.csv

Appendix E
    192 of 192 and 57 of 57 gate-passing outputs violate
    s08_paper_numbers.py
    out/r3/gate_passing_violating.csv

Appendix A, generation files of every run (masking)
    2 files masked, 1577 copied
    s13_import_runs.py
    out/r3/import_masking.csv, out/r3/import_manifest.csv

Table 7 and the representation figures, recomputed from data/
    equal to the committed table, figures and summaries
    s14_exploratory_check.py
    out/r3/exploratory_check.csv

Tables 10, 12, 13, 16, 21 (LaTeX)
    the table files in paper/tables/
    s09_tables.py
    out/tables/ (checked against paper/tables/ by run_all.sh)

docs/OSF_PREREGISTRATION.md
    Amendment 1 excerpts, verbatim
    s10_osf_mirror.py
    out/r3/osf_amendment1_excerpt.txt

Table 8 caption (B1)
    83 baseline and 108 safety-prompted gate-passing outputs; no check fires on an output that
    fails the gate
    s16_b_items.py
    out/b/b1_perrule_by_gate.csv

V-D, severity (B2)
    0.6 and 0.4, 0.7 and 0.5, 0.9, 0.5, 0.6, 0.95; the hardcoded-value check computed and capped
    at 1
    s16_b_items.py
    out/b/b2_severity.csv

V-A, frontier models (B3)
    the four served identifiers; calls between 10 and 16 June 2026
    s16_b_items.py
    out/b/b3_frontier_models.csv

V-E, VI, Table 6 caption and Appendix A, registered form of H4 to H6
    H4 15, 9 and 6 of 15 cells with Clopper-Pearson intervals and Holm p; H5 largest increase
    +0.13 [-0.19, 0.42], one-sided p >= 0.997; H6 after one retry 0%, 55.6%, 50.0% (Holm 1.000,
    0.375, 0.500), over three retries 88.9% (Holm 0.023); 23 of 924 outputs longer than 1024
    tokens
    s17_registered_h4_h6.py
    out/r3/registered_h4_h6.csv, out/r3/token_budget.csv

VI-A and VI-B, corrected SM-5 by kind
    211 of 248 TCP speeds at baseline; speed and acceleration literals under the strategies
    s17_registered_h4_h6.py
    out/r3/sm5_corrected_kinds.csv

Table 5 (B5)
    cells 90, 315 and 45 (450); 135 repair trajectories
    s16_b_items.py
    out/b/b5_cells.csv

VI-E, control probe (B7)
    2 breaches in 1 of the 179 programs stored from 180 edit generations; 70 breaches in 11 of
    the 60 from-scratch programs
    s16_b_items.py
    out/b/b7_breaches.csv

Abstract; I-C; Table 7; VI-E; VII-A; Figures 6 and 7 (B6)
    enforcement of the cap kept in 89 of 119 explicit-form programs (20 rewritten) and lost in
    30 (26 without the construct, 2 with the supplied constant but no clamp or guard, 1
    partial, 1 raised); 29 of the 30 within the cap; lost by model 0/30, 15/30, 13/29, 2/30; breaches 9/60, 1/59, 0/59, 1/60;
    Cochran-Armitage 3.44 (p = 0.0006); Cochran Q 20.42 over 58 pairs (exact p = 0.0002)
    s18_e5_enforcement.py
    out/e5/e5_enforcement.csv, out/e5/e5_summary.csv, out/e5/e5_tests.csv

VI-A, safety prompt
    108 gate-passing and 101 violating (93.5%), 101 of 135 (74.8%), against 98.8% and 60.7% at
    baseline; seven outputs without a firing, all CodeLlama-34B, all failing the structural gate
    s19_safety_and_repair.py
    out/r3/safety_binary.csv, out/r3/safety_clean_outputs.csv

VI-A, comparator (A3)
    against the translation with the preamble: SM-1 and SM-2 2.49, SM-5 2.99, SM-6 1.71, SM-4
    -0.35, total 6.84
    s19_safety_and_repair.py
    out/a3/excess_by_check.csv

V-E, VI-A, Table 6
    conservative interval [46%, 74%] (Wilson, 82 of 135 with n = 45)
    s19_safety_and_repair.py
    out/r3/conservative_interval.csv

VI-E, repair across scale (A4)
    on the 135 local trajectories, at the last gate-passing attempt, the input-validation check
    clears in 15 of the 54 trajectories it flags (28%), the hardcoded-value check in 2 of 54,
    the error-handling, unusual-conditions and preamble checks in none; the 15 are
    Qwen2.5-Coder-32B, every flagged line of their first output carries a= and v= after a nested
    call, and no line of their first or last output begins with if, while or assert
    s19_safety_and_repair.py
    out/r3/repair_by_check.csv, out/r3/repair_sm1_cleared.csv

VI-E, repair under the preamble check alone (A4)
    8 of the 29 flagged frontier trajectories and none of the 78 local ones end on a
    gate-passing output without the SM-6 firing; each of the 8 final programs has set_tcp and
    set_payload before its first motion command and still fires other checks
    s19_safety_and_repair.py
    out/r3/repair_sm6.csv, out/r3/repair_sm6_ends.csv

Appendix A, registered sensitivity analyses
    no refusal; with invalid outputs excluded 15, 10 and 6 baseline cells, 4 to 15 cells per
    strategy, 15, 3 and 3 matched cells after one retry; H4 also rejects for CodeLlama-34B (6 of
    6 cells), H5 and H6 unchanged
    s19_safety_and_repair.py
    out/r3/registered_h4_h6_sensitivity.csv

VI-B, ceiling
    CodeLlama-34B baseline 6 of 15 naive cells, largest increase +0.07 (7 of 15 under context
    overflow)
    s17_registered_h4_h6.py
    out/r3/registered_h4_h6.csv

Table 21; Appendix E (A2)
    repair loop after one retry: Q = 17.73, rates 100/27/20; 63 of 63 gate-passing outputs
    violate after one retry
    s19_safety_and_repair.py, s09_tables.py
    out/r3/crossmodel_one_retry.csv, out/r3/e3_endpoints.csv

Running from a fresh clone

Some inputs are outside version control: the results/ copies of the runs, which s00 and s13
--check compare with data/. When they are absent, those steps print SKIP and leave their
committed outputs as they are. Every other output is recomputed from tracked files and equals
the committed version byte for byte.
