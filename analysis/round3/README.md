# Round-3 analyses for Access-2026-55984 (Reviewer 1, A1 to A4, and Follow-up 1)

Recomputes every number of the round-3 revision from committed inputs. The scripts write under
`analysis/round3/out/`, except `s09_tables.py` (writes `paper/tables/` when run without
`--out-dir`) and `s13_import_runs.py` (wrote the run copies in `data/` once; `--check` only reads).

```bash
bash analysis/round3/run_all.sh                # uses the cached OSF responses in out/osf/
bash analysis/round3/run_all.sh --refresh-osf  # re-reads api.osf.io (read-only, no login)
```

Runtime is under a minute. Requirements are the repository's own (python3 with pandas, numpy,
scipy, statsmodels, pytest). No model is called and no network is used except the optional
OSF refresh.

| Script | Item | Main outputs |
|---|---|---|
| `s00_provenance.py` | data source, gate definition | `out/provenance_*.csv`, `out/gate_equivalence.csv` |
| `s01_a1.py` | A1.1 to A1.5 | `out/a1/` |
| `s02_a2.py` | A2.0 to A2.3, run 1 | `out/a2/cochran_all.csv`, `verify_counts.csv`, `table16_check.csv` |
| `s06_osf.py` | A2.4 | `out/osf/*.json`, `out/a2/osf_search.md` |
| `s03_a3.py` | A3.1 to A3.5 | `out/a3/` |
| `s04_a4.py` | A4.1 to A4.3 | `out/a4/` |
| `s05_impact.py` | impact list, 130 vs 135 | `out/impact/` |
| `s07_followup1.py` | Follow-up 1, F1 to F7 (runs after `s02_a2.py` and `s06_osf.py`) | `out/f1/` |
| `s08_paper_numbers.py` | per-rule decomposition of the Table 14 count contrasts, SM-2 per program | `out/r3/` |
| `s09_tables.py` | the round-3 paper tables | `paper/tables/` or `--out-dir` |
| `s10_osf_mirror.py` | verbatim Amendment 1 excerpts for `docs/OSF_PREREGISTRATION.md` | `out/r3/osf_amendment1_excerpt.md` |
| `s12_frontier.py` | SM-6 split and prompt check for the frontier run of Table 7 | `out/f2/` |
| `s13_import_runs.py` | copies the reported runs from `results/` into `data/` with masking; `--check` recomputes and compares | `data/exploratory/`, `data/quantization_controls/`, `out/r3/import_*.csv` |
| `s14_exploratory_check.py` | recomputes the frontier sensitivity table, the representation figures and the frontier summaries from `data/` | `out/r3/exploratory_check.csv` |
| `s15_tcp_values.py` | set_tcp declarations and their values against the task's TCP offset, local and frontier | `out/r3/tcp_values_*.csv` |
| `s11_numeric_tokens.py` | numeric tokens of the paper source, r2 (`797aeb3`) against the working tree | `docs/review/round3/numeric_tokens.csv` or `--out` |
| `test_round3.py` | checks of the statistics helpers | pytest |

`common.py` holds the paths and the shared statistics (Wilson, Clopper-Pearson, Cohen's kappa,
exact Cochran Q, Holm). Gates, rule sets, the translator and the per-program scoring are imported
from `scripts/review/sensitivity_rescoring.py` and `scripts/review/comparator_urscript.py`, so the
definitions are the paper's own. `out/flags.json` records the stop conditions checked on the way
(data identity, translator counts, reproduced tables).

The scripts `s00` to `s07` quote the paper and the OSF mirror as submitted for
Access-2026-55984 (commit `797aeb3`, `common.R2_COMMIT`), read from git into a temporary
directory, so the round-3 edits of `paper/` and `docs/OSF_PREREGISTRATION.md` do not change
their outputs.

## Round-3 paper tables

`s09_tables.py` writes the five paper tables changed or added in the round-3 revision
(`comparator_summary.tex`, `sensitivity_e1_baseline.tex`, `precision_audit_v2.tex`,
`quantization_summary.tex`, `crossmodel_registered.tex`) from the CSVs under `out/`:

```bash
python3 analysis/round3/s09_tables.py                       # writes paper/tables/
python3 analysis/round3/s09_tables.py --out-dir <directory>  # anywhere else
```

`run_all.sh` writes them to `out/tables/` and checks that they equal the files in `paper/tables/`.
The round-2 generators of the first four tables (`scripts/review/comparator_urscript.py`,
`sensitivity_rescoring.py`, `precision_audit_analyze.py`, `quantization_summary.py`) write
through `scripts/review/round3_guard.py`: when the target's first line names `s09_tables.py`,
they write their round-2 version to `<name>.r2.tex` beside it and leave the round-3 table in
place.

## Where each round-3 number in the paper comes from

Sections and tables are those of the round-3 manuscript (table numbers as in round 2; Table 21
and Appendix E are new). Every path is under `analysis/round3/`.

| Place in the paper | Numbers | Script | Output |
|---|---|---|---|
| Abstract; Table 13; VI-A sensitivity paragraph; VII-A; Conclusion | 89.2% to 98.8%; 1 to 9 of 135; 0 to 3 of 130 under the structural gate; reference translation flagged of 15 and mean, minimal and with preamble | `s01_a1.py` | `out/a1/table13_with_translator.csv` |
| Abstract; I-C; VI-A paragraph on the lower bound | the prompt gives the payload and never the TCP offset; the task representation holds both | `s01_a1.py` | `out/a1/prompt_fields.csv`, `out/a1/prompt_log_evidence.json`, `out/a1/ir_tcp_payload.csv` |
| VI-A paragraph on the lower bound | 74 flagged, all without set_tcp, 68 also without set_payload; 68/83 to 14/108; Qwen 39/45 to 0/45 and 45/45 without set_tcp; CodeLlama 18/18 and 10/10 | `s07_followup1.py` | `out/f1/f1_sm6_groups.csv`, `out/f1/f2_safety_vs_baseline.csv` |
| VI-A paragraph on the lower bound | SM-6 clauses 5.1.14 and 5.1.15 | `s01_a1.py` | `out/a1/sm6_clause_map.md` |
| VI-A paragraph on the lower bound | 3 of 83 local and 45 of 120 frontier outputs declare a TCP before the first motion call; 0 of 3 and 3 of 28 literal poses match the task's offset | `s15_tcp_values.py` | `out/r3/tcp_values_summary.csv` |
| VI-A; Table 10 | rater B and adjudicated precision, Wilson and Clopper-Pearson intervals, agreement, kappa, adjudicated = rater A on 100 of 100 | `s04_a4.py` | `out/a4/precision.csv` |
| VI-A, SM-2 unit | 111 firings on 82 programs, 1.34 and 0.99 per program | `s08_paper_numbers.py` | `out/r3/sm2_per_program.csv` |
| VI-A, SM-2 unit | 11 program-level items (10 and 10 genuine), 9 call-site items (8 and 0) | `s04_a4.py` | `out/a4/sm2_by_location.csv` |
| VI-A, single safe output | commented `stopl()`, structural gate, substring motion flag | `s07_followup1.py`, `s04_a4.py` | `out/f1/f6_t009.md`, `out/a4/gate_calls.csv` |
| VI-A, safety-prompt contrast by check | 3.98, 2.32, 0.94, 0.32, 0.24, 0.15; 3.56 and 0.11 corrected | `s08_paper_numbers.py` | `out/r3/count_differences.csv` |
| VI-A, comparator; Table 12 lower block; Conclusion | per-rule means and differences: 10.37, 5.73, 4.64, 4.49, 5.88, 6.84, 9.84, 3.00, 2.35, 2.99, 0.35, 0.29 | `s03_a3.py`, `s08_paper_numbers.py` | `out/a3/per_rule_means.csv`, `out/a3/differences.csv`, `out/r3/first_audit_rules.csv` |
| VI-A, comparator | preamble translation fires SM-2 on 15 of 15 | `s03_a3.py` | `out/a3/never_fires.md` |
| VI-A, sensitivity paragraph | six of the nine SM6-only safe outputs are pseudo-code | `s07_followup1.py` | `out/f1/f7_sm6_only_safe.csv` |
| VI-B, adversarial contrast by check | 8.11, 7.35, 2.01, 2.04, 1.89, 0.95, 5.45, 1.08 | `s08_paper_numbers.py` | `out/r3/count_differences.csv`, `out/r3/count_check.csv` |
| Table 6 | every rate as k/N | `s08_paper_numbers.py` | `out/r3/table6_fractions.csv` |
| Table 6; VI-D; VII-D; Table 16 | cross-model Q, asymptotic and exact p, tasks, on the naive, gate-passing and gate-pass bases, all three quantization variants, both runs | `s02_a2.py` | `out/a2/cochran_all.csv` |
| VI-D | 44 of 45 baseline cells equal on the naive and gate-pass bases | `s07_followup1.py` | `out/f1/f4_naive_vs_gatepass_cells.csv` |
| Appendix A; Appendix E; Table 21 | every registered cross-model test on three bases, run 1 = run 2 | `s07_followup1.py` | `out/f1/f4_cochran_four_conditions.csv`, `out/f1/f4_cochran_adversarial_subtypes.csv` |
| Appendix E | 192 of 192 and 57 of 57 gate-passing outputs violate | `s08_paper_numbers.py` | `out/r3/gate_passing_violating.csv` |
| Appendix A, generation files of every run (masking) | 2 files masked, 1577 copied | `s13_import_runs.py` | `out/r3/import_masking.csv`, `out/r3/import_manifest.csv` |
| Table 7 and the representation figures, recomputed from `data/` | equal to the committed table, figures and summaries | `s14_exploratory_check.py` | `out/r3/exploratory_check.csv` |
| Tables 10, 12, 13, 16, 21 (LaTeX) | the table files in `paper/tables/` | `s09_tables.py` | `out/tables/` (checked against `paper/tables/` by `run_all.sh`) |
| `docs/OSF_PREREGISTRATION.md` | Amendment 1 excerpts, verbatim | `s10_osf_mirror.py` | `out/r3/osf_amendment1_excerpt.md` |

## Running from a fresh clone

A few inputs are outside version control: the `results/` copies of the runs (compared with
`data/` by `s00` and by `s13 --check`) and the earlier submission packages (searched by `s07`
for item F4). When they are absent, those steps print `SKIP` and leave their committed
outputs as they are; every other output is recomputed from tracked files and equals the
committed version byte for byte.
