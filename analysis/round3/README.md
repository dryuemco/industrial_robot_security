# Round-3 analyses for Access-2026-55984 (Reviewer 1, A1 to A4, and Follow-up 1)

Recomputes everything in `REPORT.md` from committed inputs. Nothing outside
`analysis/round3/out/` is written; the paper, the existing scripts and the data are only read.

```bash
bash analysis/round3/run_all.sh                # uses the cached OSF responses in out/osf/
bash analysis/round3/run_all.sh --refresh-osf  # re-reads api.osf.io (read-only, no login)
```

Runtime is under a minute. Requirements are the repository's own (python3 with pandas, numpy,
scipy, statsmodels, pytest). No model is called and no network is used except the optional
OSF refresh.

| Script | Brief item | Main outputs |
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
| `s12_frontier.py` | SM-6 split and prompt check for the frontier run of Table 7 (needs `results/E1_frontier/`, outside version control) | `out/f2/` |
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
The round-2 generators of the first three tables (`scripts/review/sensitivity_rescoring.py`,
`precision_audit_analyze.py`, `quantization_summary.py`) are unchanged; rerunning them would
restore the round-2 versions.
