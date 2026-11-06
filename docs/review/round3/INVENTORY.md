# Round 3 (Access-2026-55984): where everything is

Branch `review/access-r3`, based on `review/access-r2` at `797aeb3` (the source of
Access-2026-55984). `main`, `review/access-r1` and `review/access-r2` are unchanged.

## In the repository

| Path | Content |
|---|---|
| `paper/main_access.tex`, `paper/tables/*.tex` | the revised manuscript source; figures in `paper/figures/` are unchanged |
| `paper/build_diff.sh`, `paper/mark_new_floats.py` | highlighted PDF against a given commit (unchanged; run with `797aeb3`) |
| `docs/review/submission_v10/` | the submission package: `01_main_manuscript/` (clean PDF, `source/`, `source.zip`), `02_highlighted_pdf/`, `03_response_to_reviewers/` (empty until the letter is written), `04_supplementary/round3_analysis.zip`, `MANIFEST.md`, `portal_notes.md` |
| `docs/review/round3/CHANGES.md` | every change by section, line and item (A1 to A4, V1 to V6), the cross-model p values, the round-2 Holm family (V3), the data import, the history rewrite, open items |
| `docs/review/round3/numeric_tokens.csv` | every numeric token of the r2 source against the r3 source, with item code and reason |
| `docs/review/round3/INVENTORY.md` | this file |
| `docs/OSF_PREREGISTRATION.md` | the OSF mirror, with the Amendment 1 analysis text added verbatim (V6) |
| `analysis/round3/` | the scripts and outputs behind every new number and table; `README.md` maps each number to its script and output file; `run_all.sh` reruns everything |
| `data/confirmatory/` | the two confirmatory runs (unchanged) |
| `data/exploratory/` | generation files of the additional runs the paper reports (single-task probe, frontier runs and probes, enriched-feedback probe, temperatures, constraint-representation probes); see its `README.md` |
| `data/quantization_controls/` | generation files of the quantization controls; see its `README.md` |
| `scripts/review/round3_guard.py` | keeps the round-2 table generators from overwriting the round-3 tables |
| `scripts/review/package_round3.sh`, `scripts/review/build_round3_supplementary.py` | build the submission package and the supplementary archive |

## Not in the repository

| Item | Where |
|---|---|
| Response letter | not written yet |
| B1 to B9 of the decision letter | not received |
| `audit_v2.zip`, `confirmatory_run_comparison.zip` (supplementary files of the earlier rounds) | the Windows machine; to be checked against the previous upload and added to `04_supplementary/` at upload |
| `results/` | the runner's output directory, outside version control: original copies of the runs now under `data/`, the cross-model output files cited for V3 (`results/stats*/`), the live-execution pilot telemetry and pilots that the paper does not report |
| `r2workdrive/` | local working directory, outside version control: build files of the earlier packages and a backup of the branch from before the history rewrite |
| built PDFs in `paper/` and `paper/diff/` | outside version control; the same PDFs are in `docs/review/submission_v10/` |

## Rebuilding

```bash
bash analysis/round3/run_all.sh          # every round-3 number and table, checked against the committed files
bash scripts/review/package_round3.sh    # clean PDF, highlighted PDF, source.zip, supplementary zip, MANIFEST
./scripts/run_tests.sh --fast            # test suite
```

The PDFs are built with the `texlive/texlive` docker image; `package_round3.sh` needs docker,
`pdfinfo`, `pdftotext` and `zip`.
