# Confirmatory data (E1, E2, E3), both runs

Raw per-call results of the three confirmatory experiments: result CSV, summary JSON,
full request/response log (`logs/ollama_log.jsonl`), runner stdout log and every generated
program (`code/`). The confirmatory matrix was generated twice with the same code path,
prompts, task representations, models and decoding settings.

| Directory | Run | Rows (E1 / E2 / E3) | Hash manifest |
|---|---|---|---|
| `run2_2026-05/` (`E1_full`, `E2_full`, `E3_full`, `chain.log`, `E1_full_stdout.log`) | 2026-05-15/16, **the run reported in the paper** | 270 / 315 / 339 | `docs/confirmatory_results/MANIFEST.json` |
| `run1_2026-04/` (`e1_confirmatory_session14`, `e2_confirmatory`, `e3_confirmatory`) | 2026-04-15/16, analysed first, not an analysis input | 270 / 315 / 339 | `docs/confirmatory_results/MANIFEST_run1_april.json` |

The directory names are the ones the runs were written under, so that logs, manifests and
session notes stay valid. The analysis, table and figure scripts read `results/`; to
regenerate the paper's tables and statistics from the shipped data:

```bash
mkdir -p results && cp -r data/confirmatory/run2_2026-05/E?_full results/
python3 scripts/mcnemar_analysis.py --results-dir results/ --output-dir results/stats/
```

`python3 scripts/review/compare_confirmatory_runs.py` compares the two runs row by row
(see "The two confirmatory runs" in `docs/REPLICATION.md` and Appendix A of the paper):
every preregistered test is identical in the two runs; the generated programs of
DeepSeek-Coder-V2-16B and CodeLlama-34B are byte-identical; all differences are in
Qwen2.5-Coder-32B and affect violation counts only.

The logs contain the inference host's private LAN address and a local checkout path; they
contain no credentials.
