# Additional (not preregistered) generation runs

Generation logs, result files and generated URScript programs of the runs that the article
reports as additional analyses, copied from the runner's output directory `results/` by
`analysis/round3/s13_import_runs.py`. Directory names are the runner's own.

| Directory | Run in the article |
|---|---|
| `smoke_test/` | the single-task probe of Section VI-A (three local models, task T001) |
| `E1_frontier/` | frontier models, baseline and safety-prompted conditions (Table 7, Section VI-E) |
| `E3_frontier/` | frontier models in the repair loop (Section VI-E) |
| `E1_frontier_probe/`, `E1_frontier_probe2/`, `E3_frontier_probe/` | the preliminary frontier probes run before the two above |
| `E3_rich_probe/`, `E3_rich_mini/` | the enriched-feedback probe (Section VI-E) |
| `temp_robustness/` | sampling temperatures 0.2 and 0.7 (Section VI-E) |
| `e4_template_probe/` | the control probe of the constraint-representation analysis (Section VI-E) |
| `e4_canary/`, `e4_smoke/` | its preliminary checks (`e4_smoke` uses a mock model) |
| `e5_repr_fragility/` | the representation probe (Section VI-E, Figures on encoding and construct removal) |

Two files were masked: in `temp_robustness/run_temp0p2.log` and `run_temp0p7.log` a local home
directory is replaced by `<HOME>` and the inference endpoint by `<IDUN_ENDPOINT>` (one and 181
substitutions in each). Only those substrings changed; no prompt sent to a model and no model
response was altered. Every other file is byte-identical to its source
(`analysis/round3/out/r3/import_manifest.csv` lists both SHA-256 values). One frontier program
(`T014_mistralai_Mistral-Large-3-675B-Instruct-2512-NVFP4_baseline_rep1.urscript`, in
`E1_frontier/`, `E3_frontier/` and `e4_template_probe/`) contains the address 192.168.1.100,
which the model wrote into its own program; it is part of the model response and is kept.
