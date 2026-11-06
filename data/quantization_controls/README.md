# Quantization controls

Generated programs, logs and result files of the quantization controls of Section VI-D and
Table 16: each model re-run at the quantization level it was not served at. `*_rerun/` holds
the cells regenerated after the token-budget fix described in Appendix A; the merged per-row
results used by the article are `paper/data/X10_*/e1_results.csv`. Copied from `results/` by
`analysis/round3/s13_import_runs.py`; no file needed masking, and every file is byte-identical
to its source (`analysis/round3/out/r3/import_manifest.csv`).
