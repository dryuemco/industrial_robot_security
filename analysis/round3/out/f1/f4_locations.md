| where | file | line | text |
|---|---|---|---|
| repository, tracked | scripts/mcnemar_analysis.py | 16 | Cochran's Q (run_cross_model_cochran_q) is NOT a fourth confirmatory |
| repository, tracked | scripts/mcnemar_analysis.py | 159 | condition: str           # e.g. 'baseline', 'safety', 'adversarial_any' |
| repository, tracked | scripts/mcnemar_analysis.py | 514 | def run_cross_model_cochran_q(df: pd.DataFrame) -> list[CochranResult]: |
| repository, tracked | scripts/mcnemar_analysis.py | 521 | - 'adversarial_any'  : OR-aggregate of rows whose condition |
| repository, tracked | scripts/mcnemar_analysis.py | 564 | matrix, models, tasks, condition_label="adversarial_any", |
| repository, tracked | scripts/mcnemar_analysis.py | 568 | watchdog = df[df["condition"] == "watchdog"] |
| repository, tracked | scripts/mcnemar_analysis.py | 571 | matrix, models, tasks, condition_label="watchdog", |
| repository, tracked | scripts/mcnemar_analysis.py | 907 | else ("E3" if cond == "watchdog" else "E2") |
| repository, tracked | scripts/mcnemar_analysis.py | 1083 | # adversarial_any_a8k (E2 pooled across A8.1-A8.7). DM-* counts are |
| repository, tracked | scripts/mcnemar_analysis.py | 1129 | - adversarial_any_a8k:    condition startswith "adversarial_A8." |
| repository, tracked | scripts/mcnemar_analysis.py | 1152 | return "adversarial_any_a8k" |
| repository, tracked | scripts/mcnemar_analysis.py | 1174 | group_order = {"baseline": 0, "safety": 1, "adversarial_any_a8k": 2} |
| repository, tracked | scripts/mcnemar_analysis.py | 1266 | cochran_results = run_cross_model_cochran_q(df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 38 | run_cross_model_cochran_q, |
| repository, tracked | tests/test_mcnemar_analysis.py | 329 | (r for r in results if r.model == "all" and r.condition_b == "watchdog"), |
| repository, tracked | tests/test_mcnemar_analysis.py | 593 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 594 | # Exactly four results: baseline, safety, adversarial_any, watchdog. |
| repository, tracked | tests/test_mcnemar_analysis.py | 597 | assert conds == {"baseline", "safety", "adversarial_any", "watchdog"} |
| repository, tracked | tests/test_mcnemar_analysis.py | 600 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 608 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 615 | results = run_cross_model_cochran_q(empty) |
| repository, tracked | tests/test_mcnemar_analysis.py | 624 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 640 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 649 | assert "adversarial_any" in md |
| repository, tracked | tests/test_mcnemar_analysis.py | 662 | results = run_cross_model_cochran_q(full_condition_df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 861 | """run_h4/h5/h6 return list[McNemarResult]; run_cross_model_cochran_q |
| repository, tracked | tests/test_mcnemar_analysis.py | 871 | cq = run_cross_model_cochran_q(df) |
| repository, tracked | tests/test_mcnemar_analysis.py | 1125 | assert row["condition_group"] == "adversarial_any_a8k" |
| repository, tracked | tests/test_mcnemar_analysis.py | 1221 | assert e3.iloc[0]["condition"] == "watchdog" |
| repository, tracked (Cochran mentions) | docs/REPLICATION.md | 139 | `mcnemar_results.csv`, `cochran_results.csv`, `contingency_tables.json` and |
| repository, tracked (Cochran mentions) | docs/REPLICATION.md | 256 | Cochran's Q across the three models is reported in the same output as exploratory only; it is not part of the confirmatory H4–H6 family. |
| repository, tracked (Cochran mentions) | docs/open_science_release.md | 102 | | Statistical analysis scripts | Pending | McNemar + Holm-Bonferroni + Cochran's Q | |
| repository, tracked (Cochran mentions) | docs/open_science_release.md | 181 | [ ] Confirmatory statistical analysis (McNemar, Holm-Bonferroni, Cochran's Q) |
| repository, tracked (Cochran mentions) | paper/main_access.tex | 893 | Cross-model heterogeneity, Cochran $Q$ & 12.60 / 20.00 & $p{=}0.004$ / $p{<}0.001$, Holm-corrected; baseline / safety-prompted \\ |
| repository, tracked (Cochran mentions) | paper/main_access.tex | 911 | Speed-cap breach by encoding form & 15.0 / 1.7 / 0 / 0\% & implicit, comment, named constant, guard; Cochran-Armitage $p{<}10^{-4}$ \\ |
| repository, tracked (Cochran mentions) | paper/main_access.tex | 1372 | three models at Q4\_K\_M gives Cochran $Q = 8.67$ at baseline and $7.82$ under the safety |
| repository, tracked (Cochran mentions) | paper/main_access.tex | 1509 | programs with zero breaches. The monotone decrease across the four ordered encodings is significant by a Cochran-Armitage trend test, with a trend statistic of  |
| repository, tracked (Cochran mentions) | paper/main_access.tex | 1531 | explicit. Breach falls monotonically with legibility (Cochran-Armitage trend statistic |
| repository, tracked (Cochran mentions) | paper/tables/quantization_summary.tex | 12 | Cochran $Q$ of \ref{sec:res-sens} with all three models at one level, with the per-task |
| repository, tracked (Cochran mentions) | paper/tables/quantization_summary.tex | 35 | \multicolumn{12}{@{}l}{\textit{Cross-model Cochran $Q$ (15 task cells, cell = any repetition violating)}} \\ |
| results/, gitignored | results/stats/cochran_results.csv | 4 | adversarial_any,3,15,8.0,2,0.018315638888734182,0.018315638888734182,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepseek-coder-v2:1 |
| results/, gitignored | results/stats/cochran_results.csv | 5 | watchdog,3,15,22.933333333333334,2,1.0473454271059438e-05,4.1893817084237754e-05,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepsee |
| results/, gitignored | results/stats/hypothesis_report.md | 76 | | adversarial_any | 3 | 15 | 8.000 | 2 | 0.018 | 0.018 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats/hypothesis_report.md | 77 | | watchdog | 3 | 15 | 22.933 | 2 | <0.001 | <0.001 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats_e2/cochran_results.csv | 4 | adversarial_any,3,15,8.0,2,0.018315638888734182,0.018315638888734182,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepseek-coder-v2:1 |
| results/, gitignored | results/stats_e2/hypothesis_report.md | 68 | | adversarial_any | 3 | 15 | 8.000 | 2 | 0.018 | 0.018 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats_e3/cochran_results.csv | 4 | adversarial_any,0,0,,0,,,False,insufficient data for Cochran's Q,, |
| results/, gitignored | results/stats_e3/hypothesis_report.md | 62 | | adversarial_any | 0 | 0 | — | — | — | — | insufficient data | |
| results/, gitignored | results/stats_e3_full/cochran_results.csv | 4 | adversarial_any,3,15,8.0,2,0.018315638888734182,0.018315638888734182,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepseek-coder-v2:1 |
| results/, gitignored | results/stats_e3_full/cochran_results.csv | 5 | watchdog,3,15,22.933333333333334,2,1.0473454271059438e-05,4.1893817084237754e-05,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepsee |
| results/, gitignored | results/stats_e3_full/hypothesis_report.md | 76 | | adversarial_any | 3 | 15 | 8.000 | 2 | 0.018 | 0.018 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats_e3_full/hypothesis_report.md | 77 | | watchdog | 3 | 15 | 22.933 | 2 | <0.001 | <0.001 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats_e3_full.before_watchdog_rerun/cochran_results.csv | 4 | adversarial_any,3,15,8.0,2,0.018315638888734182,0.018315638888734182,True,"Cochran's Q, k=3, N=15 | REJECT H0 (models differ)",codellama:34b;deepseek-coder-v2:1 |
| results/, gitignored | results/stats_e3_full.before_watchdog_rerun/hypothesis_report.md | 76 | | adversarial_any | 3 | 15 | 8.000 | 2 | 0.018 | 0.018 | ✅ REJECT H0 (models differ) | |
| results/, gitignored | results/stats_session14/cochran_results.csv | 4 | adversarial_any,0,0,,0,,,False,insufficient data for Cochran's Q,, |
| results/, gitignored | results/stats_session14/hypothesis_report.md | 54 | | adversarial_any | 0 | 0 | — | — | — | — | insufficient data | |
| zip docs/review/submission_v5/04_supplementary/audit_v2.zip (sha256 437ebd424374) | 108 members | 0 | no line matches adversarial_any, Cochran or 22.93 |
| zip docs/review/submission_v5/04_supplementary/confirmatory_run_comparison.zip (sha256 72a4802dc03c) | 7 members | 0 | no line matches adversarial_any, Cochran or 22.93 |
