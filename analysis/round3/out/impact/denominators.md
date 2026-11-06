# Uses of 130 and 135 in the paper

130 = baseline outputs with a stored program (5 of the 135 calls returned no program: 4 errors, 1 parse failure; out/gate_equivalence.csv); 135 = baseline calls.

| file:line | place | count used | text |
|---|---|---|---|
| paper/main_access.tex:57 | abstract | 135 | 135 baseline generations are valid and violation-free; a non-LLM comparator shows that part |
| paper/main_access.tex:173 | Introduction / Contributions | 135 | counts only generations that are both valid and violation-free, is 1 of 135 at |
| paper/main_access.tex:716 | Experimental Setup / Experiments [table from line 703, tab:budget] | 135 | Baseline & 270 & 135 & 3 & 2 & 191 \\ |
| paper/main_access.tex:718 | Experimental Setup / Experiments [table from line 703, tab:budget] | 135 | Repair-in-loop & 339 & 135 & 3 & 1 & 261 \\ |
| paper/main_access.tex:823 | Experimental Setup / Statistical Analysis | 135 | \textbf{Statistical units and dependence.} The baseline corpus of 135 generations is |
| paper/main_access.tex:827 | Experimental Setup / Statistical Analysis | 135 | the 45 model-task cells and the 135 calls. Point estimates are unaffected by this |
| paper/main_access.tex:879 | Results [table from line 865, tab:summary] | 135 | Substantive-safe rate (primary) & 1/135 (0.7\%) & [0.02, 4.06], Clopper-Pearson \\ |
| paper/main_access.tex:882 | Results [table from line 865, tab:summary] | 130 | \quad restricted to the check rated $\geq$90\% by rater B of the second audit & 89.2\% & SM-6 only; 9 of 130 violation-free \\ |
| paper/main_access.tex:883 | Results [table from line 865, tab:summary] | 130 | \quad restricted to the checks rated $\geq$90\% after adjudication & 98.8\% & SM-2 and SM-6; 1 of 130 violation-free \\ |
| paper/main_access.tex:885 | Results [table from line 865, tab:summary] | 130 | \quad under the structural validity gate & 100\% & 38 of 130 outputs admitted \\ |
| paper/main_access.tex:892 | Results [table from line 865, tab:summary] | 135 | Cells remediated to zero violations & 0/135 & Table~\ref{tab:regimes} \\ |
| paper/main_access.tex:937 | Results / Baseline Security of Generated Code | 135 | identical at the level of the verdict, the 45 model-task cells rather than the 135 calls |
| paper/main_access.tex:974 | Results / Baseline Security of Generated Code | 135 | the 135 baseline generations, 83 pass the validity gate and 52 do not; of the 83 |
| paper/main_access.tex:976 | Results / Baseline Security of Generated Code | 135 | substantive-safe rate of 1 of 135, or 0.7\% (Clopper-Pearson 95\% confidence interval |
| paper/main_access.tex:993 | Results / Baseline Security of Generated Code [table from line 992] | 135 | \caption{Per-rule firing counts over the 135 baseline generations and the 135 |
| paper/main_access.tex:1122 | Results / Baseline Security of Generated Code | 130 | violation rate at 74 of 83, or 89.2\%, with 9 rather than 1 of 130 baseline outputs |
| paper/main_access.tex:1125 | Results / Baseline Security of Generated Code | 130 | two leaves the rate at 98.8\% with 1 of 130 violation-free, the same as the registered |
| paper/main_access.tex:1129 | Results / Baseline Security of Generated Code | 135 | the substantive-safe rate of 1 of 135 must be read as a rate under the full registered |
| paper/main_access.tex:1147 | Results / Baseline Security of Generated Code | 135 | unchanged. On the all-outputs basis the same means are 6.38 and 5.11 (135 generations each). This |
| paper/main_access.tex:1163 | Results / Baseline Security of Generated Code | 135 | of 135 (0.7\%), since of the 83 gate-passing outputs only one triggers no check. |
| paper/main_access.tex:1219 | Results / Baseline Security of Generated Code | 130, 135 | non-URScript motion primitive. Of the 135 baseline generations, 130 have a stored |
| paper/main_access.tex:1221 | Results / Baseline Security of Generated Code | 135 | as invalid in the 1 of 135 rate. The gate-passing violation rate moves from 98.8\% to no |
| paper/main_access.tex:1222 | Results / Baseline Security of Generated Code | 130 | lower than 89.2\%, and the substantive-safe count from 1 to at most 9 of 130 scored |
| paper/main_access.tex:1247 | Results / Adversarial Uplift | 135 | 61\% (83 of 135 at baseline; 192 of 315 pooled over strategies), so the strategies do not |
| paper/main_access.tex:1317 | Results / Repair-in-Loop Defense | 135 | classifies every one of the 135 local-model cells by the shape of its violation-count |
| paper/main_access.tex:1318 | Results / Repair-in-Loop Defense | 135 | sequence. Of the 135 cells, 78 leave the loop by dropping out of valid syntax, 52 of them |
| paper/main_access.tex:1453 | Results / Additional Analyses: Frontier, Robustness, and Representation | 135 | across the 135 confirmatory cells and 60 frontier cells, yields a remediated-to-zero |
| paper/main_access.tex:1454 | Results / Additional Analyses: Frontier, Robustness, and Representation | 135 | count of 0 of 135 and 0 of 60: no model at any size from 16B to 675B used the retry |
| paper/main_access.tex:1457 | Results / Additional Analyses: Frontier, Robustness, and Representation | 135 | behavior is that the loop never runs or breaks early, with 78 of 135 cells either |
| paper/main_access.tex:1567 | Discussion / Comparison with Prior Findings | 135 | one violation and only 1 to 9 programs in 135 are both valid and rule-clean. Three factors |
| paper/main_access.tex:1765 | Conclusion | 135 | gate-passing baseline outputs violate at least one, and 1 to 9 of 135 baseline generations are |
| paper/main_access.tex:1861 | Replication and Preregistration Mapping | 135 | 135 at baseline, 192 of 315 adversarial), the single substantive-safe program (the same |
| paper/tables/sensitivity_e1_baseline.tex:4 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | E1 baseline ($n$=130 generations), re-scored offline from the stored outputs. |
| paper/tables/sensitivity_e1_baseline.tex:22 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & preregistered & 83 & 98.8 & 1/130 & 10.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:23 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & corrected-SM5 & 83 & 98.8 & 1/130 & 9.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:24 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & high-precision & 83 & 92.8 & 6/130 & 5.9 \\ |
| paper/tables/sensitivity_e1_baseline.tex:25 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & high-prec+corr & 83 & 91.6 & 7/130 & 5.3 \\ |
| paper/tables/sensitivity_e1_baseline.tex:26 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & drop-SM4 & 83 & 98.8 & 1/130 & 9.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:27 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & SM6-only & 83 & 89.2 & 9/130 & 1.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:28 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | lexical & SM2+SM6 & 83 & 98.8 & 1/130 & 3.0 \\ |
| paper/tables/sensitivity_e1_baseline.tex:30 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & preregistered & 38 & 100.0 & 0/130 & 11.6 \\ |
| paper/tables/sensitivity_e1_baseline.tex:31 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & corrected-SM5 & 38 & 100.0 & 0/130 & 10.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:32 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & high-precision & 38 & 100.0 & 0/130 & 7.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:33 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & high-prec+corr & 38 & 97.4 & 1/130 & 6.6 \\ |
| paper/tables/sensitivity_e1_baseline.tex:34 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & drop-SM4 & 38 & 100.0 & 0/130 & 11.2 \\ |
| paper/tables/sensitivity_e1_baseline.tex:35 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & SM6-only & 38 & 92.1 & 3/130 & 1.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:36 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | 130 | structural & SM2+SM6 & 38 & 100.0 & 0/130 & 3.2 \\ |
| paper/tables/adversarial_table.tex:23 | Results / Adversarial Uplift (\input at line 1253) [tab:adversarial] | 135 | & \textit{Pooled} & 135 & 83 (61\%) & 60.7 & 98.8 & 6.38 & 10.37 & 0.88 \\ |
| paper/tables/e3_regimes.tex:3 | Results / Repair-in-Loop Defense (\input at line 1330) [tab:regimes] | 135 | \caption{Repair-in-loop trajectory regimes for the 135 local-model cells of E3 |
| paper/tables/e3_regimes.tex:26 | Results / Repair-in-Loop Defense (\input at line 1330) [tab:regimes] | 135 | Cells & 45 & 45 & 45 & 135 \\ |
