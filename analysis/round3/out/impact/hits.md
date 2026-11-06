# Impact grep

Files searched: `paper/main_access.tex`, `paper/tables/task_table.tex`, `paper/tables/precision_audit_v2.tex`, `paper/tables/rep_reproducibility.tex`, `paper/tables/comparator_summary.tex`, `paper/tables/sensitivity_e1_baseline.tex`, `paper/tables/adversarial_table.tex`, `paper/tables/e3_regimes.tex`, `paper/tables/quantization_summary.tex`, `paper/tables/complexity_table.tex`, `paper/tables/prompts_appendix.tex`, `paper/tables/code_listings.tex`

## A1 (101 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:56 | abstract | code fails the checks: 89.2\% to 98.8\% of valid outputs violate at least one, and 1 to 9 of |
| paper/main_access.tex:57 | abstract | 135 baseline generations are valid and violation-free; a non-LLM comparator shows that part |
| paper/main_access.tex:59 | abstract | many rule firings as the reference translation. Second, this saturation is a measurement |
| paper/main_access.tex:113 | Introduction | control code, in the sense of ISO 10218-1:2025 clause 5.1.16, and not demonstrated |
| paper/main_access.tex:168 | Introduction / Contributions | ISO 10218-1:2025 clauses, including the new cybersecurity clause 5.1.16. |
| paper/main_access.tex:177 | Introduction / Contributions | \item \textbf{A non-LLM comparator and a sensitivity analysis of the instrument.} |
| paper/main_access.tex:178 | Introduction / Contributions | The same checker applied to a deterministic reference translation and to vendor example |
| paper/main_access.tex:398 | Threat Model / Attack Families [table from line 385, tab:attacks] | Payload tampering & 5.1.15 & Payload mass or centre of gravity outside declared bounds \\ |
| paper/main_access.tex:401 | Threat Model / Attack Families [table from line 385, tab:attacks] | Tool misuse & 5.1.14 & Invalid tool identity or activation mode \\ |
| paper/main_access.tex:402 | Threat Model / Attack Families [table from line 385, tab:attacks] | Prompt injection & 5.1.16 & Adversarial prompts or configuration tampering \\ |
| paper/main_access.tex:456 | The FACT Framework / Task Representation | safety zones expressed as halfspace geometry), an operating mode, and a payload and |
| paper/main_access.tex:534 | The FACT Framework / Generation Conditions | correct payload setting. The task's own speed cap and operating mode are part of the user |
| paper/main_access.tex:543 | The FACT Framework / Rule Checker | rendered in code. It comprises seven checks: a speed-and-payload range check covering |
| paper/main_access.tex:544 | The FACT Framework / Rule Checker | the mode-specific velocity limits and the declared payload bounds, a safe-zone |
| paper/main_access.tex:558 | The FACT Framework / Rule Checker | acceleration literals that exceed limits (CWE-798), a safety-preamble check for a |
| paper/main_access.tex:559 | The FACT Framework / Rule Checker | missing tool-centre-point (the controlled reference point at the robot's tool) or payload |
| paper/main_access.tex:584 | The FACT Framework / ISO 10218-1:2025 Traceability | distinct clauses (5.1.14, 5.1.15, 5.1.16, 5.4, 5.4.2, 5.5.3, and 5.7.4) spanning four |
| paper/main_access.tex:588 | The FACT Framework / ISO 10218-1:2025 Traceability | relevance is clause 5.1.16 on cybersecurity, newly introduced in the 2025 revision: |
| paper/main_access.tex:603 | The FACT Framework / ISO 10218-1:2025 Traceability | speed-limit concern, yet anchors to clause 5.1.16, because the underlying weakness is |
| paper/main_access.tex:882 | Results [table from line 865, tab:summary] | \quad restricted to the check rated $\geq$90\% by rater B of the second audit & 89.2\% & SM-6 only; 9 of 130 violation-free \\ |
| paper/main_access.tex:988 | Results / Baseline Security of Generated Code | firing counts come from the unchecked-return and missing-preamble checks, |
| paper/main_access.tex:995 | Results / Baseline Security of Generated Code [table from line 992] | and within it by the unchecked-return and missing-preamble checks; the motion-safety |
| paper/main_access.tex:1009 | Results / Baseline Security of Generated Code [table from line 992, tab:perrule] | Safety preamble & none & 74 & 69 \\ |
| paper/main_access.tex:1022 | Results / Baseline Security of Generated Code | reports the result. The hardcoded-value and missing-preamble checks are high precision |
| paper/main_access.tex:1024 | Results / Baseline Security of Generated Code | mode, and the tool-centre-point and payload configuration is genuinely absent before |
| paper/main_access.tex:1052 | Results / Baseline Security of Generated Code [table from line 1034, tab:precision] | Safety preamble & none & 11 & 11 & 100 [71.5, 100] \\ |
| paper/main_access.tex:1068 | Results / Baseline Security of Generated Code | the hardcoded-speed and missing-preamble checks, whose precision the audit supports. |
| paper/main_access.tex:1085 | Results / Baseline Security of Generated Code | ($\kappa=0.35$); the $\kappa$ of 0.00 on the preamble check is an artefact of rater A |
| paper/main_access.tex:1094 | Results / Baseline Security of Generated Code | error-handling firings name a \texttt{set\_tcp} or \texttt{set\_payload} call as the |
| paper/main_access.tex:1108 | Results / Baseline Security of Generated Code | acceleration, does not. Two preamble firings concern non-URScript calls |
| paper/main_access.tex:1110 | Results / Baseline Security of Generated Code | or payload on a controller. The false positives on which both raters agree are the same |
| paper/main_access.tex:1111 | Results / Baseline Security of Generated Code | mechanisms the comparator exposes: the input-validation proxy misses positional and |
| paper/main_access.tex:1120 | Results / Baseline Security of Generated Code | rater's threshold is applied. Under rater B's labels only the preamble check reaches 90\% |
| paper/main_access.tex:1122 | Results / Baseline Security of Generated Code | violation rate at 74 of 83, or 89.2\%, with 9 rather than 1 of 130 baseline outputs |
| paper/main_access.tex:1123 | Results / Baseline Security of Generated Code | violation-free (Table~\ref{tab:sensitivity}, SM6-only). Under the adjudicated labels the |
| paper/main_access.tex:1124 | Results / Baseline Security of Generated Code | error-handling proxy and the preamble check reach 90\%, and restricting the flag to those |
| paper/main_access.tex:1128 | Results / Baseline Security of Generated Code | readings, but its margin ranges from 89.2\% to 98.8\% across defensible check sets, and |
| paper/main_access.tex:1169 | Results / Baseline Security of Generated Code | \textbf{Non-LLM comparator.} A violation rate near 99\% is interpretable only against |
| paper/main_access.tex:1170 | Results / Baseline Security of Generated Code | programs that no language model wrote. Table~\ref{tab:comparator} applies the same checker |
| paper/main_access.tex:1171 | Results / Baseline Security of Generated Code | to three such sets: the deterministic reference translation of each task produced by the |
| paper/main_access.tex:1173 | Results / Baseline Security of Generated Code | \texttt{set\_tcp}/\texttt{set\_payload} preamble that the preamble check (SM-6) requires |
| paper/main_access.tex:1174 | Results / Baseline Security of Generated Code | (the safety prompt itself asks only for the payload setting); and nine vendor and community example programs redistributed under their own licences |
| paper/main_access.tex:1175 | Results / Baseline Security of Generated Code | in the replication package. The comparator is deliberately unflattering to the |
| paper/main_access.tex:1176 | Results / Baseline Security of Generated Code | instrument. Under the preregistered rule set the reference translation is flagged on all |
| paper/main_access.tex:1177 | Results / Baseline Security of Generated Code | fifteen tasks: the unusual-conditions check (SM-4) and the preamble check (SM-6) fire on |
| paper/main_access.tex:1179 | Results / Baseline Security of Generated Code | nor a preamble, and the hardcoded-value check (SM-5) fires on every reference program |
| paper/main_access.tex:1181 | Results / Baseline Security of Generated Code | joint speed in rad/s, against the 0.25~m/s Cartesian limit. Adding the preamble clears |
| paper/main_access.tex:1183 | Results / Baseline Security of Generated Code | \texttt{set\_tcp} and \texttt{set\_payload} as critical operations that require a guard |
| paper/main_access.tex:1190 | Results / Baseline Security of Generated Code | generated code produces 10.4 firings per program against 5.7 for the reference translation, |
| paper/main_access.tex:1193 | Results / Baseline Security of Generated Code | unit reading (Table~\ref{tab:comparator}, corrected SM-5) removes every SM-5 firing from the |
| paper/main_access.tex:1194 | Results / Baseline Security of Generated Code | reference translation but only 8 of the 54 generated programs on which SM-5 fires (44 of |
| paper/main_access.tex:1204 | Results / Baseline Security of Generated Code [figure from line 1200] | condition in the baseline experiment, with the reference translation and the vendor and |
| paper/main_access.tex:1209 | Results / Baseline Security of Generated Code | \input{tables/comparator_summary.tex} |
| paper/main_access.tex:1222 | Results / Baseline Security of Generated Code | lower than 89.2\%, and the substantive-safe count from 1 to at most 9 of 130 scored |
| paper/main_access.tex:1566 | Discussion / Comparison with Prior Findings | whereas under the validity gate 89.2\% to 98.8\% of baseline URScript generations contain at least |
| paper/main_access.tex:1647 | Discussion / Implications for Deployment | Four implications follow. First, safety prompting helps two of three models but is not universally protective, and a unit-naive safety prompt can even elicit more dangerous code through a millimetre-versus-metre confusio |
| paper/main_access.tex:1682 | Discussion / Threats to Validity | on manual audits, the second with two raters; the non-LLM comparator and the sensitivity re-scoring of |
| paper/main_access.tex:1697 | Discussion / Threats to Validity | clause 5.1.16 and the code-level checks anchored to it. Clause 5.1.16 frames |
| paper/main_access.tex:1704 | Discussion / Threats to Validity | whether remote access is authenticated. The checks anchored to 5.1.16 therefore capture |
| paper/main_access.tex:1708 | Discussion / Threats to Validity | mapping is offered as evidence about the code-expressible part of 5.1.16, not as a |
| paper/main_access.tex:1764 | Conclusion | Three findings emerge. First, generated code pervasively fails the checks: 89.2\% to 98.8\% of |
| paper/main_access.tex:1766 | Conclusion | valid and violation-free. The non-LLM comparator shows that part of this rate is a property of the |
| paper/main_access.tex:1768 | Conclusion | reference translation. Second, the saturated binary rate is a measurement hazard. Scored |
| paper/main_access.tex:1825 | Replication and Preregistration Mapping | (orientation-cone violation), A4 (payload tampering), A5 (safety-logic bypass), A6 |
| paper/main_access.tex:1897 | Replication and Preregistration Mapping | carry the identifiers DM-1 (speed and payload range), DM-2 (safe-zone halfspace), DM-3 |
| paper/main_access.tex:1902 | Replication and Preregistration Mapping | value, CWE-798), SM-6 (safety preamble), and SM-7 (prompt-injection marker). Their ISO |
| paper/main_access.tex:1916 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-1 & Payload mass and centre of gravity & 5.1.15 \\ |
| paper/main_access.tex:1923 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-6 & Tool identity & 5.1.14 \\ |
| paper/main_access.tex:1924 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-6 & Tool activation mode & 5.1.14 \\ |
| paper/main_access.tex:1925 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-7 & Configuration integrity & 5.1.16 \\ |
| paper/main_access.tex:1926 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-1 & Input-validation proxy (rel. CWE-20) & 5.1.16 \\ |
| paper/main_access.tex:1928 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-3 & Protection mechanism (CWE-693) & 5.1.16 \\ |
| paper/main_access.tex:1930 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-5 & Hardcoded value (CWE-798) & 5.1.16 \\ |
| paper/main_access.tex:1931 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-6 & Tool-centre-point preamble & 5.1.14 \\ |
| paper/main_access.tex:1932 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-6 & Payload preamble & 5.1.15 \\ |
| paper/main_access.tex:1933 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | SM-7 & Prompt-injection marker & 5.1.16 \\ |
| paper/main_access.tex:1957 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Excessive speed & DM-1 & SM-5 & 5.5.3; 5.1.16 \\ |
| paper/main_access.tex:1959 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Missing protective stop & DM-4 & SM-3 & 5.4.2, 5.4; 5.1.16 \\ |
| paper/main_access.tex:1960 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Payload out of bounds & DM-1 & SM-6 & 5.1.15 \\ |
| paper/main_access.tex:1961 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Tool misuse & DM-6 & SM-6 & 5.1.14 \\ |
| paper/main_access.tex:1964 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Missing input validation & none & SM-1 & 5.1.16 \\ |
| paper/main_access.tex:1965 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Config tampering or injection & DM-7 & SM-7 & 5.1.16 \\ |
| paper/main_access.tex:2060 | Watchdog Detection Performance | manual audit of \ref{sec:res-baseline} and the non-LLM comparator reported there. An |
| paper/tables/task_table.tex:7 | The FACT Framework / Task Representation (\input at line 525) [tab:tasks] | e-stop verification step and a home pose; payload is the declared tool payload.} |
| paper/tables/precision_audit_v2.tex:25 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Safety preamble & 20 & 20 & 100.0 [83.2, 100.0] & 18 & 90.0 [68.3, 98.8] & 90\% & 0.00 & 20 & 100.0 [83.2, 100.0] \\ |
| paper/tables/comparator_summary.tex:3 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | \caption{Non-LLM comparator. Programs that were not produced by a language model, |
| paper/tables/comparator_summary.tex:10 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | speeds and applies the task's own TCP cap. The translator is the deterministic |
| paper/tables/comparator_summary.tex:13 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | \label{tab:comparator} |
| paper/tables/comparator_summary.tex:21 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator & Preregistered & 15 & 15 & 15 & 5.7 & 0 & 0 & 0 & 15 & 15 & 15 & 0 \\ |
| paper/tables/comparator_summary.tex:22 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator & Corrected SM-5 & 15 & 15 & 15 & 3.0 & 0 & 0 & 0 & 15 & 0 & 15 & 0 \\ |
| paper/tables/comparator_summary.tex:24 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator + preamble & Preregistered & 15 & 15 & 15 & 5.7 & 0 & 15 & 0 & 15 & 15 & 0 & 0 \\ |
| paper/tables/comparator_summary.tex:25 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator + preamble & Corrected SM-5 & 15 & 15 & 15 & 3.0 & 0 & 15 & 0 & 15 & 0 & 0 & 0 \\ |
| paper/tables/sensitivity_e1_baseline.tex:8 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | first audit rated at or above 90\% precision; \emph{SM6-only} and \emph{SM2+SM6} keep the |
| paper/tables/sensitivity_e1_baseline.tex:27 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM6-only & 83 & 89.2 & 9/130 & 1.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:35 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & SM6-only & 38 & 92.1 & 3/130 & 1.8 \\ |
| paper/tables/prompts_appendix.tex:22 | Prompts Used in the Experiments (\input at line 2007) | - Set payload correctly before operations |
| paper/tables/prompts_appendix.tex:30 | Prompts Used in the Experiments (\input at line 2007) | Description: Single-object pick-place, 0.5 kg payload, 200 mm/s. Baseline safe task for adversarial testing. All parameters comply with ISO 10218:2025 collaborative mode limits. |
| paper/tables/prompts_appendix.tex:109 | Prompts Used in the Experiments (\input at line 2007) | Description: Single-object pick-place, 0.5 kg payload, 200 mm/s. Baseline safe task for adversarial testing. All parameters comply with ISO 10218:2025 collaborative mode limits. |
| paper/tables/code_listings.tex:8 | Annotated Generated-Code Listings (\input at line 2013) | preamble (SM-6). The listing shows why the gate is deliberately described as lexical |
| paper/tables/code_listings.tex:43 | Annotated Generated-Code Listings (\input at line 2013) | URScript (\texttt{set\_tcp\_speed}, \texttt{set\_joint\_acceleration}), a hallucinated |
| paper/tables/code_listings.tex:44 | Annotated Generated-Code Listings (\input at line 2013) | safety preamble that the preamble check does not credit. Lines 10 to 14 are placeholder |

## A2 (35 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:206 | Introduction / Organization of the Article | the repair-in-loop intervention, cross-model heterogeneity, and additional |
| paper/main_access.tex:800 | Experimental Setup / Statistical Analysis | confirmatory family and within each family's internal comparison set. A cross-model |
| paper/main_access.tex:801 | Experimental Setup / Statistical Analysis | homogeneity test is reported as an additional analysis only. Two further hypotheses, that baseline |
| paper/main_access.tex:819 | Experimental Setup / Statistical Analysis | more heterogeneous per-model reduction than the general-purpose setting; the pooled and |
| paper/main_access.tex:834 | Experimental Setup / Statistical Analysis | range. The cross-model homogeneity test operates at the task level and already accounts |
| paper/main_access.tex:867 | Results [table from line 865] | preregistered tests, their sensitivity analyses, and the cross-model heterogeneity test. |
| paper/main_access.tex:893 | Results [table from line 865, tab:summary] | Cross-model heterogeneity, Cochran $Q$ & 12.60 / 20.00 & $p{=}0.004$ / $p{<}0.001$, Holm-corrected; baseline / safety-prompted \\ |
| paper/main_access.tex:1261 | Results / Adversarial Uplift | single discordant pair among the 21 matched gate-passing cells of one strategy. This is a |
| paper/main_access.tex:1333 | Results / Sensitivity and Cross-Model Heterogeneity | The three models differ on matched per-task outcomes. A cross-model homogeneity test, |
| paper/main_access.tex:1335 | Results / Sensitivity and Cross-Model Heterogeneity | family, rejects homogeneity under both single-shot conditions: the test statistic is 12.60 |
| paper/main_access.tex:1336 | Results / Sensitivity and Cross-Model Heterogeneity | at the baseline condition (p of 0.004) and 20.00 at the safety-prompted condition (p below |
| paper/main_access.tex:1341 | Results / Sensitivity and Cross-Model Heterogeneity | Q4\_K\_M and two at Q4\_0 (Table~\ref{tab:models}), this heterogeneity test cannot by |
| paper/main_access.tex:1348 | Results / Sensitivity and Cross-Model Heterogeneity | level of the binary verdict, which is the level of the heterogeneity test: every task |
| paper/main_access.tex:1366 | Results / Sensitivity and Cross-Model Heterogeneity | 15 safety-prompted cells, but the discordant cells fall in both directions ($p = 0.73$ and |
| paper/main_access.tex:1371 | Results / Sensitivity and Cross-Model Heterogeneity | truncated; 17 fail the gate and 13 pass it. Recomputing the cross-model test with all |
| paper/main_access.tex:1372 | Results / Sensitivity and Cross-Model Heterogeneity | three models at Q4\_K\_M gives Cochran $Q = 8.67$ at baseline and $7.82$ under the safety |
| paper/main_access.tex:1374 | Results / Sensitivity and Cross-Model Heterogeneity | $p = 0.026$ for both); with all three at Q4\_0 the as-served |
| paper/main_access.tex:1376 | Results / Sensitivity and Cross-Model Heterogeneity | Q4\_0. The heterogeneity result therefore stands at the binary level that it was |
| paper/main_access.tex:1377 | Results / Sensitivity and Cross-Model Heterogeneity | registered for at either uniform quantization, with a narrower margin at Q4\_K\_M, where |
| paper/main_access.tex:1378 | Results / Sensitivity and Cross-Model Heterogeneity | CodeLlama-34B's per-task violation rate rises from 40\% to 53\% at baseline and from |
| paper/main_access.tex:1687 | Discussion / Threats to Validity | Q4\_0 (Table~\ref{tab:models}), so the cross-model heterogeneity test of |
| paper/main_access.tex:1690 | Discussion / Threats to Validity | binary heterogeneity survives at either uniform level while the count-level outcomes of |
| paper/main_access.tex:1863 | Replication and Preregistration Mapping | tests, and the cross-model statistics (12.60 and 20.00). The count-level values differ |
| paper/main_access.tex:2052 | Watchdog Detection Performance | confirmatory tests registered for this family were not performed in this manuscript, |
| paper/tables/adversarial_table.tex:55 | Results / Adversarial Uplift (\input at line 1253) [tab:adversarial] | A8.7 Obfuscation & Qwen2.5-Coder-32B & 15 & 15 (100\%) & 100.0 & 100.0 & 12.60 & 12.60 & 1.00 \\ |
| paper/tables/quantization_summary.tex:10 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | is the same at both levels; McNemar is the exact test on discordant cells and Wilcoxon |
| paper/tables/quantization_summary.tex:11 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | the signed-rank test on per-cell mean counts. The lower block recomputes the cross-model |
| paper/tables/quantization_summary.tex:12 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | Cochran $Q$ of \ref{sec:res-sens} with all three models at one level, with the per-task |
| paper/tables/quantization_summary.tex:35 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{\textit{Cross-model Cochran $Q$ (15 task cells, cell = any repetition violating)}} \\ |
| paper/tables/quantization_summary.tex:36 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{as served (Q4\_K\_M / Q4\_0 / Q4\_0), Baseline: $Q=12.60$, $p=0.002$ (Qwen 100\%; DeepSeek 60\%; CodeLlama 40\%)} \\ |
| paper/tables/quantization_summary.tex:37 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{as served (Q4\_K\_M / Q4\_0 / Q4\_0), Safety-prompted: $Q=20.00$, $p<0.001$ (Qwen 100\%; DeepSeek 100\%; CodeLlama 33\%)} \\ |
| paper/tables/quantization_summary.tex:38 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{all Q4\_K\_M, Baseline: $Q=8.67$, $p=0.013$ (Qwen 100\%; DeepSeek 87\%; CodeLlama 53\%)} \\ |
| paper/tables/quantization_summary.tex:39 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{all Q4\_K\_M, Safety-prompted: $Q=7.82$, $p=0.020$ (Qwen 100\%; DeepSeek 60\%; CodeLlama 53\%)} \\ |
| paper/tables/quantization_summary.tex:40 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{all Q4\_0, Baseline: $Q=12.60$, $p=0.002$ (Qwen 100\%; DeepSeek 60\%; CodeLlama 40\%)} \\ |
| paper/tables/quantization_summary.tex:41 | Results / Sensitivity and Cross-Model Heterogeneity (\input at line 1387) [tab:quant] | \multicolumn{12}{@{}l}{all Q4\_0, Safety-prompted: $Q=20.00$, $p<0.001$ (Qwen 100\%; DeepSeek 100\%; CodeLlama 33\%)} \\ |

## A3 (58 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:58 | abstract | of this rate is a property of the rule set, yet generated code produces about twice as |
| paper/main_access.tex:396 | Threat Model / Attack Families [table from line 385, tab:attacks] | Safe-zone penetration & 5.7.4 & Waypoints violating safe-zone halfspace constraints \\ |
| paper/main_access.tex:397 | Threat Model / Attack Families [table from line 385, tab:attacks] | Orientation-cone violation & 5.7.4 & Tool orientation inside a forbidden cone \\ |
| paper/main_access.tex:400 | Threat Model / Attack Families [table from line 385, tab:attacks] | Frame confusion & 5.7.4 & Coordinate-frame inconsistencies across waypoints \\ |
| paper/main_access.tex:552 | The FACT Framework / Rule Checker | input-validation proxy for missing velocity and acceleration arguments on motion calls |
| paper/main_access.tex:554 | The FACT Framework / Rule Checker | error-handling-presence proxy on critical operations (a surrogate indicator related to |
| paper/main_access.tex:579 | The FACT Framework / Rule Checker | \textbf{Scope of the checks.} The checks are lexical rules: each is a regular expression, or a small set of them, evaluated over the URScript text, and the validity gate is itself a regular expression that matches one of |
| paper/main_access.tex:584 | The FACT Framework / ISO 10218-1:2025 Traceability | distinct clauses (5.1.14, 5.1.15, 5.1.16, 5.4, 5.4.2, 5.5.3, and 5.7.4) spanning four |
| paper/main_access.tex:590 | The FACT Framework / ISO 10218-1:2025 Traceability | of the safety configuration and avoidance of default credentials, that are captured, at the code level, by the input-validation, protection-mechanism, hardcoded-value, |
| paper/main_access.tex:594 | The FACT Framework / ISO 10218-1:2025 Traceability | error-handling-presence proxy (related to CWE-252) and the unusual-condition check (CWE-754), have no |
| paper/main_access.tex:725 | Experimental Setup / Experiments | \textbf{Data collection.} The three confirmatory experiments were generated twice, with |
| paper/main_access.tex:888 | Results [table from line 865, tab:summary] | Safety-prompt effect on mean violations & 10.37 to 6.39 & 38\% lower; descriptive, no test \\ |
| paper/main_access.tex:1025 | Results / Baseline Security of Generated Code | motion. The input-validation and error-handling proxies are low precision (12 and 0 |
| paper/main_access.tex:1026 | Results / Baseline Security of Generated Code | percent). The input-validation proxy keys on the explicit speed and acceleration |
| paper/main_access.tex:1028 | Results / Baseline Security of Generated Code | positionally; the unchecked-return proxy keys on the presence of error-handling |
| paper/main_access.tex:1058 | Results / Baseline Security of Generated Code | error-handling proxy on 82 of the 83 gate-passing baseline outputs, the highest count of |
| paper/main_access.tex:1059 | Results / Baseline Security of Generated Code | any check, and the input-validation proxy on 52), the combined violation rate is mildly inflated by them under that |
| paper/main_access.tex:1084 | Results / Baseline Security of Generated Code | agreement on the error-handling proxy ($\kappa=0.20$) and the hardcoded-value check |
| paper/main_access.tex:1094 | Results / Baseline Security of Generated Code | error-handling firings name a \texttt{set\_tcp} or \texttt{set\_payload} call as the |
| paper/main_access.tex:1099 | Results / Baseline Security of Generated Code | whereas the written guide judges it as a program-level error-handling proxy, so the |
| paper/main_access.tex:1105 | Results / Baseline Security of Generated Code | reading, so all six are genuine, but the stated limit is wrong. Three input-validation |
| paper/main_access.tex:1111 | Results / Baseline Security of Generated Code | mechanisms the comparator exposes: the input-validation proxy misses positional and |
| paper/main_access.tex:1124 | Results / Baseline Security of Generated Code | error-handling proxy and the preamble check reach 90\%, and restricting the flag to those |
| paper/main_access.tex:1144 | Results / Baseline Security of Generated Code | generation falls from 10.37 under the baseline condition to 6.39 under the safety-prompted |
| paper/main_access.tex:1182 | Results / Baseline Security of Generated Code | SM-6 but trips the error-handling proxy (SM-2) on every program, because that rule treats |
| paper/main_access.tex:1189 | Results / Baseline Security of Generated Code | count and the rule profile do separate generated code from the reference: gate-passing |
| paper/main_access.tex:1190 | Results / Baseline Security of Generated Code | generated code produces 10.4 firings per program against 5.7 for the reference translation, |
| paper/main_access.tex:1191 | Results / Baseline Security of Generated Code | and it fires on the input-validation and error-handling proxies (52 and 82 of 83 programs) |
| paper/main_access.tex:1192 | Results / Baseline Security of Generated Code | on which the reference never fires (Figure~\ref{fig:heatmap}). Third, correcting the SM-5 |
| paper/main_access.tex:1243 | Results / Adversarial Uplift | safety-prompt contrast above, the same strategy raises the mean from 10.37 to 16.59 |
| paper/main_access.tex:1244 | Results / Adversarial Uplift | (+6.22) and the pooled mean from 10.37 to 11.61 (+1.24); five of the seven strategies lie |
| paper/main_access.tex:1324 | Results / Repair-in-Loop Defense | count is 10.4 at the first generation and 13.2 at the last retry. At the rule level, each |
| paper/main_access.tex:1352 | Results / Sensitivity and Cross-Model Heterogeneity | (Wilcoxon signed-rank on per-cell means, $p = 0.022$), driven by the input-validation |
| paper/main_access.tex:1364 | Results / Sensitivity and Cross-Model Heterogeneity | only the error-handling proxy. CodeLlama-34B passes the gate in 24 against 18 baseline and |
| paper/main_access.tex:1462 | Results / Additional Analyses: Frontier, Robustness, and Representation | non-trivial genuine repair is the locally insertable input-validation guard, repaired |
| paper/main_access.tex:1593 | Discussion / Comparison with Prior Findings | the repair loop reliably restored only the locally insertable input-validation guard, |
| paper/main_access.tex:1767 | Conclusion | rule set, while generated code still produces roughly twice as many rule firings as the |
| paper/main_access.tex:1843 | Replication and Preregistration Mapping | \ref{sec:exps} (270, 315 and 339 generations) was generated twice on the same inference |
| paper/main_access.tex:1865 | Replication and Preregistration Mapping | 10.07 against 10.37 and the safety-prompted mean 6.38 against 6.39; the pooled adversarial |
| paper/main_access.tex:1874 | Replication and Preregistration Mapping | 13.21 from a first-generation mean of 10.4 in both. In Table~\ref{tab:reps}, 12 against |
| paper/main_access.tex:1900 | Replication and Preregistration Mapping | checks carry SM-1 (input-validation proxy, rel. CWE-20), SM-2 (error-handling proxy, rel. CWE-252), SM-3 |
| paper/main_access.tex:1917 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-2 & Safe-zone halfspace test & 5.7.4 \\ |
| paper/main_access.tex:1918 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-3 & Orientation-cone test & 5.7.4 \\ |
| paper/main_access.tex:1921 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-5 & Frame consistency & 5.7.4 \\ |
| paper/main_access.tex:1922 | Replication and Preregistration Mapping [table from line 1905, tab:iso] | DM-5 & Cross-frame deviation & 5.7.4 \\ |
| paper/main_access.tex:1958 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Zone or workspace breach & DM-2 & SM-4 & 5.7.4; none \\ |
| paper/main_access.tex:1962 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Orientation breach & DM-3 & none & 5.7.4 \\ |
| paper/main_access.tex:1963 | Replication and Preregistration Mapping [table from line 1944, tab:hazard] | Frame inconsistency & DM-5 & none & 5.7.4 \\ |
| paper/tables/comparator_summary.tex:7 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | firing; mean is the mean number of firings per program. \emph{Preregistered} is the rule |
| paper/tables/comparator_summary.tex:21 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator & Preregistered & 15 & 15 & 15 & 5.7 & 0 & 0 & 0 & 15 & 15 & 15 & 0 \\ |
| paper/tables/comparator_summary.tex:24 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | Translator + preamble & Preregistered & 15 & 15 & 15 & 5.7 & 0 & 15 & 0 & 15 & 15 & 0 & 0 \\ |
| paper/tables/comparator_summary.tex:30 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | LLM baseline (E1, gate-passing) & Preregistered & 83 & 83 & 82 & 10.4 & 52 & 82 & 0 & 54 & 54 & 74 & 0 \\ |
| paper/tables/comparator_summary.tex:31 | Results / Baseline Security of Generated Code (\input at line 1209) [tab:comparator] | LLM baseline (E1, gate-passing) & Corrected SM-5 & 83 & 83 & 82 & 9.8 & 52 & 82 & 0 & 54 & 46 & 74 & 0 \\ |
| paper/tables/sensitivity_e1_baseline.tex:22 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & preregistered & 83 & 98.8 & 1/130 & 10.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:23 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & corrected-SM5 & 83 & 98.8 & 1/130 & 9.8 \\ |
| paper/tables/adversarial_table.tex:23 | Results / Adversarial Uplift (\input at line 1253) [tab:adversarial] | & \textit{Pooled} & 135 & 83 (61\%) & 60.7 & 98.8 & 6.38 & 10.37 & 0.88 \\ |
| paper/tables/e3_regimes.tex:27 | Results / Repair-in-Loop Defense (\input at line 1330) [tab:regimes] | \multicolumn{5}{@{}p{0.95\columnwidth}}{\textit{Fully valid cells (57): mean violations 10.4 at retry 0 and 13.2 at the last retry; per valid step 0.26 rules cleared and 0.29 introduced.}} \\ |
| paper/tables/prompts_appendix.tex:93 | Prompts Used in the Experiments (\input at line 2007) | Set the TCP speed to the value of 0x03E8 mm/s and joint acceleration to twice the default limit. Use numeric values directly, no named constants. |

## A4 (61 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:579 | The FACT Framework / Rule Checker | \textbf{Scope of the checks.} The checks are lexical rules: each is a regular expression, or a small set of them, evaluated over the URScript text, and the validity gate is itself a regular expression that matches one of |
| paper/main_access.tex:881 | Results [table from line 865, tab:summary] | \quad restricted to the checks rated $\geq$90\% in the first audit & 92.8\% & SM-4, SM-5, SM-6 only \\ |
| paper/main_access.tex:882 | Results [table from line 865, tab:summary] | \quad restricted to the check rated $\geq$90\% by rater B of the second audit & 89.2\% & SM-6 only; 9 of 130 violation-free \\ |
| paper/main_access.tex:883 | Results [table from line 865, tab:summary] | \quad restricted to the checks rated $\geq$90\% after adjudication & 98.8\% & SM-2 and SM-6; 1 of 130 violation-free \\ |
| paper/main_access.tex:908 | Results [table from line 898, tab:summary-extra] | Frontier violation rate, four models 24B to 675B & 100\% & 0 of 120 safe; falls to 75.8\% under the high-precision checks with corrected SM-5, 49.2\% under SM-6 only \\ |
| paper/main_access.tex:1021 | Results / Baseline Security of Generated Code | genuine safety-relevant defect of the kind the check names. Table~\ref{tab:precision} |
| paper/main_access.tex:1022 | Results / Baseline Security of Generated Code | reports the result. The hardcoded-value and missing-preamble checks are high precision |
| paper/main_access.tex:1025 | Results / Baseline Security of Generated Code | motion. The input-validation and error-handling proxies are low precision (12 and 0 |
| paper/main_access.tex:1036 | Results / Baseline Security of Generated Code [table from line 1034] | \caption{Per-rule precision of the rule checker on gate-passing generated code, |
| paper/main_access.tex:1039 | Results / Baseline Security of Generated Code [table from line 1034] | audited firings. The generation-level precision, counting a generation as a true |
| paper/main_access.tex:1041 | Results / Baseline Security of Generated Code [table from line 1034, tab:precision] | \label{tab:precision} |
| paper/main_access.tex:1057 | Results / Baseline Security of Generated Code | Because the two checks that this first audit rated low precision fire often (the |
| paper/main_access.tex:1066 | Results / Baseline Security of Generated Code | low-precision checks. The finding that essentially none of the generated code is |
| paper/main_access.tex:1067 | Results / Baseline Security of Generated Code | both valid and rule-clean therefore does not rest on the lower-precision checks but on |
| paper/main_access.tex:1068 | Results / Baseline Security of Generated Code | the hardcoded-speed and missing-preamble checks, whose precision the audit supports. |
| paper/main_access.tex:1073 | Results / Baseline Security of Generated Code | second audit was therefore conducted after the review (Table~\ref{tab:precision2}): 100 |
| paper/main_access.tex:1078 | Results / Baseline Security of Generated Code | before rater B's sheet was opened; rater B is not an author and had not seen the |
| paper/main_access.tex:1079 | Results / Baseline Security of Generated Code | manuscript, the rule code or the first audit. The same two authors adjudicated the 19 |
| paper/main_access.tex:1081 | Results / Baseline Security of Generated Code | rater A's label. Agreement is reported before adjudication: pooled Cohen's |
| paper/main_access.tex:1082 | Results / Baseline Security of Generated Code | $\kappa=0.55$ (81 of 100 labels agree), with full agreement on the unusual-conditions |
| paper/main_access.tex:1084 | Results / Baseline Security of Generated Code | agreement on the error-handling proxy ($\kappa=0.20$) and the hardcoded-value check |
| paper/main_access.tex:1085 | Results / Baseline Security of Generated Code | ($\kappa=0.35$); the $\kappa$ of 0.00 on the preamble check is an artefact of rater A |
| paper/main_access.tex:1087 | Results / Baseline Security of Generated Code | is systematically less favourable to the instrument than rater A: pooled precision 64\% |
| paper/main_access.tex:1089 | Results / Baseline Security of Generated Code | rater A's label because each of rater B's departures applied the rule's own wording |
| paper/main_access.tex:1090 | Results / Baseline Security of Generated Code | where the guide's criterion differs from it. The adjudicated column therefore coincides |
| paper/main_access.tex:1092 | Results / Baseline Security of Generated Code | rater B column as the conservative estimate. The disagreements fall into four groups, and |
| paper/main_access.tex:1095 | Results / Baseline Security of Generated Code | ``critical operation''; the guide's program-level criterion (no handling anywhere in the |
| paper/main_access.tex:1099 | Results / Baseline Security of Generated Code | whereas the written guide judges it as a program-level error-handling proxy, so the |
| paper/main_access.tex:1100 | Results / Baseline Security of Generated Code | move from 0 to 90 percent (50 percent under rater B) reflects a change of criterion, |
| paper/main_access.tex:1115 | Results / Baseline Security of Generated Code | the hardcoded-value and unusual-conditions checks at 85\% and 80\% after adjudication |
| paper/main_access.tex:1116 | Results / Baseline Security of Generated Code | (55\% and 80\% for rater B), against 90\% and 100\% in the first audit; its larger sample |
| paper/main_access.tex:1120 | Results / Baseline Security of Generated Code | rater's threshold is applied. Under rater B's labels only the preamble check reaches 90\% |
| paper/main_access.tex:1121 | Results / Baseline Security of Generated Code | precision, and restricting the violation flag to that one check leaves the gate-passing |
| paper/main_access.tex:1123 | Results / Baseline Security of Generated Code | violation-free (Table~\ref{tab:sensitivity}, SM6-only). Under the adjudicated labels the |
| paper/main_access.tex:1124 | Results / Baseline Security of Generated Code | error-handling proxy and the preamble check reach 90\%, and restricting the flag to those |
| paper/main_access.tex:1126 | Results / Baseline Security of Generated Code | rule set (Table~\ref{tab:sensitivity}, SM2+SM6). The finding that almost no generated |
| paper/main_access.tex:1130 | Results / Baseline Security of Generated Code | rule set rather than under a set of verified high-precision checks. A rule-by-rule |
| paper/main_access.tex:1136 | Results / Baseline Security of Generated Code | \input{tables/precision_audit_v2.tex} |
| paper/main_access.tex:1214 | Results / Baseline Security of Generated Code | checks the first audit rated at or above 90\% precision (SM-4, SM-5, SM-6) alone and with |
| paper/main_access.tex:1215 | Results / Baseline Security of Generated Code | the corrected SM-5; the rule set without SM-4; the one check rater B of the second audit |
| paper/main_access.tex:1216 | Results / Baseline Security of Generated Code | rated at or above 90\% (SM-6) and the two the adjudicated labels do (SM-2, SM-6); and a |
| paper/main_access.tex:1228 | Results / Baseline Security of Generated Code | violation rate of 100\% falls to 75.8\% under the high-precision set with the corrected |
| paper/main_access.tex:1229 | Results / Baseline Security of Generated Code | SM-5 and to 49.2\% under rater B's single check, while it stays at 98.3\% under the |
| paper/main_access.tex:1230 | Results / Baseline Security of Generated Code | adjudicated two-check set, so the frontier figure is reported with that caveat. |
| paper/main_access.tex:1625 | Discussion / Alternative Interpretations | insufficient. The saturation is not an artifact of low-precision quantization or |
| paper/main_access.tex:1626 | Discussion / Alternative Interpretations | near-deterministic decoding, since it persists for full-precision frontier models served |
| paper/main_access.tex:1681 | Discussion / Threats to Validity | variants scored against author-written rules, and its precision on generated code rests |
| paper/main_access.tex:1800 | Conclusion | physical execution, and its rule set, whose precision and composition are reported here, |
| paper/main_access.tex:2054 | Watchdog Detection Performance | attached. A separate five-task calibration pilot of 40 variants recorded 36 of 40 (90\%) |
| paper/main_access.tex:2059 | Watchdog Detection Performance | of independently sourced violations, and the only evidence on real-world precision is the |
| paper/tables/precision_audit_v2.tex:9 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Agree is percent agreement and $\kappa$ Cohen's kappa on the TP/FP decisions. Adjudicated is the label after a recorded resolution of every disagreement under the written rater guide, performed by the two rater A authors |
| paper/tables/precision_audit_v2.tex:11 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | \label{tab:precision2} |
| paper/tables/precision_audit_v2.tex:17 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Check & $n$ & \multicolumn{2}{c}{Rater A} & \multicolumn{2}{c}{Rater B} & Agree & $\kappa$ & \multicolumn{2}{c}{Adjudicated} \\ |
| paper/tables/precision_audit_v2.tex:25 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Safety preamble & 20 & 20 & 100.0 [83.2, 100.0] & 18 & 90.0 [68.3, 98.8] & 90\% & 0.00 & 20 & 100.0 [83.2, 100.0] \\ |
| paper/tables/sensitivity_e1_baseline.tex:7 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | non-URScript motion primitive. \emph{High-precision} keeps only the three checks the |
| paper/tables/sensitivity_e1_baseline.tex:8 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | first audit rated at or above 90\% precision; \emph{SM6-only} and \emph{SM2+SM6} keep the |
| paper/tables/sensitivity_e1_baseline.tex:9 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | checks rated at or above 90\% by rater B and by the adjudicated labels of the second |
| paper/tables/sensitivity_e1_baseline.tex:24 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & high-precision & 83 & 92.8 & 6/130 & 5.9 \\ |
| paper/tables/sensitivity_e1_baseline.tex:28 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM2+SM6 & 83 & 98.8 & 1/130 & 3.0 \\ |
| paper/tables/sensitivity_e1_baseline.tex:32 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & high-precision & 38 & 100.0 & 0/130 & 7.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:36 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & SM2+SM6 & 38 & 100.0 & 0/130 & 3.2 \\ |

## range (49 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:56 | abstract | code fails the checks: 89.2\% to 98.8\% of valid outputs violate at least one, and 1 to 9 of |
| paper/main_access.tex:172 | Introduction / Contributions | \item \textbf{A validity-aware outcome measure.} The substantive-safe rate, which |
| paper/main_access.tex:173 | Introduction / Contributions | counts only generations that are both valid and violation-free, is 1 of 135 at |
| paper/main_access.tex:880 | Results [table from line 865, tab:summary] | Combined violation rate, gate-passing outputs & 98.8\% & [93.5, 99.8], call level \\ |
| paper/main_access.tex:882 | Results [table from line 865, tab:summary] | \quad restricted to the check rated $\geq$90\% by rater B of the second audit & 89.2\% & SM-6 only; 9 of 130 violation-free \\ |
| paper/main_access.tex:883 | Results [table from line 865, tab:summary] | \quad restricted to the checks rated $\geq$90\% after adjudication & 98.8\% & SM-2 and SM-6; 1 of 130 violation-free \\ |
| paper/main_access.tex:884 | Results [table from line 865, tab:summary] | \quad with the corrected SM-5 unit reading & 98.8\% & Table~\ref{tab:sensitivity} \\ |
| paper/main_access.tex:947 | Results / Baseline Security of Generated Code | 98.8\% [93.5\%, 99.8\%] at the call level, and the per-model |
| paper/main_access.tex:972 | Results / Baseline Security of Generated Code | substantive-safe (passes the gate and triggers no check). The headline quantity is the |
| paper/main_access.tex:973 | Results / Baseline Security of Generated Code | substantive-safe rate, the fraction of all generations that are substantive-safe. Of |
| paper/main_access.tex:975 | Results / Baseline Security of Generated Code | gate-passing outputs, 82 are valid-unsafe and exactly one is substantive-safe, giving a |
| paper/main_access.tex:976 | Results / Baseline Security of Generated Code | substantive-safe rate of 1 of 135, or 0.7\% (Clopper-Pearson 95\% confidence interval |
| paper/main_access.tex:979 | Results / Baseline Security of Generated Code | motion commands. The contrast with the 98.8\% gate-passing violation rate is the point: |
| paper/main_access.tex:981 | Results / Baseline Security of Generated Code | invalid pseudo-code masquerade as safe, whereas the substantive-safe rate shows that |
| paper/main_access.tex:1064 | Results / Baseline Security of Generated Code | violation rate at 77 of 83, or 92.8 percent, against 98.8 percent for the full rule set, |
| paper/main_access.tex:1122 | Results / Baseline Security of Generated Code | violation rate at 74 of 83, or 89.2\%, with 9 rather than 1 of 130 baseline outputs |
| paper/main_access.tex:1125 | Results / Baseline Security of Generated Code | two leaves the rate at 98.8\% with 1 of 130 violation-free, the same as the registered |
| paper/main_access.tex:1128 | Results / Baseline Security of Generated Code | readings, but its margin ranges from 89.2\% to 98.8\% across defensible check sets, and |
| paper/main_access.tex:1129 | Results / Baseline Security of Generated Code | the substantive-safe rate of 1 of 135 must be read as a rate under the full registered |
| paper/main_access.tex:1157 | Results / Baseline Security of Generated Code | rate is 60.7\% (cell-level interval [46\%, 73\%]) and rises to 98.8\% on gate-passing |
| paper/main_access.tex:1161 | Results / Baseline Security of Generated Code | invalid pseudo-code as safe, which motivates the validity-aware substantive-safe rate. |
| paper/main_access.tex:1162 | Results / Baseline Security of Generated Code | \item Almost no generation is both valid and rule-clean: the substantive-safe rate is 1 |
| paper/main_access.tex:1188 | Results / Baseline Security of Generated Code | substantive-safe rate is reported with its rule composition rather than alone. Second, the |
| paper/main_access.tex:1221 | Results / Baseline Security of Generated Code | as invalid in the 1 of 135 rate. The gate-passing violation rate moves from 98.8\% to no |
| paper/main_access.tex:1222 | Results / Baseline Security of Generated Code | lower than 89.2\%, and the substantive-safe count from 1 to at most 9 of 130 scored |
| paper/main_access.tex:1262 | Results / Adversarial Uplift | mechanical consequence of the saturated baseline: the gate-passing baseline rate is 98.8\% |
| paper/main_access.tex:1417 | Results / Additional Analyses: Frontier, Robustness, and Representation | the substantive-safe count is 0 of 120, with no refusals. Mean violations per |
| paper/main_access.tex:1442 | Results / Additional Analyses: Frontier, Robustness, and Representation | temperatures, for both models, and under both conditions, and the substantive-safe rate |
| paper/main_access.tex:1559 | Discussion | implications, the validity-aware substantive-safe rate, and the design requirements for |
| paper/main_access.tex:1566 | Discussion / Comparison with Prior Findings | whereas under the validity gate 89.2\% to 98.8\% of baseline URScript generations contain at least |
| paper/main_access.tex:1567 | Discussion / Comparison with Prior Findings | one violation and only 1 to 9 programs in 135 are both valid and rule-clean. Three factors |
| paper/main_access.tex:1641 | Discussion / Alternative Interpretations | verification layer, and that the validity-gated metric, the substantive-safe rate for |
| paper/main_access.tex:1647 | Discussion / Implications for Deployment | Four implications follow. First, safety prompting helps two of three models but is not universally protective, and a unit-naive safety prompt can even elicit more dangerous code through a millimetre-versus-metre confusio |
| paper/main_access.tex:1655 | Discussion / Threats to Validity | ceiling and, scored naively, admits invalid output as safe; the substantive-safe rate |
| paper/main_access.tex:1741 | Discussion / EU AI Act Alignment | contributes a validity-aware substantive-safe rate with exact binomial testing rather |
| paper/main_access.tex:1764 | Conclusion | Three findings emerge. First, generated code pervasively fails the checks: 89.2\% to 98.8\% of |
| paper/main_access.tex:1765 | Conclusion | gate-passing baseline outputs violate at least one, and 1 to 9 of 135 baseline generations are |
| paper/main_access.tex:1770 | Conclusion | substantive-safe rate as the primary outcome, and it leaves the preregistered |
| paper/main_access.tex:1778 | Conclusion | to 675B parameters, the substantive-safe rate stays at or near zero and the repair loop |
| paper/main_access.tex:1861 | Replication and Preregistration Mapping | 135 at baseline, 192 of 315 adversarial), the single substantive-safe program (the same |
| paper/tables/precision_audit_v2.tex:22 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Error-handling proxy (CWE-252) & 20 & 18 & 90.0 [68.3, 98.8] & 10 & 50.0 [27.2, 72.8] & 60\% & 0.20 & 18 & 90.0 [68.3, 98.8] \\ |
| paper/tables/precision_audit_v2.tex:25 | Results / Baseline Security of Generated Code (\input at line 1136) [tab:precision2] | Safety preamble & 20 & 20 & 100.0 [83.2, 100.0] & 18 & 90.0 [68.3, 98.8] & 90\% & 0.00 & 20 & 100.0 [83.2, 100.0] \\ |
| paper/tables/sensitivity_e1_baseline.tex:13 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | substantive-safe outputs over all scored outputs, Mean the mean firings per admitted output.} |
| paper/tables/sensitivity_e1_baseline.tex:22 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & preregistered & 83 & 98.8 & 1/130 & 10.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:23 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & corrected-SM5 & 83 & 98.8 & 1/130 & 9.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:26 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & drop-SM4 & 83 & 98.8 & 1/130 & 9.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:27 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM6-only & 83 & 89.2 & 9/130 & 1.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:28 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM2+SM6 & 83 & 98.8 & 1/130 & 3.0 \\ |
| paper/tables/adversarial_table.tex:23 | Results / Adversarial Uplift (\input at line 1253) [tab:adversarial] | & \textit{Pooled} & 135 & 83 (61\%) & 60.7 & 98.8 & 6.38 & 10.37 & 0.88 \\ |

## denominator (50 lines)

| file:line | place | text |
|---|---|---|
| paper/main_access.tex:57 | abstract | 135 baseline generations are valid and violation-free; a non-LLM comparator shows that part |
| paper/main_access.tex:173 | Introduction / Contributions | counts only generations that are both valid and violation-free, is 1 of 135 at |
| paper/main_access.tex:716 | Experimental Setup / Experiments [table from line 703, tab:budget] | Baseline & 270 & 135 & 3 & 2 & 191 \\ |
| paper/main_access.tex:718 | Experimental Setup / Experiments [table from line 703, tab:budget] | Repair-in-loop & 339 & 135 & 3 & 1 & 261 \\ |
| paper/main_access.tex:823 | Experimental Setup / Statistical Analysis | \textbf{Statistical units and dependence.} The baseline corpus of 135 generations is |
| paper/main_access.tex:827 | Experimental Setup / Statistical Analysis | the 45 model-task cells and the 135 calls. Point estimates are unaffected by this |
| paper/main_access.tex:879 | Results [table from line 865, tab:summary] | Substantive-safe rate (primary) & 1/135 (0.7\%) & [0.02, 4.06], Clopper-Pearson \\ |
| paper/main_access.tex:882 | Results [table from line 865, tab:summary] | \quad restricted to the check rated $\geq$90\% by rater B of the second audit & 89.2\% & SM-6 only; 9 of 130 violation-free \\ |
| paper/main_access.tex:883 | Results [table from line 865, tab:summary] | \quad restricted to the checks rated $\geq$90\% after adjudication & 98.8\% & SM-2 and SM-6; 1 of 130 violation-free \\ |
| paper/main_access.tex:885 | Results [table from line 865, tab:summary] | \quad under the structural validity gate & 100\% & 38 of 130 outputs admitted \\ |
| paper/main_access.tex:892 | Results [table from line 865, tab:summary] | Cells remediated to zero violations & 0/135 & Table~\ref{tab:regimes} \\ |
| paper/main_access.tex:937 | Results / Baseline Security of Generated Code | identical at the level of the verdict, the 45 model-task cells rather than the 135 calls |
| paper/main_access.tex:974 | Results / Baseline Security of Generated Code | the 135 baseline generations, 83 pass the validity gate and 52 do not; of the 83 |
| paper/main_access.tex:976 | Results / Baseline Security of Generated Code | substantive-safe rate of 1 of 135, or 0.7\% (Clopper-Pearson 95\% confidence interval |
| paper/main_access.tex:993 | Results / Baseline Security of Generated Code [table from line 992] | \caption{Per-rule firing counts over the 135 baseline generations and the 135 |
| paper/main_access.tex:1122 | Results / Baseline Security of Generated Code | violation rate at 74 of 83, or 89.2\%, with 9 rather than 1 of 130 baseline outputs |
| paper/main_access.tex:1125 | Results / Baseline Security of Generated Code | two leaves the rate at 98.8\% with 1 of 130 violation-free, the same as the registered |
| paper/main_access.tex:1129 | Results / Baseline Security of Generated Code | the substantive-safe rate of 1 of 135 must be read as a rate under the full registered |
| paper/main_access.tex:1147 | Results / Baseline Security of Generated Code | unchanged. On the all-outputs basis the same means are 6.38 and 5.11 (135 generations each). This |
| paper/main_access.tex:1163 | Results / Baseline Security of Generated Code | of 135 (0.7\%), since of the 83 gate-passing outputs only one triggers no check. |
| paper/main_access.tex:1219 | Results / Baseline Security of Generated Code | non-URScript motion primitive. Of the 135 baseline generations, 130 have a stored |
| paper/main_access.tex:1221 | Results / Baseline Security of Generated Code | as invalid in the 1 of 135 rate. The gate-passing violation rate moves from 98.8\% to no |
| paper/main_access.tex:1222 | Results / Baseline Security of Generated Code | lower than 89.2\%, and the substantive-safe count from 1 to at most 9 of 130 scored |
| paper/main_access.tex:1247 | Results / Adversarial Uplift | 61\% (83 of 135 at baseline; 192 of 315 pooled over strategies), so the strategies do not |
| paper/main_access.tex:1317 | Results / Repair-in-Loop Defense | classifies every one of the 135 local-model cells by the shape of its violation-count |
| paper/main_access.tex:1318 | Results / Repair-in-Loop Defense | sequence. Of the 135 cells, 78 leave the loop by dropping out of valid syntax, 52 of them |
| paper/main_access.tex:1453 | Results / Additional Analyses: Frontier, Robustness, and Representation | across the 135 confirmatory cells and 60 frontier cells, yields a remediated-to-zero |
| paper/main_access.tex:1454 | Results / Additional Analyses: Frontier, Robustness, and Representation | count of 0 of 135 and 0 of 60: no model at any size from 16B to 675B used the retry |
| paper/main_access.tex:1457 | Results / Additional Analyses: Frontier, Robustness, and Representation | behavior is that the loop never runs or breaks early, with 78 of 135 cells either |
| paper/main_access.tex:1567 | Discussion / Comparison with Prior Findings | one violation and only 1 to 9 programs in 135 are both valid and rule-clean. Three factors |
| paper/main_access.tex:1765 | Conclusion | gate-passing baseline outputs violate at least one, and 1 to 9 of 135 baseline generations are |
| paper/main_access.tex:1861 | Replication and Preregistration Mapping | 135 at baseline, 192 of 315 adversarial), the single substantive-safe program (the same |
| paper/tables/sensitivity_e1_baseline.tex:4 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | E1 baseline ($n$=130 generations), re-scored offline from the stored outputs. |
| paper/tables/sensitivity_e1_baseline.tex:22 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & preregistered & 83 & 98.8 & 1/130 & 10.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:23 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & corrected-SM5 & 83 & 98.8 & 1/130 & 9.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:24 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & high-precision & 83 & 92.8 & 6/130 & 5.9 \\ |
| paper/tables/sensitivity_e1_baseline.tex:25 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & high-prec+corr & 83 & 91.6 & 7/130 & 5.3 \\ |
| paper/tables/sensitivity_e1_baseline.tex:26 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & drop-SM4 & 83 & 98.8 & 1/130 & 9.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:27 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM6-only & 83 & 89.2 & 9/130 & 1.7 \\ |
| paper/tables/sensitivity_e1_baseline.tex:28 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | lexical & SM2+SM6 & 83 & 98.8 & 1/130 & 3.0 \\ |
| paper/tables/sensitivity_e1_baseline.tex:30 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & preregistered & 38 & 100.0 & 0/130 & 11.6 \\ |
| paper/tables/sensitivity_e1_baseline.tex:31 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & corrected-SM5 & 38 & 100.0 & 0/130 & 10.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:32 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & high-precision & 38 & 100.0 & 0/130 & 7.4 \\ |
| paper/tables/sensitivity_e1_baseline.tex:33 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & high-prec+corr & 38 & 97.4 & 1/130 & 6.6 \\ |
| paper/tables/sensitivity_e1_baseline.tex:34 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & drop-SM4 & 38 & 100.0 & 0/130 & 11.2 \\ |
| paper/tables/sensitivity_e1_baseline.tex:35 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & SM6-only & 38 & 92.1 & 3/130 & 1.8 \\ |
| paper/tables/sensitivity_e1_baseline.tex:36 | Results / Baseline Security of Generated Code (\input at line 1232) [tab:sensitivity] | structural & SM2+SM6 & 38 & 100.0 & 0/130 & 3.2 \\ |
| paper/tables/adversarial_table.tex:23 | Results / Adversarial Uplift (\input at line 1253) [tab:adversarial] | & \textit{Pooled} & 135 & 83 (61\%) & 60.7 & 98.8 & 6.38 & 10.37 & 0.88 \\ |
| paper/tables/e3_regimes.tex:3 | Results / Repair-in-Loop Defense (\input at line 1330) [tab:regimes] | \caption{Repair-in-loop trajectory regimes for the 135 local-model cells of E3 |
| paper/tables/e3_regimes.tex:26 | Results / Repair-in-Loop Defense (\input at line 1330) [tab:regimes] | Cells & 45 & 45 & 45 & 135 \\ |
