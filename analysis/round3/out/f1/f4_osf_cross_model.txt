# OSF Amendment 1: the cross-model test and the condition labels, verbatim

Source: `out/osf/schema_response_69d3c5c421a5eb766f7a850e.json` (registration ve5m2, Amendment 1, created 2026-04-06, state approved), field `revision_responses`. HTML entities are unescaped; nothing else is changed. The last column of each heading says whether the same text is in the three later versions (2026-05-12, 2026-05-15, 2026-09-21).

## Field 344-58, the conditions (unchanged later: yes)
```
Prompt Condition (within-subject across E1/E2/E3):

Neutral (E1): Plain task description, no safety guidance, no adversarial content.
Safety-augmented (E1): Task description prefixed with explicit ISO 10218:2025 safety guidance.
Adversarial A6.1–A6.8 (E2): Task description with one of 8 prompt-injection variants embedded in the task metadata, applied deterministically per task.
Watchdog-in-the-loop (E3): Initial generation under neutral prompt; if analyze_combined() reports any violation, the structured violation report is returned to the LLM as feedback for one retry.
```

## Field 344-62, the model variable (unchanged later: yes)
```
LLM Model Identifier:

Type: Categorical (qwen2.5-coder:32b, deepseek-coder-v2:16b, codellama:34b).
Used for: Stratified analysis and Cochran's Q cross-model comparison.
```

## Field 344-62, the condition variable (unchanged later: yes)
```
Prompt Condition:

Type: Categorical (neutral, safety_augmented, A6.1, …, A6.8, watchdog_in_loop).
Used for: Within-model comparisons across E1/E2/E3.
```

## Field 344-66, the cross-model index (unchanged later: yes)
```
Cross-Model Cochran's Q (exploratory):
Cochran's Q applied across the 3 models on per-task matched outcomes, used to test whether models differ systematically in violation rates within each condition. Reported as exploratory; not used to reject H4–H6.
```

## Field 344-71, the analysis plan (unchanged later: yes)
```
--- Cross-Model Comparison (Exploratory) ---
Cochran's Q test across the 3 models on per-task matched outcomes within each prompt condition. Reports whether models differ systematically. Exploratory only; not used to evaluate H4–H6.
```

## Field 344-75, the condition labels (unchanged later: yes)
```
Prompt Condition: Nominal, stored as string labels: neutral, safety_augmented, A6.1..A6.8, watchdog_in_loop. No dummy coding; analyses are performed per condition.
```

## Field 344-77, the inference criteria (unchanged later: yes)
```
--- Cross-Model Comparison (Exploratory) ---

Cochran's Q across the 3 models per condition; reported with p-value, no correction; clearly labeled exploratory
```

