# "on which the reference never fires"

paper/main_access.tex lines 1188-1192:
```
1188: substantive-safe rate is reported with its rule composition rather than alone. Second, the
1189: count and the rule profile do separate generated code from the reference: gate-passing
1190: generated code produces 10.4 firings per program against 5.7 for the reference translation,
1191: and it fires on the input-validation and error-handling proxies (52 and 82 of 83 programs)
1192: on which the reference never fires (Figure~\ref{fig:heatmap}). Third, correcting the SM-5
```

Programs firing SM-1 and SM-2 (preregistered rule set), from out/a3/per_rule_means.csv:

| set | n | SM-1 programs | SM-2 programs | mean SM-2 global | mean SM-2 per-line |
|---|---|---|---|---|---|
| LLM baseline (E1, run 2, gate-passing) | 83 | 52 | 82 | 0.99 | 0.35 |
| Translator | 15 | 0 | 0 | 0.00 | 0.00 |
| Translator + preamble | 15 | 0 | 15 | 0.00 | 2.00 |
