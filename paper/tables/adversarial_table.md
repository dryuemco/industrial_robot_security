| Strategy | Model | n | Gate-pass | Viol. rate (all) | Viol. rate (gate) | Mean viol. (all) | Mean viol. (gate) | Sev. max (gate) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Baseline (E1, 3 reps) | Qwen2.5-Coder-32B | 45 | 45 (100%) | 100.0% | 100.0% | 10.91 | 10.91 | 0.92 |
| Baseline (E1, 3 reps) | DeepSeek-Coder-V2-16B | 45 | 20 (44%) | 42.2% | 95.0% | 4.22 | 9.50 | 0.68 |
| Baseline (E1, 3 reps) | CodeLlama-34B | 45 | 18 (40%) | 40.0% | 100.0% | 4.00 | 10.00 | 1.00 |
| Baseline (E1, 3 reps) | pooled | 135 | 83 (61%) | 60.7% | 98.8% | 6.38 | 10.37 | 0.88 |
| A8.1 Direct override | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 12.20 | 12.20 | 0.98 |
| A8.1 Direct override | DeepSeek-Coder-V2-16B | 15 | 6 (40%) | 40.0% | 100.0% | 3.00 | 7.50 | 0.80 |
| A8.1 Direct override | CodeLlama-34B | 15 | 5 (33%) | 33.3% | 100.0% | 4.13 | 12.40 | 1.00 |
| A8.1 Direct override | pooled | 45 | 26 (58%) | 57.8% | 100.0% | 6.44 | 11.15 | 0.94 |
| A8.2 Role-playing | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 10.93 | 10.93 | 0.93 |
| A8.2 Role-playing | DeepSeek-Coder-V2-16B | 15 | 9 (60%) | 60.0% | 100.0% | 3.47 | 5.78 | 0.70 |
| A8.2 Role-playing | CodeLlama-34B | 15 | 4 (27%) | 26.7% | 100.0% | 2.93 | 11.00 | 1.00 |
| A8.2 Role-playing | pooled | 45 | 28 (62%) | 62.2% | 100.0% | 5.78 | 9.29 | 0.86 |
| A8.3 Context overflow | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 11.00 | 11.00 | 1.00 |
| A8.3 Context overflow | DeepSeek-Coder-V2-16B | 15 | 7 (47%) | 46.7% | 100.0% | 3.73 | 8.00 | 0.70 |
| A8.3 Context overflow | CodeLlama-34B | 15 | 7 (47%) | 46.7% | 100.0% | 4.53 | 9.71 | 1.00 |
| A8.3 Context overflow | pooled | 45 | 29 (64%) | 64.4% | 100.0% | 6.42 | 9.97 | 0.93 |
| A8.4 Incremental | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 22.73 | 22.73 | 1.00 |
| A8.4 Incremental | DeepSeek-Coder-V2-16B | 15 | 6 (40%) | 40.0% | 100.0% | 2.73 | 6.83 | 0.75 |
| A8.4 Incremental | CodeLlama-34B | 15 | 6 (40%) | 40.0% | 100.0% | 4.40 | 11.00 | 0.95 |
| A8.4 Incremental | pooled | 45 | 27 (60%) | 60.0% | 100.0% | 9.96 | 16.59 | 0.93 |
| A8.5 Authority claim | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 13.53 | 13.53 | 0.96 |
| A8.5 Authority claim | DeepSeek-Coder-V2-16B | 15 | 11 (73%) | 73.3% | 100.0% | 5.60 | 7.64 | 0.70 |
| A8.5 Authority claim | CodeLlama-34B | 15 | 5 (33%) | 33.3% | 100.0% | 3.07 | 9.20 | 1.00 |
| A8.5 Authority claim | pooled | 45 | 31 (69%) | 68.9% | 100.0% | 7.40 | 10.74 | 0.87 |
| A8.6 Performance framing | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 12.73 | 12.73 | 0.96 |
| A8.6 Performance framing | DeepSeek-Coder-V2-16B | 15 | 5 (33%) | 33.3% | 100.0% | 4.60 | 13.80 | 0.82 |
| A8.6 Performance framing | CodeLlama-34B | 15 | 5 (33%) | 33.3% | 100.0% | 2.40 | 7.20 | 0.94 |
| A8.6 Performance framing | pooled | 45 | 25 (56%) | 55.6% | 100.0% | 6.58 | 11.84 | 0.93 |
| A8.7 Obfuscation | Qwen2.5-Coder-32B | 15 | 15 (100%) | 100.0% | 100.0% | 12.60 | 12.60 | 1.00 |
| A8.7 Obfuscation | DeepSeek-Coder-V2-16B | 15 | 5 (33%) | 33.3% | 100.0% | 2.13 | 6.40 | 0.70 |
| A8.7 Obfuscation | CodeLlama-34B | 15 | 6 (40%) | 40.0% | 100.0% | 6.20 | 15.50 | 0.85 |
| A8.7 Obfuscation | pooled | 45 | 26 (58%) | 57.8% | 100.0% | 6.98 | 12.08 | 0.91 |
