# Figures: captions and source scripts

## line 440: fig:pipeline
- image: figures/pipeline.pdf
- drawn by: scripts/figures/fig_pipeline.py
- caption: The FACT pipeline. A vendor-neutral task representation is rendered into a baseline, safety-augmented, or adversarial prompt; an open-weight code model generates URScript; a lexical rule checker of seven specification-level motion-safety checks and seven weakness-mapped code-level security checks, traceable to seven clauses of ISO 10218-1:2025, emits a violation verdict. A repair-in-loop variant feeds detected violations back as repair prompts.

## line 466: fig:taskjson
- image: 
- drawn by: not found by file name
- caption: Abbreviated vendor-neutral JSON task representation for task T001 (pick-place, collaborative mode).

## line 957: fig:per-model
- image: figures/fig_violation_rates.pdf
- drawn by: scripts/figures/fig_violation_rates.py
- caption: Combined violation rate for each model under the baseline condition, computed over all generations (naive) and over only the generations that pass the validity gate, with Wilson 95\% intervals. For DeepSeek-Coder-V2-16B and CodeLlama-34B the gap between the bars reflects the high rate of invalid pseudo-code, which the naive rate scores as zero-violation.

## line 1200: fig:heatmap
- image: figures/fig_rule_heatmap.pdf
- drawn by: scripts/figures/fig_rule_heatmap.py
- caption: Share of gate-passing outputs on which each security check fires, by model and condition in the baseline experiment, with the reference translation and the vendor and community programs as comparison rows. The protection-mechanism (SM-3) and prompt-injection-marker (SM-7) checks never fire on generated code.

## line 1303: fig:retry
- image: figures/fig_repair_trajectory.pdf
- drawn by: scripts/figures/fig_repair_trajectory.py
- caption: Representative self-repair trajectories under rule-checker feedback for Qwen2.5-Coder-32B, the one model that maintains valid output throughout, showing total violations against retry index, one panel per regime on a shared scale. The panels show the six regimes of Table~\ref{tab:regimes}: monotone reduction that does not reach zero, net reduction with one reversal, two-state oscillation in which each retry undoes the previous fix, transient improvement followed by regression, monotone increase, and an invariant trajectory. In each panel the dark curve is a representative cell, labelled with its task, and the gray curves are the other Qwen2.5-Coder-32B cells of that regime (the three repetitions of a task often coincide); the cell count in each panel title is that model's column of the table.

## line 1526: fig:repbreach
- image: figures/fig_representation_breach.pdf
- drawn by: scripts/figures/fig_representation_breach.py
- caption: Speed-cap breach rate after one benign performance-framed edit, by how the cap is written, over 240 generations. The four encodings are ordered from least to most explicit. Breach falls monotonically with legibility (Cochran-Armitage trend statistic 4.03, p below $10^{-4}$); counts are shown above each bar. The two zero-breach forms are qualified by construct survival in the text: in the two explicit forms the encoding was deleted in 41\% of programs yet stayed within the cap only because the residual literal speeds happened to be safe.

## line 1538: fig:construct
- image: figures/fig_construct_removal.pdf
- drawn by: scripts/figures/fig_construct_removal.py
- caption: Silent removal of the encoded speed-cap construct after the same benign edit, by model scale, across four ordered sizes (one repetition per cell; exact 95\% Clopper-Pearson intervals; counts shown as removed/valid). No trend is fitted: four sizes at one repetition each support no inferential statistic, and the middle two points are not ordered (50.0\% at 120B against 44.8\% at 358B). The smallest model removed the construct in none of its cells and the largest in 70\%, inverting the expectation that larger models are safer.
