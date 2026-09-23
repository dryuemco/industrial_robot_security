#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: per-model x per-rule firing rates (E1 baseline vs safety-prompted).

Extra analysis X12. For each model and condition, the share of gate-passing
generations on which each security check fires, drawn as a grayscale
heat-map with the percentage printed in every cell, so it reads in print
without colour. The reference translator output is added as a bottom row
for comparison (revision item R2-1).

Usage:
    python3 scripts/figures/fig_rule_heatmap.py --e1 results/E1_full/e1_results.csv \
        --comparator paper/tables/comparator_programs.csv \
        --out paper/figures/fig_rule_heatmap.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

import _style
from _style import plt

RULES = ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]
RULE_SHORT = {
    "SM-1": "SM-1\ninput val.", "SM-2": "SM-2\nerror hand.", "SM-3": "SM-3\nprotection",
    "SM-4": "SM-4\nunusual cond.", "SM-5": "SM-5\nhardcoded", "SM-6": "SM-6\npreamble", "SM-7": "SM-7\ninjection",
}
MODEL_LABEL = {
    "qwen2.5-coder:32b": "Qwen 32B",
    "deepseek-coder-v2:16b": "DeepSeek 16B",
    "codellama:34b": "CodeLlama 34B",
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e1", type=Path, default=Path("results/E1_full/e1_results.csv"))
    ap.add_argument("--comparator", type=Path, default=Path("paper/tables/comparator_programs.csv"))
    ap.add_argument("--out", type=Path, default=Path("paper/figures/fig_rule_heatmap.pdf"))
    args = ap.parse_args()

    e1 = pd.read_csv(args.e1)
    e1 = e1[e1.status == "success"].copy()
    e1["rules"] = e1.violation_types.fillna("").astype(str).str.split(",")

    rows, labels = [], []
    for cond in ("baseline", "safety"):
        for model, mlabel in MODEL_LABEL.items():
            g = e1[(e1.condition == cond) & (e1.model == model)]
            rows.append([np.mean([r in rs for rs in g.rules]) if len(g) else np.nan for r in RULES])
            labels.append(f"{mlabel}, n={len(g)}")
    if args.comparator.is_file():
        c = pd.read_csv(args.comparator)
        for set_name, short in (("Translator", "Reference translator"), ("Vendor / community examples", "Vendor/community")):
            g = c[(c.set == set_name) & (c.gate == 1)]
            if len(g):
                rows.append([(g[f"pre_{r}"] > 0).mean() for r in RULES])
                labels.append(f"{short}, n={len(g)}")
    m = np.array(rows, dtype=float)

    _style.apply()
    fig, ax = plt.subplots(figsize=(_style.COL_W, 0.24 * len(rows) + 0.75), constrained_layout=True)
    ax.imshow(m, cmap="Greys", vmin=-0.06, vmax=1.15, aspect="auto")
    ax.set_xticks(range(len(RULES)))
    ax.set_xticklabels(RULES, fontsize=7)
    ax.xaxis.tick_top()
    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(labels, fontsize=7)
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            v = m[i, j]
            if np.isnan(v):
                continue
            ax.text(j, i, f"{100*v:.0f}", ha="center", va="center", fontsize=7,
                    color="white" if v > 0.5 else "black")
    # white gaps between cells, a rule between the two conditions, a heavier rule
    # between generated code and the comparison programs
    ax.set_xticks(np.arange(-0.5, len(RULES)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(rows)), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.2)
    ax.tick_params(which="minor", length=0)
    ax.axhline(2.5, color=_style.INK, linewidth=0.5, xmin=-0.02, xmax=1.08, clip_on=False)
    ax.axhline(5.5, color=_style.INK, linewidth=1.0, xmin=-0.02, xmax=1.08, clip_on=False)
    groups = (("Baseline", 0, 2), ("Safety-\nprompted", 3, 5), ("Comparison", 6, len(rows) - 1))
    for name, a, b in groups:
        ax.text(len(RULES) - 0.3, (a + b) / 2, name, rotation=270, ha="left", va="center",
                fontsize=7, linespacing=1.0)
    ax.set_xlabel("% of gate-passing outputs on which the check fires", fontsize=7)
    ax.tick_params(length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    _style.save(fig, args.out)
    print(pd.DataFrame(m, index=labels, columns=RULES).round(2).to_string())


if __name__ == "__main__":
    main()
