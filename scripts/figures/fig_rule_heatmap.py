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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

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
    for cond, cname in (("baseline", "baseline"), ("safety", "safety-prompted")):
        for model, mlabel in MODEL_LABEL.items():
            g = e1[(e1.condition == cond) & (e1.model == model)]
            rows.append([np.mean([r in rs for rs in g.rules]) if len(g) else np.nan for r in RULES])
            labels.append(f"{mlabel}\n{cname}, n={len(g)}")
    if args.comparator.is_file():
        c = pd.read_csv(args.comparator)
        for set_name, short in (("Translator", "Reference\ntranslator"), ("Vendor / community examples", "Vendor/community\nexamples")):
            g = c[(c.set == set_name) & (c.gate == 1)]
            if len(g):
                rows.append([(g[f"pre_{r}"] > 0).mean() for r in RULES])
                labels.append(f"{short}, n={len(g)}")
    m = np.array(rows, dtype=float)

    plt.rcParams.update({"font.size": 7.5, "font.family": "serif"})
    fig, ax = plt.subplots(figsize=(3.5, 0.42 * len(rows) + 1.0), dpi=300)
    im = ax.imshow(m, cmap="Greys", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(RULES)))
    ax.set_xticklabels(RULES, fontsize=6.2)
    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(labels, fontsize=5.8)
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            v = m[i, j]
            if np.isnan(v):
                continue
            ax.text(j, i, f"{100*v:.0f}", ha="center", va="center", fontsize=6,
                    color="white" if v > 0.55 else "black")
    ax.axhline(5.5, color="black", linewidth=0.8)
    if len(rows) > 6:
        ax.axhline(len(rows) - 2.5 if len(rows) == 8 else len(rows) - 1.5, color="black", linewidth=0.5, linestyle=":")
    ax.set_xlabel("Security check; cell = % of gate-passing outputs on which it fires", fontsize=6)
    ax.tick_params(length=0)
    for s in ax.spines.values():
        s.set_visible(False)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out)
    fig.savefig(args.out.with_suffix(".png"))
    print(pd.DataFrame(m, index=labels, columns=RULES).round(2).to_string())
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
