#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: silent removal of the encoded speed-cap construct by model scale (E5).

Revision item R2-7b: the submitted figure drew a dashed trend guide through
four points at one repetition per cell. This version shows the four points
with exact (Clopper-Pearson) 95% intervals and the n per point, no fitted
line, and labels the axis as ordinal model size. Grayscale-safe.

Usage:
    python3 scripts/figures/fig_construct_removal.py \
        --e5 results/exploratory/e5_repr_fragility/e5_results.csv \
        --out paper/figures/fig_construct_removal.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from scipy.stats import beta

import _style
from _style import plt

# ordered by parameter count; label, size in B
MODELS = [
    ("mistralai/Devstral-Small-2-24B-Instruct-2512", "Devstral-Small-2", 24),
    ("openai/gpt-oss-120b", "gpt-oss", 120),
    ("zai-org/GLM-4.7-FP8", "GLM-4.7", 358),
    ("mistralai/Mistral-Large-3-675B-Instruct-2512-NVFP4", "Mistral-Large-3", 675),
]
EXPLICIT = {"R3_named", "R4_guard"}


def cp(k: int, n: int) -> tuple[float, float]:
    lo = 0.0 if k == 0 else beta.ppf(0.025, k, n - k + 1)
    hi = 1.0 if k == n else beta.ppf(0.975, k + 1, n - k)
    return lo, hi


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e5", type=Path, default=Path("results/exploratory/e5_repr_fragility/e5_results.csv"))
    ap.add_argument("--out", type=Path, default=Path("paper/figures/fig_construct_removal.pdf"))
    args = ap.parse_args()

    d = pd.read_csv(args.e5)
    d = d[d.representation.isin(EXPLICIT) & (d.status == "success")]
    rows = []
    for key, label, size in MODELS:
        g = d[d.model == key]
        n = len(g)
        k = int((~g.mechanism_survived.astype(bool)).sum())
        lo, hi = cp(k, n)
        rows.append({"model": label, "size_b": size, "n": n, "removed": k, "rate": k / n, "lo": lo, "hi": hi})
    t = pd.DataFrame(rows)

    _style.apply()
    fig, ax = plt.subplots(figsize=(_style.COL_W, 2.3), constrained_layout=True)
    x = range(len(t))
    ax.errorbar(list(x), 100 * t.rate, yerr=[100 * (t.rate - t.lo), 100 * (t.hi - t.rate)],
                fmt="o", color=_style.INK, ecolor=_style.MID, elinewidth=0.9, capsize=3, markersize=5,
                markerfacecolor="white", markeredgewidth=1.2, clip_on=False, zorder=3)
    for i, r in t.iterrows():
        ax.annotate(f"{r.removed}/{r.n}", (i, 100 * r.rate), xytext=(7, 0 if r.rate > 0.05 else 5),
                    textcoords="offset points", fontsize=7, va="center" if r.rate > 0.05 else "bottom")
    ax.set_xticks(list(x))
    ax.set_xticklabels([f"{r.model}\n{r.size_b}B" for _, r in t.iterrows()], fontsize=7)
    ax.tick_params(axis="x", length=0)
    ax.set_xlabel("Model, ordered by parameter count (ordinal axis)")
    ax.set_ylabel("Construct removed (%)")
    ax.set_ylim(0, 100)
    ax.set_xlim(-0.5, len(t) - 0.5)
    _style.grid_y(ax)
    ax.text(0.02, 0.99, "one repetition per cell; exact 95% intervals; no trend fitted",
            transform=ax.transAxes, fontsize=7, va="top", color="0.3")
    _style.save(fig, args.out)
    print(t.to_string(index=False))


if __name__ == "__main__":
    main()
