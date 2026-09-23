#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: speed-cap breach rate after one benign edit, by constraint encoding (E5).

For each of the four encodings of the speed cap, ordered from least to most
explicit, the share of edited programs that can command a TCP speed above the
cap, with the counts above each bar. Pooled over the four frontier models;
outputs that failed to parse are excluded. Grayscale-safe.

Usage:
    python3 scripts/figures/fig_representation_breach.py \
        --e5 results/exploratory/e5_repr_fragility/e5_results.csv \
        --out paper/figures/fig_representation_breach.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from matplotlib.ticker import PercentFormatter

import _style
from _style import plt

ENCODINGS = [
    ("R1_implicit", "Implicit\nliterals"),
    ("R2_comment", "Comment"),
    ("R3_named", "Constant\n+ clamp"),
    ("R4_guard", "Runtime\nguard"),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e5", type=Path, default=Path("results/exploratory/e5_repr_fragility/e5_results.csv"))
    ap.add_argument("--out", type=Path, default=Path("paper/figures/fig_representation_breach.pdf"))
    args = ap.parse_args()

    e5 = pd.read_csv(args.e5)
    e5 = e5[e5.status == "success"]
    rows = []
    for key, label in ENCODINGS:
        g = e5[e5.representation == key]
        k = int((~g.tcp_safe.astype(bool)).sum())
        rows.append({"encoding": label, "k": k, "n": len(g), "rate": k / len(g)})
    t = pd.DataFrame(rows)

    _style.apply()
    fig, ax = plt.subplots(figsize=(_style.COL_W, 2.2), constrained_layout=True)
    x = range(len(t))
    ax.bar(x, 100 * t.rate, 0.6, color=_style.FILL_DARK, edgecolor=_style.INK, linewidth=0.6)
    for i, r in t.iterrows():
        pct = f"{100 * r.rate:.1f}%" if 0 < r.rate < 0.1 else f"{100 * r.rate:.0f}%"
        ax.text(i, 100 * r.rate + 0.5, f"{pct}\n({r.k}/{r.n})", ha="center", va="bottom",
                fontsize=7, linespacing=1.1)
    ax.set_xticks(list(x))
    ax.set_xticklabels(t.encoding)
    ax.tick_params(axis="x", length=0)
    ax.set_xlabel("Constraint encoding (least to most explicit)")
    ax.set_ylabel("Speed cap breached after edit")
    ax.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    ax.set_ylim(0, 20)
    ax.set_yticks([0, 5, 10, 15, 20])
    _style.grid_y(ax)
    _style.save(fig, args.out)
    print(t.to_string(index=False))


if __name__ == "__main__":
    main()
