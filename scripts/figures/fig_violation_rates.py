#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: baseline combined violation rate per model, naive vs gate-passing (E1).

For each model under the baseline condition, the share of generations that
carry at least one violation, computed over all generations (naive: invalid
pseudo-code counts as zero-violation) and over only the generations that pass
the validity gate, with Wilson 95% intervals and the counts above each bar.
Grayscale-safe.

Usage:
    python3 scripts/figures/fig_violation_rates.py \
        --e1 results/E1_full/e1_results.csv \
        --out paper/figures/fig_violation_rates.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

import _style
from _style import plt

MODELS = [
    ("qwen2.5-coder:32b", "Qwen2.5-Coder-32B"),
    ("deepseek-coder-v2:16b", "DeepSeek-Coder-V2-16B"),
    ("codellama:34b", "CodeLlama-34B"),
]


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e1", type=Path, default=Path("results/E1_full/e1_results.csv"))
    ap.add_argument("--out", type=Path, default=Path("paper/figures/fig_violation_rates.pdf"))
    args = ap.parse_args()

    e1 = pd.read_csv(args.e1)
    b = e1[e1.condition == "baseline"]
    rows = []
    for key, label in MODELS:
        g = b[b.model == key]
        valid = g.status == "success"
        k = int((valid & (g.total_violations > 0)).sum())
        for basis, n in (("naive", len(g)), ("gate", int(valid.sum()))):
            lo, hi = wilson(k, n)
            rows.append({"model": label, "basis": basis, "k": k, "n": n, "rate": k / n, "lo": lo, "hi": hi})
    t = pd.DataFrame(rows)

    _style.apply()
    fig, ax = plt.subplots(figsize=(_style.COL_W, 2.3), constrained_layout=True)
    w = 0.36
    series = (
        ("naive", "Naive (all generations)", "white", "////"),
        ("gate", "Gate-passing only", _style.FILL_DARK, None),
    )
    for s, (basis, name, fill, hatch) in enumerate(series):
        d = t[t.basis == basis].reset_index(drop=True)
        x = np.arange(len(d)) + (s - 0.5) * w
        y = 100 * d.rate
        ax.bar(x, y, w * 0.94, color=fill, edgecolor=_style.INK, linewidth=0.6, hatch=hatch, label=name)
        yerr = np.clip([y - 100 * d.lo, 100 * d.hi - y], 0, None)
        ax.errorbar(x, y, yerr=yerr, fmt="none",
                    ecolor=_style.INK, elinewidth=0.7, capsize=2)
        for xi, r in zip(x, d.itertuples()):
            ax.text(xi, 100 * r.hi + 2, f"{r.k}/{r.n}", ha="center", va="bottom", fontsize=6.5)
    ax.set_xticks(np.arange(len(MODELS)))
    ax.set_xticklabels([m.replace("-Coder", "-\nCoder", 1) if "DeepSeek" in m or "Qwen" in m
                        else m.replace("-34B", "-\n34B") for _, m in MODELS])
    ax.set_ylabel("Combined violation rate (%)")
    ax.set_ylim(0, 115)
    ax.set_yticks(range(0, 101, 20))
    ax.tick_params(axis="x", length=0)
    _style.grid_y(ax)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False,
              handlelength=1.6, columnspacing=1.5, borderaxespad=0.2)
    _style.save(fig, args.out)
    print(t.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
