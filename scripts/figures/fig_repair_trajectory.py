#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: self-repair trajectories under rule-checker feedback (E3), grayscale-safe.

Revision items R1-6 and R2-M4. One panel per regime for the model that keeps
valid output throughout (Qwen2.5-Coder-32B), on a shared scale. In each panel
the dark curve is a representative cell, labelled with its task, and the thin
gray curves are the other cells of that regime; the panel title carries the
count of cells in the regime from scripts/review/e3_regimes.py. Regimes are
separated by panel rather than by line style, so the figure reads in grayscale.

Usage:
    python3 scripts/figures/fig_repair_trajectory.py \
        --e3 results/E3_full/e3_results.csv \
        --cells paper/tables/e3_regimes_cells.csv \
        --out paper/figures/fig_repair_trajectory.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

import _style
from _style import plt

MODEL = "qwen2.5-coder:32b"
# regime -> panel title, in the order of the figure caption
STYLE = {
    "monotone_decrease": "Monotone reduction",
    "nonmonotone_decrease": "Net reduction, one reversal",
    "oscillation": "Oscillation",
    "transient": "Transient improvement",
    "monotone_increase": "Monotone increase",
    "invariant": "Invariant",
}


def pick_examples(cells: pd.DataFrame) -> dict[str, tuple[str, int]]:
    """Choose, per regime, the Qwen cell whose sequence best illustrates it."""
    out = {}
    q = cells[(cells.model == MODEL) & (cells.n_retries == 3)]
    for regime in STYLE:
        g = q[q.regime == regime]
        if g.empty:
            continue
        # prefer the largest excursion, so the shape is visible
        g = g.assign(span=g.v_start.astype(int).sub(g.v_min.astype(int)).abs()
                     + g.v_end.astype(int).sub(g.v_min.astype(int)).abs())
        r = g.sort_values(["span", "task_id"], ascending=[False, True]).iloc[0]
        out[regime] = (r.task_id, int(r.rep))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e3", type=Path, default=Path("results/E3_full/e3_results.csv"))
    ap.add_argument("--cells", type=Path, default=Path("paper/tables/e3_regimes_cells.csv"))
    ap.add_argument("--out", type=Path, default=Path("paper/figures/fig_repair_trajectory.pdf"))
    args = ap.parse_args()

    e3 = pd.read_csv(args.e3)
    cells = pd.read_csv(args.cells)
    counts = cells[cells.model == MODEL].regime.value_counts().to_dict()
    examples = pick_examples(cells)

    def sequence(task: str, rep: int) -> list[int]:
        g = e3[(e3.model == MODEL) & (e3.task_id == task) & (e3.rep == rep)].sort_values("retry")
        return g.total_violations.tolist()

    q = cells[cells.model == MODEL]
    _style.apply()
    fig, axs = plt.subplots(2, 3, figsize=(_style.COL_W, 2.9), sharex=True, sharey=True,
                            constrained_layout=True)
    for ax, (regime, title) in zip(axs.flat, STYLE.items()):
        for c in q[q.regime == regime].itertuples():
            ax.plot(range(4), sequence(c.task_id, c.rep), color="0.72", linewidth=0.6, zorder=1)
        task, rep = examples[regime]
        y = sequence(task, rep)
        ax.plot(range(4), y, color=_style.INK, linewidth=1.3, marker="o", markersize=3,
                markerfacecolor="white", zorder=3)
        ax.annotate(task, (3, y[-1]), xytext=(3, 0), textcoords="offset points", fontsize=6.5,
                    va="center")
        ax.set_title(f"{title}\n{counts.get(regime, 0)} cells", fontsize=7, linespacing=1.1, pad=3)
        ax.set_xticks([0, 1, 2, 3])
        ax.set_xlim(-0.3, 3.9)
        _style.grid_y(ax)
    fig.supxlabel("Retry index (0 = single-shot generation)", fontsize=8)
    fig.supylabel("Total violations", fontsize=8)
    _style.save(fig, args.out)
    print("examples:", examples)
    print("counts:", counts)


if __name__ == "__main__":
    main()
