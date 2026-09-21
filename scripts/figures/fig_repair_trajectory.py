#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: self-repair trajectories under rule-checker feedback (E3), grayscale-safe.

Revision items R1-6 and R2-M4. One representative cell per regime for the
model that keeps valid output throughout (Qwen2.5-Coder-32B), each drawn
with its own marker AND line style AND a direct end label, so the curves
remain distinguishable when printed in grayscale. The legend carries the
count of cells in each regime from scripts/review/e3_regimes.py.

Usage:
    python3 scripts/figures/fig_repair_trajectory.py \
        --e3 results/E3_full/e3_results.csv \
        --cells paper/tables/e3_regimes_cells.csv \
        --out paper/figures/fig_repair_trajectory.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

MODEL = "qwen2.5-coder:32b"
# regime -> (label, marker, linestyle, gray level)
STYLE = {
    "monotone_decrease": ("Monotone reduction, not to zero", "o", "-", "0.0"),
    "nonmonotone_decrease": ("Net reduction, one reversal", "D", (0, (5, 2)), "0.15"),
    "oscillation": ("Oscillation", "s", (0, (2, 1.5)), "0.3"),
    "transient": ("Transient improvement", "^", (0, (6, 2, 1, 2)), "0.45"),
    "monotone_increase": ("Monotone increase", "v", (0, (1, 1)), "0.55"),
    "invariant": ("Invariant", "x", (0, (8, 3)), "0.65"),
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

    plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.linewidth": 0.6})
    fig, ax = plt.subplots(figsize=(3.5, 3.1), dpi=300)
    ends = []  # (task, x_end, y_end, gray) for the direct end labels
    for regime, (label, marker, ls, gray) in STYLE.items():
        if regime not in examples:
            continue
        task, rep = examples[regime]
        seq = e3[(e3.model == MODEL) & (e3.task_id == task) & (e3.rep == rep)].sort_values("retry")
        x, y = seq.retry.tolist(), seq.total_violations.tolist()
        ax.plot(x, y, marker=marker, linestyle=ls, color=gray, markersize=4, linewidth=1.1,
                markerfacecolor="white" if marker in "oDs" else gray, markeredgewidth=0.9,
                label=f"{label} ({counts.get(regime, 0)} cells), e.g. {task}")
        ends.append((task, x[-1], y[-1], gray))
    ax.set_xlim(-0.2, 3.6)
    ax.set_ylim(bottom=0)
    # Direct end labels: push labels apart vertically (in points) so that curves ending
    # at neighbouring values do not overprint, and draw a thin leader where a label moved.
    min_gap_pt = 7.0
    ends.sort(key=lambda e: e[2])
    to_pt = 72.0 / fig.dpi
    ys_pt = [ax.transData.transform((xe, ye))[1] * to_pt for _, xe, ye, _ in ends]
    placed = []
    for y_pt in ys_pt:
        placed.append(y_pt if not placed else max(y_pt, placed[-1] + min_gap_pt))
    # re-centre the stack so that it is not pushed only upwards
    shift = (sum(placed) - sum(ys_pt)) / len(placed) if placed else 0.0
    placed = [p - shift for p in placed]
    for (task, xe, ye, gray), y_pt, y_lab in zip(ends, ys_pt, placed):
        dy = y_lab - y_pt
        ax.annotate(task, (xe, ye), xytext=(7, dy), textcoords="offset points", fontsize=6,
                    va="center", ha="left", color=gray,
                    arrowprops=dict(arrowstyle="-", color=gray, linewidth=0.4,
                                    shrinkA=0, shrinkB=2.5) if abs(dy) > 0.5 else None)
    ax.set_xlabel("Retry index (0 = single-shot generation)")
    ax.set_ylabel("Total violations")
    ax.set_xticks([0, 1, 2, 3])
    ax.grid(True, linewidth=0.3, color="0.85")
    ax.legend(fontsize=5.8, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.22),
              ncol=1, handlelength=3.2)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out)
    fig.savefig(args.out.with_suffix(".png"))
    print("examples:", examples)
    print("counts:", counts)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
