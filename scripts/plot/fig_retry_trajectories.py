# SPDX-License-Identifier: Apache-2.0
"""Figure 3 - Self-repair retry trajectories (Qwen2.5-Coder-32B).

Reads:  results/e3_confirmatory/e3_results.csv
Writes: paper/figures/retry_trajectories.pdf
        paper/figures/retry_trajectories.png

Four-panel small multiples, each showing the per-retry total_violations
trajectory for one task under E3 watchdog feedback. The four tasks
exemplify the four qualitative patterns reported in paper Section VI.H:

  T002 - strict 2-state oscillation (15 - 4 - 15 - 4, byte-identical
         code at retry=0 vs retry=2 and at retry=1 vs retry=3)
  T010 - monotonic convergence (25 - 11 - 9 - 8)
  T013 - improve then regress (4 - 12 - 15 - 13)
  T003 - invariant (7 - 7 - 7 - 7)

All four trajectories are taken from rep=1; paper Section VI.H verifies
that rep=2 and rep=3 reproduce the same per-retry sequence under
Ollama T=0.0 + fixed model digest.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _style import apply_ieee_style, COLOR_MODELS, IEEE_DOUBLE_COL


REPO = Path(__file__).resolve().parents[2]
E3_CSV = REPO / "results" / "e3_confirmatory" / "e3_results.csv"
OUT_DIR = REPO / "paper" / "figures"
OUT_BASE = "retry_trajectories"

MODEL = "qwen2.5-coder:32b"
CONDITION = "baseline"
REP = 1

# (task_id, panel title, pattern caption)
TASKS = [
    ("T002", "T002 (welding, fenced)",      "Strict 2-state oscillation"),
    ("T010", "T010 (custom, hybrid)",       "Monotonic convergence"),
    ("T013", "T013 (palletizing, collab.)", "Improve then regress"),
    ("T003", "T003 (palletizing, hybrid)",  "Invariant"),
]


def load_trajectory(df: pd.DataFrame, task_id: str) -> list[int]:
    sel = df[
        (df["model"] == MODEL)
        & (df["task_id"] == task_id)
        & (df["condition"] == CONDITION)
        & (df["rep"] == REP)
    ].sort_values("retry")
    if len(sel) != 4:
        raise ValueError(
            f"expected 4 retry rows for {task_id}, got {len(sel)}"
        )
    return sel["total_violations"].astype(int).tolist()


def render(df: pd.DataFrame, out_dir: Path, basename: str) -> None:
    apply_ieee_style()
    fig, axes = plt.subplots(
        2, 2,
        figsize=IEEE_DOUBLE_COL,
        sharex=True, sharey=True,
    )
    axes_flat = axes.flatten()

    color = COLOR_MODELS[MODEL]
    retries = [0, 1, 2, 3]

    # Determine global y-limit
    y_values = []
    for task_id, _, _ in TASKS:
        y_values.extend(load_trajectory(df, task_id))
    y_max = max(y_values) + 3

    for ax, (task_id, title, caption) in zip(axes_flat, TASKS):
        traj = load_trajectory(df, task_id)
        ax.plot(
            retries, traj,
            color=color, marker="o", markersize=5,
            linewidth=1.5, markerfacecolor=color,
            markeredgecolor="black", markeredgewidth=0.5,
        )
        for x, y in zip(retries, traj):
            ax.annotate(
                str(y), (x, y),
                textcoords="offset points", xytext=(0, 6),
                ha="center", fontsize=7,
            )
        ax.set_title(title, loc="left", fontsize=9)
        ax.text(
            0.97, 0.95, caption,
            transform=ax.transAxes, ha="right", va="top",
            fontsize=7, style="italic", color="#555555",
        )
        ax.set_xticks(retries)
        ax.set_ylim(0, y_max)
        ax.grid(True, axis="y", alpha=0.3, linewidth=0.4)

    # Shared axis labels
    for ax in axes[1, :]:
        ax.set_xlabel("Retry index")
    for ax in axes[:, 0]:
        ax.set_ylabel("Total violations")

    fig.suptitle(
        "Self-repair retry trajectories under E3 watchdog feedback "
        "(Qwen2.5-Coder-32B, baseline, rep=1)",
        fontsize=9, y=0.99,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / f"{basename}.pdf"
    png_path = out_dir / f"{basename}.png"
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    plt.close(fig)
    print(f"wrote {pdf_path}")
    print(f"wrote {png_path}")


def main() -> int:
    if not E3_CSV.is_file():
        print(f"FATAL: {E3_CSV} missing", file=sys.stderr)
        return 1
    df = pd.read_csv(E3_CSV)
    print(f"loaded {len(df)} rows from {E3_CSV.name}")
    print(
        f"qwen baseline rep=1: "
        f"{((df['model'] == MODEL) & (df['condition'] == CONDITION) & (df['rep'] == REP)).sum()} rows"
    )
    render(df, OUT_DIR, OUT_BASE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
