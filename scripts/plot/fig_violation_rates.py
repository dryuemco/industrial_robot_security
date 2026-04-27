# SPDX-License-Identifier: Apache-2.0
"""Figure 1 — Per-model combined violation rate across four conditions.

Reads:  results/stats_e3_full/cochran_results.csv
Writes: paper/figures/per_model_violation_rate.pdf
        paper/figures/per_model_violation_rate.png

The CSV holds one row per condition with semicolon-separated
`per_condition_labels` (model identifiers) and `per_condition_rates`
(combined violation rate, [0, 1]). We pivot to a 4-condition x 3-model
grouped bar chart, sorted by CONDITION_ORDER and consistent model
colour mapping from _style.py.

Re-run safely; output overwrites in place.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _style import (
    apply_ieee_style,
    IEEE_SINGLE_COL,
    COLOR_MODELS,
    PRETTY_MODEL,
    CONDITION_ORDER,
    PRETTY_CONDITION,
)


REPO = Path(__file__).resolve().parents[2]
COCHRAN_CSV = REPO / "results" / "stats_e3_full" / "cochran_results.csv"
OUT_DIR = REPO / "paper" / "figures"
OUT_BASE = "per_model_violation_rate"


def load_data() -> pd.DataFrame:
    """Load cochran_results.csv and explode label/rate pairs into long form."""
    df = pd.read_csv(COCHRAN_CSV)
    rows = []
    for _, r in df.iterrows():
        labels = r["per_condition_labels"].split(";")
        rates = [float(x) for x in r["per_condition_rates"].split(";")]
        for lbl, rate in zip(labels, rates):
            rows.append({
                "condition": r["condition"],
                "model":     lbl,
                "rate":      rate,
            })
    return pd.DataFrame(rows)


def render(long_df: pd.DataFrame, out_dir: Path, basename: str) -> None:
    apply_ieee_style()
    fig, ax = plt.subplots(figsize=IEEE_SINGLE_COL)

    n_cond = len(CONDITION_ORDER)
    models = list(COLOR_MODELS.keys())
    n_models = len(models)
    bar_w = 0.8 / n_models
    x_idx = np.arange(n_cond)

    for i, model in enumerate(models):
        rates = []
        for cond in CONDITION_ORDER:
            sel = long_df[
                (long_df["condition"] == cond) & (long_df["model"] == model)
            ]
            rates.append(sel["rate"].iloc[0] if len(sel) else np.nan)
        offsets = x_idx + (i - (n_models - 1) / 2.0) * bar_w
        ax.bar(
            offsets, rates,
            width=bar_w * 0.95,
            color=COLOR_MODELS[model],
            label=PRETTY_MODEL[model],
            edgecolor="black",
            linewidth=0.4,
        )

    ax.set_xticks(x_idx)
    ax.set_xticklabels(
        [PRETTY_CONDITION[c] for c in CONDITION_ORDER],
        rotation=15, ha="right",
    )
    ax.set_ylabel("Combined violation rate")
    ax.set_ylim(0, 1.05)
    ax.set_yticks(np.linspace(0, 1.0, 6))
    ax.axhline(0.30, color="grey", linewidth=0.5,
               linestyle="--", alpha=0.7)
    # Place the threshold label outside the plotting area, right side,
    # so no model bar can ever cross it. White bbox guards readability
    # if the layout is later cropped.
    ax.text(
        1.005, 0.30, "H4 threshold (0.30)",
        transform=ax.get_yaxis_transform(),
        fontsize=6, color="grey", ha="left", va="center",
        bbox=dict(facecolor="white", edgecolor="none", pad=0.5),
    )
    ax.legend(
        loc="upper left", bbox_to_anchor=(1.0, 1.0),
        frameon=False, handlelength=1.5,
    )
    ax.set_title(
        "Per-model combined violation rate (n=15 tasks per cell)",
        loc="left",
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / f"{basename}.pdf"
    png_path = out_dir / f"{basename}.png"
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    plt.close(fig)
    print(f"wrote {pdf_path}")
    print(f"wrote {png_path}")


def main() -> int:
    if not COCHRAN_CSV.is_file():
        print(f"FATAL: {COCHRAN_CSV} missing", file=sys.stderr)
        return 1
    long_df = load_data()
    print(
        f"loaded {len(long_df)} rows "
        f"({long_df['condition'].nunique()} conditions x "
        f"{long_df['model'].nunique()} models)"
    )
    render(long_df, OUT_DIR, OUT_BASE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
