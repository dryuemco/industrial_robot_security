# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Shared style for the paper figures.

Every figure is drawn at its final printed width (one IEEE Access column,
or the full text width for figure* floats), so the font sizes set here are
the sizes that appear on the page. The serif face is Liberation Serif, which
is metric-compatible with the Times used for the body text; fonts are
embedded as TrueType (Type 42) rather than Type 3. All figures are grayscale
so that they read identically in print.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COL_W = 3.5    # in, one column
TEXT_W = 7.16  # in, full text width (figure*)

INK = "0.1"     # marks and text
MID = "0.45"    # secondary marks
GRID = "0.85"   # grid lines
FILL_DARK = "0.3"
FILL_LIGHT = "0.75"


def apply() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Liberation Serif", "Nimbus Roman", "Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "axes.linewidth": 0.6,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.major.size": 2.5,
        "ytick.major.size": 2.5,
        "axes.edgecolor": INK,
        "text.color": INK,
        "axes.labelcolor": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "hatch.linewidth": 0.5,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.dpi": 300,
    })


def grid_y(ax) -> None:
    ax.grid(True, axis="y", linewidth=0.4, color=GRID)
    ax.set_axisbelow(True)


def save(fig, out: Path) -> None:
    """Write the PDF used by the paper and a 300 dpi PNG preview beside it."""
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    fig.savefig(out.with_suffix(".png"))
    print(f"Wrote {out}")
