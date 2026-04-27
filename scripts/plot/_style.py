# SPDX-License-Identifier: Apache-2.0
"""Shared matplotlib style for ENFIELD paper figures.

IEEE single-column figure default: 3.5" wide. Double column: 7.16".
Fonts default to a serif family at 8 pt for IEEE house style.

Usage:
    from _style import apply_ieee_style, IEEE_SINGLE_COL, COLOR_MODELS
    apply_ieee_style()
    fig, ax = plt.subplots(figsize=IEEE_SINGLE_COL)
"""
from __future__ import annotations

import matplotlib as mpl
import matplotlib.pyplot as plt


# Figure widths (inches) per IEEE column convention
IEEE_SINGLE_COL = (3.5, 2.4)
IEEE_DOUBLE_COL = (7.16, 3.0)

# Stable colour assignment for the three confirmatory models.
# Keys match the per_condition_labels values in cochran_results.csv.
COLOR_MODELS = {
    "codellama:34b":          "#4477AA",   # blue
    "deepseek-coder-v2:16b":  "#EE6677",   # red
    "qwen2.5-coder:32b":      "#228833",   # green
}

# Pretty model labels for axis text.
PRETTY_MODEL = {
    "codellama:34b":          "CodeLlama-34B",
    "deepseek-coder-v2:16b":  "DeepSeek-Coder-V2-16B",
    "qwen2.5-coder:32b":      "Qwen2.5-Coder-32B",
}

# Pretty condition labels for x-axis ordering.
CONDITION_ORDER = ["baseline", "safety", "adversarial_any", "watchdog"]
PRETTY_CONDITION = {
    "baseline":         "Baseline",
    "safety":           "Safety prompt",
    "adversarial_any":  "Adversarial",
    "watchdog":         "Watchdog-in-loop",
}


def apply_ieee_style() -> None:
    """Apply IEEE-style rcParams. Call once at the top of each fig_*.py."""
    mpl.rcParams.update({
        "font.family":      "serif",
        "font.serif":       ["DejaVu Serif", "Times New Roman", "Times"],
        "font.size":        8,
        "axes.labelsize":   8,
        "axes.titlesize":   9,
        "legend.fontsize":  7,
        "xtick.labelsize":  7,
        "ytick.labelsize":  7,
        "axes.linewidth":   0.6,
        "axes.spines.top":   False,
        "axes.spines.right": False,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "lines.linewidth":   1.0,
        "savefig.dpi":       300,
        "savefig.bbox":      "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype":     42,   # editable text in PDF
        "ps.fonttype":      42,
    })
