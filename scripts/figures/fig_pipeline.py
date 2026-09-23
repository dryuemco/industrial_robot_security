#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Figure: the FACT pipeline (full text width).

Task representation -> prompt builder -> open-weight code model -> validity
gate -> lexical rule checker -> violation verdict, with the repair-in-loop
feedback path. Outputs that fail the gate are routed directly to the
"invalid" verdict. Drawn at the printed width so the text is at final size.

Usage:
    python3 scripts/figures/fig_pipeline.py --out paper/figures/pipeline.pdf
"""
from __future__ import annotations

import argparse
from pathlib import Path

from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

import _style
from _style import plt

W, H = _style.TEXT_W, 1.6
BOX_H, BOX_Y = 0.72, 0.42
WIDTHS = [1.12, 0.98, 1.02, 1.43, 1.13]
GAPS = [0.2, 0.2, 0.8, 0.2]  # the third gap holds the validity gate
BOXES = [
    ("Task representation", "vendor-neutral JSON\nwaypoints, modes,\nsafety constraints", "white"),
    ("Prompt builder", "baseline / safety /\nadversarial", "white"),
    ("Code model", "open-weight\nlocal 16–34B\nfrontier 24–675B", "white"),
    ("Lexical rule checker", "7 motion-safety checks (IR)\n7 security checks (code,\nCWE-mapped)", "0.9"),
    ("Violation verdict", "invalid / valid-unsafe /\nsubstantive-safe\nISO 10218-1:2025", "0.9"),
]


def arrow(ax, p, q, **kw) -> None:
    style = dict(arrowstyle="-|>", mutation_scale=8, linewidth=0.9, color=_style.INK, shrinkA=0, shrinkB=0)
    style.update(kw)
    ax.add_patch(FancyArrowPatch(p, q, **style))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=Path("paper/figures/pipeline.pdf"))
    args = ap.parse_args()

    _style.apply()
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    xs = [0.04]
    for w, g in zip(WIDTHS, GAPS):
        xs.append(xs[-1] + w + g)
    for x, bw, (title, body, fill) in zip(xs, WIDTHS, BOXES):
        ax.add_patch(FancyBboxPatch((x, BOX_Y), bw, BOX_H, boxstyle="round,pad=0,rounding_size=0.05",
                                    facecolor=fill, edgecolor=_style.INK, linewidth=0.8))
        ax.text(x + bw / 2, BOX_Y + BOX_H - 0.1, title, ha="center", va="top", fontsize=8.5,
                fontweight="bold")
        ax.text(x + bw / 2, BOX_Y + BOX_H - 0.3, body, ha="center", va="top", fontsize=7.5,
                linespacing=1.15)

    ymid = BOX_Y + BOX_H / 2
    for i in range(len(BOXES) - 1):
        if i == 2:
            continue
        arrow(ax, (xs[i] + WIDTHS[i], ymid), (xs[i + 1], ymid))

    # code model -> validity gate -> checker, with the gate drawn as a diamond on the arrow
    gx = (xs[2] + WIDTHS[2] + xs[3]) / 2
    d = 0.075
    ax.plot([xs[2] + WIDTHS[2], gx - d], [ymid, ymid], color=_style.INK, linewidth=0.9)
    ax.add_patch(Polygon([(gx - d, ymid), (gx, ymid + d), (gx + d, ymid), (gx, ymid - d)],
                         closed=True, facecolor="white", edgecolor=_style.INK, linewidth=0.8, zorder=3))
    arrow(ax, (gx + d, ymid), (xs[3], ymid))
    ax.text(gx, ymid + d + 0.03, "URScript\nvalidity gate", ha="center", va="bottom", fontsize=7,
            style="italic", linespacing=1.1)

    # gate failure -> "invalid" verdict, routed below the checker
    ylow = 0.16
    ax.plot([gx, gx, xs[4] + WIDTHS[4] / 2], [ymid - d, ylow, ylow], color=_style.MID, linewidth=0.8)
    arrow(ax, (xs[4] + WIDTHS[4] / 2, ylow), (xs[4] + WIDTHS[4] / 2, BOX_Y), color=_style.MID, linewidth=0.8)
    ax.text(xs[3] + WIDTHS[3] / 2, ylow - 0.03, "fails gate: invalid pseudo-code",
            ha="center", va="top", fontsize=7, style="italic", color=_style.INK)

    # repair-in-loop feedback: checker -> prompt builder, over the top
    ytop = BOX_Y + BOX_H + 0.2
    src, dst = xs[3] + WIDTHS[3] / 2, xs[1] + WIDTHS[1] / 2
    ax.plot([src, src, dst], [BOX_Y + BOX_H, ytop, ytop], color=_style.MID, linewidth=0.8,
            linestyle=(0, (4, 2)))
    arrow(ax, (dst, ytop), (dst, BOX_Y + BOX_H), color=_style.MID, linewidth=0.8, linestyle=(0, (4, 2)))
    ax.text((src + dst) / 2, ytop + 0.03, "repair-in-loop: detected violations fed back as a repair prompt",
            ha="center", va="bottom", fontsize=7, style="italic")

    _style.save(fig, args.out)


if __name__ == "__main__":
    main()
