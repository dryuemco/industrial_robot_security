#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Quantization sensitivity: the same model at Q4_K_M vs Q4_0 (extra analysis X10).

Reviewer 2 noted that Qwen2.5-Coder-32B was served at Q4_K_M while the other
two models were at Q4_0, so the cross-model heterogeneity test cannot
separate model identity from quantization. This script compares the
confirmatory Qwen run (Q4_K_M, from E1_full) with a re-run of the same
model at Q4_0 on the same 15 tasks, 2 conditions, 3 repetitions, same
prompts and decoding settings, using the experiment-time task IR.

Reported per condition: gate-pass, binary violation rate (all and gate
basis), mean violation count, per-rule firing shares, matched-cell
agreement, McNemar exact test on the binary verdict over the 15 task cells
(cell = majority over reps), and a Wilcoxon signed-rank test on the
per-cell mean violation count.

Usage:
    python3 scripts/review/quantization_compare.py \
        --ref results/E1_full/e1_results.csv --ref-model qwen2.5-coder:32b \
        --new results/X10_qwen_q4_0/e1_results.csv --new-model qwen2.5-coder:32b-instruct-q4_0 \
        --out-dir paper/tables
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon

RULES = ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]


def cell_table(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["viol"] = (df.total_violations > 0).astype(int)
    df["gate"] = (df.status == "success").astype(int)
    g = df.groupby(["condition", "task_id"]).agg(
        viol_mean=("viol", "mean"), count_mean=("total_violations", "mean"), gate_mean=("gate", "mean"), n=("rep", "size"))
    g["viol_major"] = (g.viol_mean >= 0.5).astype(int)
    return g


def rule_shares(df: pd.DataFrame) -> dict[str, float]:
    g = df[df.status == "success"]
    if g.empty:
        return {r: float("nan") for r in RULES}
    sets = g.violation_types.fillna("").astype(str).str.split(",")
    return {r: float(np.mean([r in s for s in sets])) for r in RULES}


def mcnemar_exact(a: pd.Series, b: pd.Series) -> tuple[int, int, float]:
    b01 = int(((a == 0) & (b == 1)).sum())
    b10 = int(((a == 1) & (b == 0)).sum())
    n = b01 + b10
    p = binomtest(b01, n, 0.5).pvalue if n else 1.0
    return b01, b10, p


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ref", type=Path, default=Path("results/E1_full/e1_results.csv"))
    ap.add_argument("--ref-model", default="qwen2.5-coder:32b")
    ap.add_argument("--ref-label", default="Q4\\_K\\_M (confirmatory)")
    ap.add_argument("--new", type=Path, required=True)
    ap.add_argument("--new-model", default=None)
    ap.add_argument("--new-label", default="Q4\\_0 (re-run)")
    ap.add_argument("--out-dir", type=Path, default=Path("paper/tables"))
    args = ap.parse_args()

    ref = pd.read_csv(args.ref)
    ref = ref[ref.model == args.ref_model]
    new = pd.read_csv(args.new)
    if args.new_model:
        new = new[new.model == args.new_model]
    assert len(ref) and len(new), "empty selection"

    rows = []
    for cond in ["baseline", "safety"]:
        r, n = ref[ref.condition == cond], new[new.condition == cond]
        cr, cn = cell_table(r).loc[cond], cell_table(n).loc[cond]
        common = cr.index.intersection(cn.index)
        b01, b10, p_mc = mcnemar_exact(cr.loc[common, "viol_major"], cn.loc[common, "viol_major"])
        d = (cn.loc[common, "count_mean"] - cr.loc[common, "count_mean"]).values
        p_w = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
        rec = {
            "condition": cond,
            "ref_n": len(r), "new_n": len(n),
            "ref_gate": int((r.status == "success").sum()), "new_gate": int((n.status == "success").sum()),
            "ref_viol_all": (r.total_violations > 0).mean(), "new_viol_all": (n.total_violations > 0).mean(),
            "ref_viol_gate": (r[r.status == "success"].total_violations > 0).mean() if (r.status == "success").any() else np.nan,
            "new_viol_gate": (n[n.status == "success"].total_violations > 0).mean() if (n.status == "success").any() else np.nan,
            "ref_count_gate": r[r.status == "success"].total_violations.mean(),
            "new_count_gate": n[n.status == "success"].total_violations.mean(),
            "cells": len(common), "cells_agree_binary": int((cr.loc[common, "viol_major"] == cn.loc[common, "viol_major"]).sum()),
            "mcnemar_b01": b01, "mcnemar_b10": b10, "mcnemar_p": p_mc,
            "wilcoxon_p": p_w, "mean_count_diff": float(np.mean(d)),
        }
        for k, v in rule_shares(r).items():
            rec[f"ref_{k}"] = v
        for k, v in rule_shares(n).items():
            rec[f"new_{k}"] = v
        rows.append(rec)
    t = pd.DataFrame(rows)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    t.to_csv(args.out_dir / "quantization_compare.csv", index=False)

    lines = [
        "% Generated by scripts/review/quantization_compare.py -- do not edit by hand",
        "\\begin{table}[t]",
        "\\caption{Quantization sensitivity for Qwen2.5-Coder-32B: the confirmatory run",
        f"({args.ref_label}) against a re-run of the same weights at {args.new_label} on the",
        "same tasks, prompts and decoding settings (15 tasks, 3 repetitions per condition).",
        "Cells agreeing gives the number of task cells with the same majority binary verdict;",
        "McNemar is the exact test on discordant cells; Wilcoxon is the signed-rank test on",
        "per-cell mean violation counts. Rule columns give the share of gate-passing outputs on",
        "which each check fires.}",
        "\\label{tab:quant}",
        "\\centering",
        "\\footnotesize",
        "\\setlength{\\tabcolsep}{3pt}",
        "\\begin{tabular}{@{}llrrrrrrrrrr@{}}",
        "\\toprule",
        "Cond. & Quant. & $n$ & Gate & Viol. (gate) & Mean & SM-1 & SM-2 & SM-4 & SM-5 & SM-6 & Tests \\\\",
        "\\midrule",
    ]
    for _, r in t.iterrows():
        cond = "Baseline" if r.condition == "baseline" else "Safety"
        for tag, lab in (("ref", args.ref_label), ("new", args.new_label)):
            tests = ""
            if tag == "new":
                tests = (f"agree {int(r.cells_agree_binary)}/{int(r.cells)}; McN $p$={r.mcnemar_p:.2f}; "
                         f"Wilc. $p$={r.wilcoxon_p:.2f}")
            lines.append(
                f"{cond if tag == 'ref' else ''} & {lab} & {int(r[f'{tag}_n'])} & {int(r[f'{tag}_gate'])} & "
                f"{100*r[f'{tag}_viol_gate']:.1f} & {r[f'{tag}_count_gate']:.2f} & "
                + " & ".join(f"{100*r[f'{tag}_{k}']:.0f}" for k in ["SM-1", "SM-2", "SM-4", "SM-5", "SM-6"])
                + f" & {tests} \\\\")
        lines.append("\\addlinespace")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", ""]
    (args.out_dir / "quantization_compare.tex").write_text("\n".join(lines))
    pd.set_option("display.width", 250)
    cols = ["condition", "ref_n", "new_n", "ref_gate", "new_gate", "ref_viol_gate", "new_viol_gate",
            "ref_count_gate", "new_count_gate", "cells_agree_binary", "cells", "mcnemar_p", "wilcoxon_p", "mean_count_diff"]
    print(t[cols].round(3).to_string(index=False))


if __name__ == "__main__":
    main()
