#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Combined quantization-control table and uniform-quantization Cochran's Q.

Inputs: the confirmatory E1 run (one model at Q4_K_M, two at Q4_0) and the
three post-review control runs (each model at the other level). Produces:

  1. one table with, per model and condition, gate-pass, gate-passing
     violation rate, mean violation count at both quantizations, the number
     of task cells whose binary verdict agrees, and the Wilcoxon p on
     per-cell mean counts;
  2. Cochran's Q across the three models recomputed on three model sets:
     as served (the manuscript's mixed levels), all Q4_K_M, all Q4_0, using
     the same (task x model) OR-over-reps matrix as scripts/mcnemar_analysis.py.

Usage:
    python3 scripts/review/quantization_summary.py --out-dir paper/tables
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon
from statsmodels.stats.contingency_tables import cochrans_q

REPO = Path(__file__).resolve().parents[2]
MODELS = [  # (display, confirmatory id, confirmatory level, control csv, control id, control level)
    ("Qwen2.5-Coder-32B", "qwen2.5-coder:32b", "Q4_K_M",
     REPO / "paper/data/X10_qwen_q4_0/e1_results.csv", "qwen2.5-coder:32b-instruct-q4_0", "Q4_0"),
    ("DeepSeek-Coder-V2-16B", "deepseek-coder-v2:16b", "Q4_0",
     REPO / "paper/data/X10_deepseek_q4_K_M/e1_results.csv", "deepseek-coder-v2:16b-lite-instruct-q4_K_M", "Q4_K_M"),
    ("CodeLlama-34B", "codellama:34b", "Q4_0",
     REPO / "paper/data/X10_codellama_q4_K_M/e1_results.csv", "codellama:34b-instruct-q4_K_M", "Q4_K_M"),
]
COND_LABEL = {"baseline": "Baseline", "safety": "Safety-prompted"}


def load_all() -> dict[str, dict[str, pd.DataFrame]]:
    e1 = pd.read_csv(REPO / "results/E1_full/e1_results.csv")
    out = {}
    for disp, ref_id, ref_lvl, ctrl_csv, ctrl_id, ctrl_lvl in MODELS:
        ref = e1[e1.model == ref_id].copy()
        ref["level"] = ref_lvl
        d = {ref_lvl: ref}
        if ctrl_csv.is_file():
            c = pd.read_csv(ctrl_csv)
            c = c[c.model == ctrl_id].copy()
            c["level"] = ctrl_lvl
            d[ctrl_lvl] = c
        out[disp] = d
    return out


def cell_stats(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["viol"] = (df.total_violations > 0).astype(int)
    g = df.groupby("task_id").agg(viol_any=("viol", "max"), count_mean=("total_violations", "mean"))
    return g


def per_model_rows(data: dict) -> pd.DataFrame:
    rows = []
    for disp, levels in data.items():
        for cond in ("baseline", "safety"):
            rec = {"model": disp, "condition": cond}
            for lvl in ("Q4_K_M", "Q4_0"):
                if lvl not in levels:
                    rec.update({f"{lvl}_n": 0, f"{lvl}_gate": np.nan, f"{lvl}_viol": np.nan, f"{lvl}_mean": np.nan})
                    continue
                d = levels[lvl][levels[lvl].condition == cond]
                g = d[d.status == "success"]
                rec[f"{lvl}_n"] = len(d)
                rec[f"{lvl}_gate"] = len(g)
                rec[f"{lvl}_viol"] = (g.total_violations > 0).mean() if len(g) else np.nan
                rec[f"{lvl}_mean"] = g.total_violations.mean() if len(g) else np.nan
            if len(levels) == 2:
                a = cell_stats(levels["Q4_K_M"][levels["Q4_K_M"].condition == cond])
                b = cell_stats(levels["Q4_0"][levels["Q4_0"].condition == cond])
                common = a.index.intersection(b.index)
                agree = int((a.loc[common, "viol_any"] == b.loc[common, "viol_any"]).sum())
                b01 = int(((a.loc[common, "viol_any"] == 0) & (b.loc[common, "viol_any"] == 1)).sum())
                b10 = int(((a.loc[common, "viol_any"] == 1) & (b.loc[common, "viol_any"] == 0)).sum())
                p_mc = binomtest(b01, b01 + b10, 0.5).pvalue if b01 + b10 else 1.0
                diff = (b.loc[common, "count_mean"] - a.loc[common, "count_mean"]).values
                p_w = wilcoxon(diff).pvalue if np.any(diff != 0) else 1.0
                rec.update({"cells": len(common), "agree": agree, "mcnemar_p": p_mc, "wilcoxon_p": p_w})
            rows.append(rec)
    return pd.DataFrame(rows)


def cochran_sets(data: dict) -> pd.DataFrame:
    sets = {
        "as served (Q4_K_M / Q4_0 / Q4_0)": {"Qwen2.5-Coder-32B": "Q4_K_M", "DeepSeek-Coder-V2-16B": "Q4_0", "CodeLlama-34B": "Q4_0"},
        "all Q4_K_M": {m: "Q4_K_M" for m in data},
        "all Q4_0": {m: "Q4_0" for m in data},
    }
    rows = []
    for name, choice in sets.items():
        for cond in ("baseline", "safety"):
            cols = {}
            ok = True
            for m, lvl in choice.items():
                if lvl not in data[m]:
                    ok = False
                    break
                d = data[m][lvl]
                d = d[d.condition == cond].copy()
                d["viol"] = (d.total_violations > 0).astype(int)
                cols[m] = d.groupby("task_id")["viol"].max()
            if not ok:
                rows.append({"set": name, "condition": cond, "note": "control run missing"})
                continue
            mat = pd.DataFrame(cols).dropna()
            res = cochrans_q(mat.to_numpy(dtype=int), return_object=True)
            rates = {m: float(mat[m].mean()) for m in mat.columns}
            rows.append({"set": name, "condition": cond, "n_tasks": len(mat), "Q": float(res.statistic),
                         "p": float(res.pvalue), **{f"rate_{m}": r for m, r in rates.items()}})
    return pd.DataFrame(rows)


def to_latex(pm: pd.DataFrame, cq: pd.DataFrame) -> str:
    lines = [
        "% Generated by scripts/review/quantization_summary.py -- do not edit by hand",
        "\\begin{table*}[t]",
        "\\caption{Quantization control. Each of the three models was re-run after the review at",
        "the quantization level it was \\emph{not} served at in the confirmatory run (Q4\\_K\\_M for",
        "the two models served at Q4\\_0 and Q4\\_0 for the model served at Q4\\_K\\_M), on the same",
        "fifteen tasks, two conditions, three repetitions, prompts, decoding settings and frozen",
        "task set, on a local RTX~5090. Gate is the number of the 45 outputs per condition that",
        "pass the validity gate; violation rate and mean count are on the gate-passing basis.",
        "Agree is the number of the 15 task cells whose binary verdict (any repetition violating)",
        "is the same at both levels; McNemar is the exact test on discordant cells and Wilcoxon",
        "the signed-rank test on per-cell mean counts. The lower block recomputes the cross-model",
        "Cochran $Q$ of \\ref{sec:res-sens} with all three models at one level, with the per-task",
        "violation rate of each model in parentheses; $p$ values there are uncorrected. Served at",
        "gives the level of the confirmatory run.}",
        "\\label{tab:quant}",
        "\\centering",
        "\\footnotesize",
        "\\setlength{\\tabcolsep}{3.5pt}",
        "\\begin{tabular}{@{}llrrrrrrrrrr@{}}",
        "\\toprule",
        " & & \\multicolumn{3}{c}{Q4\\_K\\_M} & \\multicolumn{3}{c}{Q4\\_0} & & & & \\\\",
        "\\cmidrule(lr){3-5}\\cmidrule(lr){6-8}",
        "Model & Condition & Gate & Viol.\\ \\% & Mean & Gate & Viol.\\ \\% & Mean & Agree & McNemar $p$ & Wilcoxon $p$ & Served at \\\\",
        "\\midrule",
    ]
    served = {m[0]: m[2] for m in MODELS}
    for _, r in pm.iterrows():
        def f(v, fmt):
            return "--" if pd.isna(v) else fmt.format(v)
        lines.append(
            f"{r.model if r.condition == 'baseline' else ''} & {COND_LABEL[r.condition]} & "
            f"{f(r['Q4_K_M_gate'], '{:.0f}')} & {f(100*r['Q4_K_M_viol'], '{:.0f}')} & {f(r['Q4_K_M_mean'], '{:.2f}')} & "
            f"{f(r['Q4_0_gate'], '{:.0f}')} & {f(100*r['Q4_0_viol'], '{:.0f}')} & {f(r['Q4_0_mean'], '{:.2f}')} & "
            f"{f(r.get('agree', np.nan), '{:.0f}')}/{f(r.get('cells', np.nan), '{:.0f}')} & "
            f"{f(r.get('mcnemar_p', np.nan), '{:.2f}')} & {f(r.get('wilcoxon_p', np.nan), '{:.3f}')} & "
            f"{served[r.model].replace('_', chr(92)+'_') if r.condition == 'baseline' else ''} \\\\")
        if r.condition == "safety":
            lines.append("\\addlinespace")
    lines += ["\\midrule",
              "\\multicolumn{12}{@{}l}{\\textit{Cross-model Cochran $Q$ (15 task cells, cell = any repetition violating)}} \\\\"]
    for _, r in cq.iterrows():
        if "Q" not in r or pd.isna(r.get("Q", np.nan)):
            lines.append(f"\\multicolumn{{12}}{{@{{}}l}}{{{r['set']}, {COND_LABEL[r.condition]}: {r.get('note', '')}}} \\\\")
            continue
        short = {"Qwen2.5-Coder-32B": "Qwen", "DeepSeek-Coder-V2-16B": "DeepSeek", "CodeLlama-34B": "CodeLlama"}
        rates = "; ".join(f"{short.get(k.replace('rate_', ''), k)} {100*v:.0f}\\%" for k, v in r.items() if str(k).startswith("rate_"))
        setname = str(r["set"]).replace("_", "\\_")
        ptxt = "p<0.001" if r.p < 0.001 else f"p={r.p:.3f}"
        lines.append(f"\\multicolumn{{12}}{{@{{}}l}}{{{setname}, {COND_LABEL[r.condition]}: $Q={r.Q:.2f}$, ${ptxt}$ ({rates})}} \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table*}", ""]
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out-dir", type=Path, default=REPO / "paper" / "tables")
    args = ap.parse_args()
    data = load_all()
    pm = per_model_rows(data)
    cq = cochran_sets(data)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    pm.to_csv(args.out_dir / "quantization_summary.csv", index=False)
    cq.to_csv(args.out_dir / "quantization_cochran.csv", index=False)
    (args.out_dir / "quantization_summary.tex").write_text(to_latex(pm, cq))
    pd.set_option("display.width", 250)
    print(pm.round(3).to_string(index=False))
    print(cq.round(4).to_string(index=False))


if __name__ == "__main__":
    main()
