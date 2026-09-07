#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Quantify repair-in-loop trajectory regimes for the local models (E3).

Revision item R2-M4: the manuscript draws four trajectory regimes
schematically (Fig. 4) but never counts them. This script classifies every
(model, task, rep) cell of E3 by the shape of its total-violation sequence
over retries, and reports counts per regime and per model, plus the
rule-level churn between consecutive retries (rules cleared vs introduced).

Regimes (applied in order):
  dropout_immediate  retry 0 already failed the validity gate; no feedback
                     loop possible (the loop stops on invalid output).
  dropout_later      started valid, a later retry fell out of validity.
  remediated         last valid output has zero violations.
  invariant          every retry has the same violation count.
  monotone_decrease  never increases, ends strictly below the start.
  monotone_increase  never decreases, ends strictly above the start.
  oscillation        at least two direction changes (e.g. 5,12,5,5 or 7,3,7,3).
  nonmonotone_decrease one direction change, ends strictly below the start.
  transient          one direction change, ends at or above the start
                     (improves then regresses, or the reverse).
  other              anything left.

Usage:
    python3 scripts/review/e3_regimes.py --e3 results/E3_full/e3_results.csv \
        --out-dir paper/tables
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

MODEL_LABEL = {
    "qwen2.5-coder:32b": "Qwen2.5-Coder-32B",
    "deepseek-coder-v2:16b": "DeepSeek-Coder-V2-16B",
    "codellama:34b": "CodeLlama-34B",
}
REGIMES = [
    "dropout_immediate", "dropout_later", "remediated", "invariant",
    "monotone_decrease", "nonmonotone_decrease", "monotone_increase", "oscillation",
    "transient", "other",
]
REGIME_LABEL = {
    "dropout_immediate": "Dropout at retry 0 (no loop)",
    "dropout_later": "Dropout after $\\geq$1 retry",
    "remediated": "Remediated to zero",
    "invariant": "Invariant",
    "monotone_decrease": "Monotone reduction, not to zero",
    "nonmonotone_decrease": "Net reduction with one reversal",
    "monotone_increase": "Monotone increase",
    "oscillation": "Oscillation ($\\geq$2 reversals)",
    "transient": "Transient (1 reversal)",
    "other": "Other",
}


def _direction_changes(seq: list[int]) -> int:
    diffs = [b - a for a, b in zip(seq, seq[1:]) if b != a]
    signs = [1 if d > 0 else -1 for d in diffs]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def classify(cell: pd.DataFrame) -> dict:
    cell = cell.sort_values("retry")
    statuses = cell["status"].tolist()
    counts = cell["total_violations"].astype(int).tolist()
    valid = [s == "success" for s in statuses]

    if not valid[0]:
        regime = "dropout_immediate"
    elif not all(valid):
        regime = "dropout_later"
    else:
        if counts[-1] == 0:
            regime = "remediated"
        elif len(set(counts)) == 1:
            regime = "invariant"
        elif all(b <= a for a, b in zip(counts, counts[1:])):
            regime = "monotone_decrease"
        elif all(b >= a for a, b in zip(counts, counts[1:])):
            regime = "monotone_increase"
        else:
            dc = _direction_changes(counts)
            if dc >= 2:
                regime = "oscillation"
            elif dc == 1 and counts[-1] < counts[0]:
                regime = "nonmonotone_decrease"
            elif dc == 1:
                regime = "transient"
            else:
                regime = "other"

    # rule churn between consecutive valid retries
    rules = [set(str(v).split(",")) - {"", "nan"} for v in cell["violation_types"].tolist()]
    cleared = introduced = steps = 0
    for (va, ra), (vb, rb) in zip(zip(valid, rules), zip(valid[1:], rules[1:])):
        if va and vb:
            steps += 1
            cleared += len(ra - rb)
            introduced += len(rb - ra)
    return {
        "regime": regime,
        "n_retries": len(cell) - 1,
        "v_start": counts[0],
        "v_end": counts[-1],
        "v_min": min(counts),
        "valid_steps": steps,
        "rules_cleared": cleared,
        "rules_introduced": introduced,
        "sequence": "-".join(str(c) if v else "x" for c, v in zip(counts, valid)),
    }


def build(e3: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (model, task, rep), cell in e3.groupby(["model", "task_id", "rep"]):
        rec = {"model": model, "task_id": task, "rep": rep}
        rec.update(classify(cell))
        rows.append(rec)
    return pd.DataFrame(rows)


def summary(cells: pd.DataFrame) -> pd.DataFrame:
    tab = cells.pivot_table(index="regime", columns="model", values="task_id",
                            aggfunc="count", fill_value=0)
    tab = tab.reindex(REGIMES, fill_value=0)
    tab = tab[[m for m in MODEL_LABEL if m in tab.columns]]
    tab["pooled"] = tab.sum(axis=1)
    return tab


def to_latex(tab: pd.DataFrame, cells: pd.DataFrame) -> str:
    valid_all = cells[~cells.regime.str.startswith("dropout")]
    lines = [
        "% Generated by scripts/review/e3_regimes.py -- do not edit by hand",
        "\\begin{table}[t]",
        "\\caption{Repair-in-loop trajectory regimes for the 135 local-model cells of E3",
        "(three models, fifteen tasks, three repetitions; up to three retries). A cell",
        "is classified by the shape of its violation-count sequence over retries; dropout",
        "means the output failed the URScript validity gate, which terminates the loop.",
        "No cell reached zero violations.}",
        "\\label{tab:regimes}",
        "\\centering",
        "\\footnotesize",
        "\\begin{tabular}{@{}p{0.46\\columnwidth}rrrr@{}}",
        "\\toprule",
        "Regime & Qwen & DeepSeek & CodeLlama & Pooled \\\\",
        "\\midrule",
    ]
    for reg in REGIMES:
        r = tab.loc[reg]
        vals = [int(r.get(m, 0)) for m in MODEL_LABEL] + [int(r["pooled"])]
        if sum(vals) == 0 and reg == "other":
            continue
        lines.append(f"{REGIME_LABEL[reg]} & " + " & ".join(str(v) for v in vals) + " \\\\")
    lines += ["\\midrule",
              "Cells & " + " & ".join(str(int(tab[m].sum())) for m in MODEL_LABEL) + f" & {int(tab['pooled'].sum())} \\\\"]
    if len(valid_all):
        lines.append(
            f"\\multicolumn{{5}}{{@{{}}p{{0.95\\columnwidth}}}}{{\\textit{{Fully valid cells ({len(valid_all)}): "
            f"mean violations {valid_all.v_start.mean():.1f} at retry 0 and {valid_all.v_end.mean():.1f} at the last "
            f"retry; per valid step {valid_all.rules_cleared.sum() / max(valid_all.valid_steps.sum(), 1):.2f} rules "
            f"cleared and {valid_all.rules_introduced.sum() / max(valid_all.valid_steps.sum(), 1):.2f} introduced.}}}} \\\\"
        )
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", ""]
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--e3", type=Path, default=Path("results/E3_full/e3_results.csv"))
    ap.add_argument("--out-dir", type=Path, default=Path("paper/tables"))
    args = ap.parse_args()

    e3 = pd.read_csv(args.e3)
    cells = build(e3)
    tab = summary(cells)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    cells.to_csv(args.out_dir / "e3_regimes_cells.csv", index=False)
    tab.to_csv(args.out_dir / "e3_regimes.csv")
    (args.out_dir / "e3_regimes.tex").write_text(to_latex(tab, cells))

    print(tab.to_string())
    fv = cells[~cells.regime.str.startswith("dropout")]
    print(f"\nfully valid cells: {len(fv)}; v_start mean {fv.v_start.mean():.2f}, v_end mean {fv.v_end.mean():.2f}, "
          f"min reached mean {fv.v_min.mean():.2f}")
    print(f"rule churn per valid step: cleared {fv.rules_cleared.sum()/max(fv.valid_steps.sum(),1):.2f}, "
          f"introduced {fv.rules_introduced.sum()/max(fv.valid_steps.sum(),1):.2f} (steps={fv.valid_steps.sum()})")
    print("example sequences per regime:")
    for reg in REGIMES:
        ex = cells[cells.regime == reg].head(2)
        for _, r in ex.iterrows():
            print(f"  {reg:18s} {MODEL_LABEL.get(r.model, r.model):22s} {r.task_id} rep{r.rep}: {r.sequence}")
    print(f"Wrote {args.out_dir}/e3_regimes{{.csv,.tex,_cells.csv}}")


if __name__ == "__main__":
    main()
