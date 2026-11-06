#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A2: the cross-model Cochran Q of VI-D and Table 16 on three bases.

  naive         the current definition: cell = any repetition violating, outputs that
                fail the gate count as non-violating (scripts/review/quantization_summary.py,
                scripts/mcnemar_analysis.py)
  gate-passing  cell = any gate-passing repetition violating; only tasks on which every
                model has >= 1 gate-passing output
  gate-pass     cell = any repetition passes the gate (status == "success")

Gate = status == "success" (equals the lexical gate on the stored programs; see
out/gate_equivalence.csv). Exact p from the within-block permutation distribution
(common.cochran_exact). Holm over the two conditions within each variant and basis.

Outputs under out/a2/.
"""
from __future__ import annotations

import logging
import warnings

import numpy as np
import pandas as pd

from common import (MODELS, REPO, RUN1_E1, RUN2_DIR, RUN2_E1, SERVED, TABLES, X10, cochran_exact,
                    get_flag, holm, load_e1_csv, load_x10, out, rel, set_flag)

logging.disable(logging.INFO)
warnings.filterwarnings("ignore", category=FutureWarning)

VARIANTS = {
    "as served (Q4_K_M / Q4_0 / Q4_0)": {"Qwen2.5-Coder-32B": "Q4_K_M", "DeepSeek-Coder-V2-16B": "Q4_0", "CodeLlama-34B": "Q4_0"},
    "all Q4_K_M": {m: "Q4_K_M" for m in MODELS},
    "all Q4_0": {m: "Q4_0" for m in MODELS},
}


def levels(conf: pd.DataFrame) -> dict[str, dict[str, pd.DataFrame]]:
    data = {}
    for mid, (disp, served) in SERVED.items():
        d = {served: conf[conf.model == mid]}
        if conf is not None:
            ctrl = X10[disp][2]
            d[ctrl] = load_x10(disp)
        data[disp] = d
    return data


def cell_matrix(data, choice, cond, basis) -> tuple[pd.DataFrame, int]:
    cols = {}
    for m, lvl in choice.items():
        d = data[m][lvl]
        d = d[d.condition == cond]
        if basis == "naive":
            cols[m] = d.groupby("task_id")["viol"].max()
        elif basis == "gate-passing":
            g = d[d.gate == 1]
            cols[m] = g.groupby("task_id")["viol"].max()
        elif basis == "gate-pass":
            cols[m] = d.groupby("task_id")["gate"].max()
    mat = pd.DataFrame(cols)
    n_before = len(mat)
    mat = mat.dropna()
    return mat[list(choice)], n_before


def run_tests(data, variants: dict, run: str) -> pd.DataFrame:
    rows = []
    for vname, choice in variants.items():
        for basis in ("naive", "gate-passing", "gate-pass"):
            block = []
            for cond in ("baseline", "safety"):
                mat, n_union = cell_matrix(data, choice, cond, basis)
                res = cochran_exact(mat.to_numpy(dtype=int))
                rec = {"run": run, "variant": vname, "basis": basis, "condition": cond,
                       "tasks_in_test": len(mat), "tasks_with_any_cell": n_union,
                       "n_discordant": res["n_discordant"],
                       "Q": res["Q"], "p_asymptotic": res["p_asymptotic"], "p_exact": res["p_exact"],
                       "exact_method": f"full enumeration of {res['n_arrangements']} arrangements" if res["n_arrangements"] else "",
                       "Q_note": "" if res["n_discordant"] else "undefined (no discordant blocks)"}
                for m in choice:
                    rec[f"rate_{m}"] = float(mat[m].mean()) if len(mat) else float("nan")
                block.append(rec)
            pa = holm([r["p_asymptotic"] for r in block])
            pe = holm([r["p_exact"] for r in block])
            for r, a, e in zip(block, pa, pe):
                r["p_asymptotic_holm2"] = a
                r["p_exact_holm2"] = e
            rows += block
    return pd.DataFrame(rows)


def verify_counts(conf: pd.DataFrame, run: str) -> pd.DataFrame:
    """A2.0: gate counts, naive cell rates, gate-passing per-generation rates (Table 14)."""
    adv = pd.read_csv(TABLES / "adversarial_table.csv")
    rows = []
    for mid, (disp, _) in SERVED.items():
        for cond in ("baseline", "safety"):
            d = conf[(conf.model == mid) & (conf.condition == cond)]
            g = d[d.gate == 1]
            cell_naive = d.groupby("task_id")["viol"].max()
            cell_gate = d.groupby("task_id")["gate"].max()
            rec = {"run": run, "model": disp, "condition": cond, "n": len(d), "gate_pass": len(g),
                   "naive_cell_rate": float(cell_naive.mean()),
                   "gate_passing_generation_viol_rate": float(g.viol.mean()) if len(g) else float("nan"),
                   "gate_passing_violating": int(g.viol.sum()),
                   "cells_naive_eq_gatepass": int((cell_naive == cell_gate).sum()),
                   "cells": len(cell_naive)}
            if cond == "baseline":
                a = adv[(adv.strategy == "baseline") & (adv.model == mid)].iloc[0]
                rec.update({"table14_gate_n": int(a.gate_n), "table14_viol_rate_gate": float(a.viol_rate_gate)})
            rows.append(rec)
    return pd.DataFrame(rows)


def vid_four_family(run_dirs: list, run: str) -> pd.DataFrame:
    """VI-D's Holm family as computed by scripts/mcnemar_analysis.py (four conditions)."""
    import mcnemar_analysis as ma
    frames = [ma.load_results(d) for d in run_dirs]
    df = pd.concat(frames, ignore_index=True)
    res = ma.run_cross_model_cochran_q(df)
    return pd.DataFrame([{"run": run, "condition": r.condition_label if hasattr(r, "condition_label") else r.condition,
                          "n_subjects": r.n_subjects, "Q": r.q_statistic, "p": r.p_value,
                          "p_holm_4family": r.p_adjusted} for r in res])


def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    conf2 = load_e1_csv(RUN2_E1)
    conf1 = load_e1_csv(RUN1_E1)

    v = pd.concat([verify_counts(conf2, "run2"), verify_counts(conf1, "run1")])
    v.to_csv(out("a2", "verify_counts.csv"), index=False)

    t2 = run_tests(levels(conf2), VARIANTS, "run2")
    # Run 1: the quantization controls were run once, against run 2; only the as-served
    # variant is a run-1 quantity.
    t1 = run_tests(levels(conf1), {k: VARIANTS[k] for k in ["as served (Q4_K_M / Q4_0 / Q4_0)"]}, "run1")
    tests = pd.concat([t2, t1], ignore_index=True)
    tests.to_csv(out("a2", "cochran_all.csv"), index=False)

    # A2.1 check: naive basis, run 2 vs paper/tables/quantization_cochran.csv
    ref = pd.read_csv(TABLES / "quantization_cochran.csv")
    chk = []
    for _, r in ref.iterrows():
        mine = t2[(t2.variant == r["set"]) & (t2.condition == r.condition) & (t2.basis == "naive")].iloc[0]
        rec = {"variant": r["set"], "condition": r.condition, "paper_Q": r.Q, "mine_Q": mine.Q,
               "paper_p": r.p, "mine_p_asymptotic": mine.p_asymptotic, "mine_p_exact": mine.p_exact}
        for m in MODELS:
            rec[f"paper_rate_{m}"] = r[f"rate_{m}"]
            rec[f"mine_rate_{m}"] = mine[f"rate_{m}"]
        rec["match"] = bool(abs(r.Q - mine.Q) < 1e-9 and abs(r.p - mine.p_asymptotic) < 1e-12
                            and all(abs(r[f"rate_{m}"] - mine[f"rate_{m}"]) < 1e-12 for m in MODELS))
        chk.append(rec)
    chk = pd.DataFrame(chk)
    chk.to_csv(out("a2", "table16_check.csv"), index=False)
    set_flag("table16_cochran_reproduced", bool(chk.match.all()))

    fam = pd.concat([vid_four_family([RUN2_DIR], "run2"),
                     vid_four_family([RUN1_E1, REPO / "data/confirmatory/run1_2026-04/e2_confirmatory",
                                      REPO / "data/confirmatory/run1_2026-04/e3_confirmatory"], "run1")])
    fam.to_csv(out("a2", "vid_four_condition_family.csv"), index=False)

    pd.set_option("display.width", 250)
    print(v.to_string(index=False))
    print(chk[["variant", "condition", "paper_Q", "mine_Q", "paper_p", "mine_p_asymptotic", "match"]].to_string(index=False))
    print(fam.to_string(index=False))
    cols = ["run", "variant", "basis", "condition", "tasks_in_test", "n_discordant", "Q", "p_asymptotic", "p_exact",
            "p_exact_holm2", "Q_note"] + [f"rate_{m}" for m in MODELS]
    print(tests[cols].to_string(index=False))


if __name__ == "__main__":
    main()
