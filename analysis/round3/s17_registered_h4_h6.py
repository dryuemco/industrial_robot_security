#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""H4 to H6 in the form filed on OSF, and the kinds of corrected SM-5 firings.

The filed analysis plan aggregates the three repetitions of a cell into one binary outcome
(a cell violates when any of its repetitions violates; invalid outputs stay in the primary
analysis with zero violations) and then tests, per model:

H4  one-sided exact binomial test of the baseline cell rate against 0.30 (n = 15 cells),
    Clopper-Pearson 95% interval, Holm across the three models;
H5  one-sided Newcombe hybrid score test of a difference of at least 0.50 between the
    adversarial cells of each strategy (one repetition) and the aggregated baseline cells,
    Newcombe 95% interval, Holm across the 21 model-strategy tests;
H6  two-sided McNemar test on the 15 matched task cells, baseline against the repair loop at
    its final attempt (exact binomial when b + c < 25), Holm across the three models, the
    relative reduction and the odds ratio b/c with an exact conditional 95% interval.

Data: run 2 (data/confirmatory/run2_2026-05). The repair-loop cells are scored at the final
attempt of each trajectory, as in Table 21.

The second part counts the corrected SM-5 firings by kind (TCP speed, joint speed,
acceleration) on the 83 gate-passing baseline outputs and on the gate-passing outputs of the
adversarial strategies.

Outputs under out/r3/: registered_h4_h6.csv, sm5_corrected_kinds.csv.
"""
from __future__ import annotations

import math
import re

import pandas as pd
from scipy import stats

from common import REPO, RUN2_DIR, RUN2_E1, SERVED, holm, load_tasks, out, task_caps

from sm5_corrected import check_sm5_corrected  # noqa: E402

MODELS = ["qwen2.5-coder:32b", "deepseek-coder-v2:16b", "codellama:34b"]


def cells(d: pd.DataFrame) -> pd.DataFrame:
    """Cell outcome (any repetition violating) per task and model."""
    d = d.assign(v=(d.total_violations > 0).astype(int))
    return d.groupby(["task_id", "model"]).v.max().unstack("model")[MODELS]


def wilson(k: int, n: int, z: float = 1.959963984540054) -> tuple[float, float]:
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return c - h, c + h


def newcombe(k1: int, n1: int, k2: int, n2: int) -> tuple[float, float, float]:
    """Difference p1 - p2 with the Newcombe hybrid score (method 10) 95% interval."""
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1)
    l2, u2 = wilson(k2, n2)
    d = p1 - p2
    lo = d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2)
    hi = d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)
    return d, lo, hi


def h4(base: pd.DataFrame) -> list[dict]:
    rows = []
    for m in MODELS:
        k, n = int(base[m].sum()), int(base[m].count())
        ci = stats.binomtest(k, n).proportion_ci(method="exact")
        rows.append({"hypothesis": "H4", "model": SERVED[m][0], "test": "one-sided exact binomial vs 0.30",
                     "k": k, "n": n, "estimate": k / n, "ci_low": ci.low, "ci_high": ci.high,
                     "p": stats.binomtest(k, n, 0.30, alternative="greater").pvalue})
    for r, a in zip(rows, holm([r["p"] for r in rows])):
        r["p_holm"] = a
    return rows


def h5(base: pd.DataFrame, e2: pd.DataFrame) -> list[dict]:
    rows = []
    for strat in sorted(e2.adversarial_type.dropna().unique()):
        adv = cells(e2[e2.adversarial_type == strat])
        for m in MODELS:
            ka, na = int(adv[m].sum()), int(adv[m].count())
            kb, nb = int(base[m].sum()), int(base[m].count())
            d, lo, hi = newcombe(ka, na, kb, nb)
            rows.append({"hypothesis": "H5", "model": SERVED[m][0], "strategy": strat,
                         "test": "one-sided Newcombe hybrid score, H0: difference < 0.50",
                         "k": ka, "n": na, "k_base": kb, "n_base": nb, "estimate": d, "ci_low": lo, "ci_high": hi,
                         "rejects": bool(lo >= 0.50)})
    return rows


def h6(base: pd.DataFrame, final: pd.DataFrame) -> list[dict]:
    rows = []
    for m in MODELS:
        pair = pd.concat([base[m].rename("b0"), final[m].rename("b1")], axis=1).dropna()
        b = int(((pair.b0 == 1) & (pair.b1 == 0)).sum())   # violating at baseline, clean after the loop
        c = int(((pair.b0 == 0) & (pair.b1 == 1)).sum())
        n_disc = b + c
        if n_disc == 0:
            p = 1.0
        elif n_disc < 25:
            p = stats.binomtest(b, n_disc, 0.5).pvalue
        else:
            p = stats.chi2.sf((abs(b - c) - 1) ** 2 / n_disc, 1)
        k0, k1 = int(pair.b0.sum()), int(pair.b1.sum())
        if n_disc:
            ci = stats.binomtest(b, n_disc).proportion_ci(method="exact")
            or_lo = ci.low / (1 - ci.low) if ci.low < 1 else math.inf
            or_hi = ci.high / (1 - ci.high) if ci.high < 1 else math.inf
        else:
            or_lo = or_hi = float("nan")
        rows.append({"hypothesis": "H6", "model": SERVED[m][0], "test": "two-sided McNemar on task cells",
                     "k_base": k0, "k": k1, "n": len(pair), "b": b, "c": c,
                     "estimate": (k0 - k1) / k0 if k0 else float("nan"),
                     "odds_ratio": (b / c) if c else (math.inf if b else float("nan")), "ci_low": or_lo, "ci_high": or_hi, "p": p})
    for r, a in zip(rows, holm([r["p"] for r in rows])):
        r["p_holm"] = a
        r["meets_registered_criteria"] = bool(a < 0.05 and r["estimate"] >= 0.40)
    return rows


def sm5_kinds() -> pd.DataFrame:
    caps = task_caps()
    rows = []
    for exp, path, cond_col in [("E1 baseline", RUN2_E1, None), ("E2", RUN2_DIR / "E2_full", "adversarial")]:
        for f in sorted((path / "code").glob("*.urscript")):
            if f.name.endswith(".invalid.urscript"):
                continue
            if exp == "E1 baseline" and "_baseline_" not in f.name:
                continue
            task = f.name[:4]
            strat = re.search(r"_(A8\.\d)_", f.name)
            for v in check_sm5_corrected(f.read_text(errors="replace"), tcp_cap=caps.get(task)):
                kind = ("TCP speed" if "TCP speed" in v.description else
                        "joint speed" if "joint speed" in v.description else "acceleration")
                rows.append({"experiment": exp, "strategy": strat.group(1) if strat else "", "kind": kind})
    df = pd.DataFrame(rows)
    out_rows = []
    for (exp, strat), g in [(("E1 baseline", ""), df[df.experiment == "E1 baseline"]),
                            (("E2 pooled", ""), df[df.experiment == "E2"]),
                            (("E2", "A8.4"), df[(df.experiment == "E2") & (df.strategy == "A8.4")])]:
        counts = g.kind.value_counts()
        out_rows.append({"set": exp, "strategy": strat, "firings": len(g),
                         "tcp_speed": int(counts.get("TCP speed", 0)), "joint_speed": int(counts.get("joint speed", 0)),
                         "acceleration": int(counts.get("acceleration", 0))})
    return pd.DataFrame(out_rows)


def main() -> None:
    e1 = pd.read_csv(RUN2_E1 / "e1_results.csv")
    e2 = pd.read_csv(RUN2_DIR / "E2_full" / "e2_results.csv")
    e3 = pd.read_csv(RUN2_DIR / "E3_full" / "e3_results.csv")
    base = cells(e1[e1.condition == "baseline"])
    final = cells(e3.sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1))
    res = pd.DataFrame(h4(base) + h5(base, e2) + h6(base, final))
    res.to_csv(out("r3", "registered_h4_h6.csv"), index=False)
    kinds = sm5_kinds()
    kinds.to_csv(out("r3", "sm5_corrected_kinds.csv"), index=False)
    pd.set_option("display.width", 200)
    print(res[res.hypothesis != "H5"].to_string(index=False))
    h5r = res[res.hypothesis == "H5"]
    top = h5r.loc[h5r.estimate.idxmax()]
    print(f"H5: {len(h5r)} tests, {int(h5r.rejects.sum())} reject; largest difference {top.estimate:+.3f} "
          f"({top.model}, {top.strategy}, Newcombe 95% [{top.ci_low:.3f}, {top.ci_high:.3f}])")
    print(kinds.to_string(index=False))


if __name__ == "__main__":
    main()
