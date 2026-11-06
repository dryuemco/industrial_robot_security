#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Numbers for the round-3 paper revision that no earlier script produces.

Outputs under out/r3/:
  count_decomposition.csv     per-rule mean firings per gate-passing output for E1 baseline,
                              E1 safety-prompted, each E2 strategy and E2 pooled, under the
                              preregistered and the corrected SM-5 rule sets (Table 14 contrasts)
  count_differences.csv       each condition minus E1 baseline, per rule
  count_check.csv             the per-rule totals against the gate-passing means of Table 14
  first_audit_rules.csv       SM-4 + SM-5 + SM-6 mean for the LLM baseline and both translators
  sm2_per_program.csv         SM-2 mean per program counted per call site and deduplicated per program
  gate_passing_violating.csv  gate-passing outputs that violate, E2 and the final E3 attempt
"""
from __future__ import annotations

import logging
import re
import warnings

import pandas as pd

from common import RULES, RUN2_DIR, RUN2_E1, get_flag, load_code, out, rule_counts, sr, task_caps

logging.disable(logging.INFO)
warnings.filterwarnings("ignore", category=FutureWarning)

E2_NAME = re.compile(r"^(T\d{3})_(.+?)_adversarial_(A8\.\d)_rep(\d)(\.invalid)?\.urscript$")
MODEL_TOKEN = {"qwen2.5-coder_32b": "Qwen2.5-Coder-32B", "deepseek-coder-v2_16b": "DeepSeek-Coder-V2-16B",
               "codellama_34b": "CodeLlama-34B"}


def program_rows() -> pd.DataFrame:
    caps = task_caps()
    items = []
    for cond in ("baseline", "safety"):
        for p in load_code(RUN2_E1, cond):
            items.append((f"E1 {cond}", p["model_disp"], p["task_id"], p["code"]))
    for f in sorted((RUN2_DIR / "E2_full" / "code").glob("*.urscript")):
        m = E2_NAME.match(f.name)
        if m:
            items.append((f"E2 {m.group(3)}", MODEL_TOKEN[m.group(2)], m.group(1), f.read_text(errors="replace")))
    rows = []
    for cond, model, tid, code in items:
        if not sr.gate_lexical(code):
            continue
        rc = rule_counts(code, caps.get(tid))
        rec = {"condition": cond, "model": model, "task_id": tid}
        for r in RULES:
            rec[f"pre_{r}"] = rc["pre"][r]
            rec[f"corr_{r}"] = rc["corr"][r]
        rows.append(rec)
    return pd.DataFrame(rows)


def decomposition(df: pd.DataFrame) -> pd.DataFrame:
    groups = [(c, df[df.condition == c]) for c in df.condition.unique()]
    groups.append(("E2 pooled", df[df.condition.str.startswith("E2")]))
    rows = []
    for name, g in groups:
        for basis in ("pre", "corr"):
            rec = {"condition": name, "basis": "preregistered" if basis == "pre" else "corrected-SM5",
                   "gate_passing": len(g)}
            for r in RULES:
                rec[r] = g[f"{basis}_{r}"].mean()
            rec["total"] = sum(rec[r] for r in RULES)
            rows.append(rec)
    return pd.DataFrame(rows)


def differences(dec: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for basis in ("preregistered", "corrected-SM5"):
        base = dec[(dec.condition == "E1 baseline") & (dec.basis == basis)].iloc[0]
        for _, r in dec[(dec.basis == basis) & (dec.condition != "E1 baseline")].iterrows():
            rec = {"contrast": f"{r.condition} minus E1 baseline", "basis": basis}
            for k in RULES + ["total"]:
                rec[k] = r[k] - base[k]
            rec["SM1+SM2"] = rec["SM-1"] + rec["SM-2"]
            rec["SM4+SM5+SM6"] = rec["SM-4"] + rec["SM-5"] + rec["SM-6"]
            rows.append(rec)
    return pd.DataFrame(rows)


def check(dec: pd.DataFrame) -> pd.DataFrame:
    """The preregistered totals must equal the gate-passing means of the result files
    (the source of Table 14 and of the safety-prompt contrast)."""
    e1 = pd.read_csv(RUN2_E1 / "e1_results.csv")
    e2 = pd.read_csv(RUN2_DIR / "E2_full" / "e2_results.csv")
    ref = {"E1 baseline": e1[e1.condition == "baseline"], "E1 safety": e1[e1.condition == "safety"],
           "E2 pooled": e2}
    for k in sorted(e2.condition.unique()):
        ref["E2 " + k.replace("adversarial_", "")] = e2[e2.condition == k]
    rows = []
    for _, r in dec[dec.basis == "preregistered"].iterrows():
        g = ref[r.condition]
        g = g[g.status == "success"]
        rows.append({"condition": r.condition, "gate_passing": r.gate_passing, "results_gate_passing": len(g),
                     "mean_total": r.total, "results_mean_total_violations": g.total_violations.mean(),
                     "match": bool(len(g) == r.gate_passing and abs(g.total_violations.mean() - r.total) < 1e-9)})
    df = pd.DataFrame(rows)
    assert df.match.all(), df
    return df


def first_audit_rules() -> pd.DataFrame:
    m = pd.read_csv(out("a3", "per_rule_means.csv"))
    m = m[m.model.isin(["pooled", "-"])]
    rows = []
    for _, r in m.iterrows():
        rows.append({"set": r["set"], "basis": r.basis, "SM-4": r["SM-4"], "SM-5": r["SM-5"], "SM-6": r["SM-6"],
                     "SM4+SM5+SM6": r["SM-4"] + r["SM-5"] + r["SM-6"], "SM1+SM2": r["SM-1"] + r["SM-2"],
                     "total": r.total})
    return pd.DataFrame(rows)


def sm2_per_program() -> pd.DataFrame:
    p = pd.read_csv(out("a3", "programs.csv"))
    rows = []
    for name, g in p[(p.gate_lexical == 1) | (p["set"] != p["set"].iloc[0])].groupby("set", sort=False):
        rows.append({"set": name, "programs": len(g), "firings_call_site": int(g["pre_SM-2"].sum()),
                     "programs_firing": int((g["pre_SM-2"] > 0).sum()),
                     "mean_per_program_call_site": g["pre_SM-2"].mean(),
                     "mean_per_program_deduplicated": (g["pre_SM-2"] > 0).mean(),
                     "firings_global": int(g.sm2_global.sum()), "firings_per_line": int(g.sm2_line.sum())})
    return pd.DataFrame(rows)


def gate_passing_violating() -> pd.DataFrame:
    """E2 outputs and final E3 attempts (as scripts/mcnemar_analysis.py selects them) that pass
    the gate, and how many of them violate at least one check."""
    import mcnemar_analysis as ma
    e2 = pd.read_csv(RUN2_DIR / "E2_full" / "e2_results.csv")
    df = ma.load_results(RUN2_DIR)
    e3 = df[df.condition == "watchdog"]
    rows = []
    for name, g in (("E2, all strategies", e2), ("E3, final attempt", e3)):
        g = g[g.status == "success"]
        rows.append({"set": name, "gate_passing": len(g), "violating": int((g.total_violations > 0).sum())})
    return pd.DataFrame(rows)


def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)
    df = program_rows()
    dec = decomposition(df)
    dec.to_csv(out("r3", "count_decomposition.csv"), index=False)
    dif = differences(dec)
    dif.to_csv(out("r3", "count_differences.csv"), index=False)
    chk = check(dec)
    chk.to_csv(out("r3", "count_check.csv"), index=False)
    fa = first_audit_rules()
    fa.to_csv(out("r3", "first_audit_rules.csv"), index=False)
    s2 = sm2_per_program()
    s2.to_csv(out("r3", "sm2_per_program.csv"), index=False)
    gv = gate_passing_violating()
    gv.to_csv(out("r3", "gate_passing_violating.csv"), index=False)
    print(gv.to_string(index=False))
    print(dec.round(3).to_string(index=False))
    print(dif.round(3).to_string(index=False))
    print(chk.round(3).to_string(index=False))
    print(fa.round(3).to_string(index=False))
    print(s2.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
