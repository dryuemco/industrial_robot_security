#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A3: per-rule decomposition of the mean firings per program.

Outputs under out/a3/:
  programs.csv            one row per program (LLM baseline run 2, both translator variants):
                          gate, motion present, per-rule counts (preregistered and corrected SM-5),
                          SM-2 split into the global and the per-line firing
  per_rule_means.csv      mean firings per program per rule, by set (and per model)
  reviewer_check.csv      the values derived by the reviewer and by the authors against direct means
  differences.csv         LLM minus translator, per rule, both SM-5 definitions
  ratios.csv              candidate pairs for "about twice as many rule firings"
  never_fires.md          the "on which the reference never fires" sentence against the counts
"""
from __future__ import annotations

import re

import pandas as pd

from common import (PAPER, RULES, RUN2_E1, get_flag, load_code, out, rule_counts, sr, task_caps,
                    violations)

FIRST_MOTION = re.compile(r"\b(movej|movel|movec|servoj|speedl|speedj)\s*\(", re.I)
SETS = ["LLM baseline (E1, run 2, gate-passing)", "Translator", "Translator + preamble"]


def program_rows() -> pd.DataFrame:
    caps = task_caps()
    rows = []
    llm = load_code(RUN2_E1, "baseline")
    items = [(SETS[0], p["model_disp"], p["task_id"], p["rep"], p["file"].name, p["code"]) for p in llm]
    for variant, folder in (("Translator", "minimal"), ("Translator + preamble", "preamble")):
        for f in sorted((out("a1", "translator", folder, "x").parent).glob("*.urscript")):
            items.append((variant, "-", f.stem[:4], 0, f.name, f.read_text()))
    for set_name, model, tid, rep, name, code in items:
        rc = rule_counts(code, caps.get(tid))
        sm2 = [v for v in violations(code) if v.detection_mechanism == "SM-2"]
        rec = {"set": set_name, "model": model, "task_id": tid, "rep": rep, "program": name,
               "gate_lexical": int(sr.gate_lexical(code)), "gate_structural": int(sr.gate_structural(code)),
               "has_motion_command": int(bool(FIRST_MOTION.search(code))),  # URScript call syntax, as SM-6 reads it
               "sm2_global": sum(1 for v in sm2 if v.location == "global"),
               "sm2_line": sum(1 for v in sm2 if v.location != "global")}
        for r in RULES:
            rec[f"pre_{r}"] = rc["pre"][r]
            rec[f"corr_{r}"] = rc["corr"][r]
        rec["pre_total"] = sum(rc["pre"].values())
        rec["corr_total"] = sum(rc["corr"].values())
        rows.append(rec)
    df = pd.DataFrame(rows)
    df.to_csv(out("a3", "programs.csv"), index=False)
    return df


def means(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    llm = df[(df.set == SETS[0]) & (df.gate_lexical == 1)]
    groups = [(SETS[0], "pooled", llm)]
    groups += [(SETS[0], m, g) for m, g in llm.groupby("model", sort=False)]
    groups += [(s, "-", df[df.set == s]) for s in SETS[1:]]
    for name, model, g in groups:
        for basis in ("pre", "corr"):
            rec = {"set": name, "model": model, "basis": "preregistered" if basis == "pre" else "corrected-SM5",
                   "n_programs": len(g)}
            for r in RULES:
                rec[r] = g[f"{basis}_{r}"].mean()
                rec[f"{r}_programs_firing"] = int((g[f"{basis}_{r}"] > 0).sum())
            rec["total"] = g[f"{basis}_total"].mean()
            rows.append(rec)
    m = pd.DataFrame(rows)
    m.to_csv(out("a3", "per_rule_means.csv"), index=False)
    return m


def reviewer_check(m: pd.DataFrame) -> pd.DataFrame:
    """The reviewer derived the LLM values by differencing Table 13's Mean column."""
    t13 = pd.read_csv(out("a1", "table13_with_translator.csv"))
    lex = t13[(t13.gate == "lexical") & (t13.set == "LLM baseline (E1, run 2)")].set_index("rules")["mean"]
    derived = {"SM-4": lex["preregistered"] - lex["drop-SM4"],
               "SM-6": lex["SM6-only"],
               "SM-2": lex["SM2+SM6"] - lex["SM6-only"]}
    derived["SM-5"] = lex["high-precision"] - derived["SM-4"] - derived["SM-6"]
    derived["SM-1"] = lex["preregistered"] - lex["high-precision"] - derived["SM-2"]
    claims = [("LLM", "SM-1", 3.2, "reviewer"), ("LLM", "SM-2", 1.3, "reviewer"), ("LLM", "SM-4", 0.7, "reviewer"),
              ("LLM", "SM-5", 3.5, "reviewer"), ("LLM", "SM-6", 1.7, "reviewer"),
              ("Translator", "SM-4", 1.0, "authors"), ("Translator", "SM-6", 2.0, "authors")]
    rows = []
    for who, rule, val, src in claims:
        if who == "LLM":
            direct = m[(m.set == SETS[0]) & (m.model == "pooled") & (m.basis == "preregistered")].iloc[0][rule]
            via = derived[rule]
        else:
            direct = m[(m.set == "Translator") & (m.basis == "preregistered")].iloc[0][rule]
            via = float("nan")
        rows.append({"set": who, "rule": rule, "claimed": val, "claimed_by": src,
                     "direct_mean": direct, "via_table13_differencing": via,
                     "direct_rounded_1dp": round(direct, 1), "matches_1dp": round(direct, 1) == val})
    llm = m[(m.set == SETS[0]) & (m.model == "pooled") & (m.basis == "preregistered")].iloc[0]
    rows.append({"set": "LLM", "rule": "SM-3 and SM-7 (assumed 0 by the differencing)", "claimed": 0.0,
                 "claimed_by": "reviewer (implicit)", "direct_mean": llm["SM-3"] + llm["SM-7"],
                 "via_table13_differencing": float("nan"), "direct_rounded_1dp": round(llm["SM-3"] + llm["SM-7"], 1),
                 "matches_1dp": round(llm["SM-3"] + llm["SM-7"], 1) == 0.0})
    df = pd.DataFrame(rows)
    df.to_csv(out("a3", "reviewer_check.csv"), index=False)
    return df


def differences(m: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for basis in ("preregistered", "corrected-SM5"):
        llm = m[(m.set == SETS[0]) & (m.model == "pooled") & (m.basis == basis)].iloc[0]
        for ref in SETS[1:]:
            tr = m[(m.set == ref) & (m.basis == basis)].iloc[0]
            rec = {"basis": basis, "contrast": f"LLM minus {ref}"}
            for r in RULES + ["total"]:
                rec[r] = llm[r] - tr[r]
            excess = rec["total"]
            rec["SM1+SM2"] = rec["SM-1"] + rec["SM-2"]
            rec["SM1+SM2_share_of_excess"] = rec["SM1+SM2"] / excess if excess else float("nan")
            rec["LLM_SM1+SM2_share_of_LLM_total"] = (llm["SM-1"] + llm["SM-2"]) / llm["total"]
            rows.append(rec)
    df = pd.DataFrame(rows)
    df.to_csv(out("a3", "differences.csv"), index=False)
    return df


def ratios(m: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for basis in ("preregistered", "corrected-SM5"):
        llm = m[(m.set == SETS[0]) & (m.model == "pooled") & (m.basis == basis)].iloc[0]["total"]
        for ref in SETS[1:]:
            tr = m[(m.set == ref) & (m.basis == basis)].iloc[0]["total"]
            rows.append({"basis": basis, "numerator": "LLM gate-passing mean", "denominator": f"{ref} mean",
                         "llm_mean": llm, "ref_mean": tr, "ratio": llm / tr})
    df = pd.DataFrame(rows)
    df.to_csv(out("a3", "ratios.csv"), index=False)
    return df


def never_fires(m: pd.DataFrame, progs: pd.DataFrame) -> None:
    tex = PAPER.read_text().splitlines()
    hits = [i for i, l in enumerate(tex) if "never fires" in l]
    md = ["# \"on which the reference never fires\"", ""]
    for i in hits:
        md.append(f"paper/main_access.tex lines {i - 3}-{i + 1}:")
        md.append("```")
        md += [f"{j + 1}: {tex[j]}" for j in range(i - 4, i + 1)]
        md.append("```")
    md.append("")
    md.append("Programs firing SM-1 and SM-2 (preregistered rule set), from out/a3/per_rule_means.csv:")
    md.append("")
    md.append("| set | n | SM-1 programs | SM-2 programs | mean SM-2 global | mean SM-2 per-line |")
    md.append("|---|---|---|---|---|---|")
    for s in SETS:
        r = m[(m.set == s) & (m.model.isin(["pooled", "-"])) & (m.basis == "preregistered")].iloc[0]
        p = progs[progs.set == s]
        if s == SETS[0]:
            p = p[p.gate_lexical == 1]
        md.append(f"| {s} | {int(r.n_programs)} | {int(r['SM-1_programs_firing'])} | {int(r['SM-2_programs_firing'])} | "
                  f"{p.sm2_global.mean():.2f} | {p.sm2_line.mean():.2f} |")
    out("a3", "never_fires.md").write_text("\n".join(md) + "\n")


def main() -> None:
    if get_flag("data_mismatch") or not get_flag("translator_matches_paper_csv", False):
        print("STOP: data mismatch or translator counts differ from the paper CSV (see flags.json)")
        return
    progs = program_rows()
    m = means(progs)
    rc = reviewer_check(m)
    d = differences(m)
    r = ratios(m)
    never_fires(m, progs)
    pd.set_option("display.width", 250)
    print(m[["set", "model", "basis", "n_programs"] + RULES + ["total"]].round(3).to_string(index=False))
    print(rc.to_string(index=False))
    print(d.round(3).to_string(index=False))
    print(r.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
