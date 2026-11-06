#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A4: precision per rater, rater B as the primary estimate, and the unit of SM-2.

Outputs under out/a4/:
  precision.csv            per rule and pooled: TP and precision for rater A, rater B,
                           adjudicated; Wilson and Clopper-Pearson 95%; agreement and kappa
  precision_check.csv      against paper/tables/precision_audit_v2.csv
  disagreements.csv        every A/B disagreement and the adjudicated label
  sm2_by_location.csv      SM-2 audit items split into the global and the per-line firing
  ruleset_by_rater.csv     the >= 90% rule set under each rater and criterion, and the
                           Table 13 quantities it gives (both gates, Safe over 130 and 135)
  safe_programs.csv        the programs that are violation-free under each such rule set
  sm2_units.csv            SM-2 firings per call site vs per program (LLM, translators, audit population)
  audit_population.csv     the audit population re-enumerated from the committed code
  rater_guide_quotes.md    the audit unit and the SM-2 criterion, verbatim
"""
from __future__ import annotations

import pandas as pd

from common import (AUDIT_DIR, REPO, RULES, RUN2_DIR, RUN2_E1, TABLES, clopper_pearson, cohen_kappa,
                    get_flag, load_code, load_e1_csv, out, rel, sha256, sr, task_caps, wd, wilson)

THRESH = 0.90


def load_sheets() -> pd.DataFrame:
    a = pd.read_csv(AUDIT_DIR / "sheet_rater_A.csv").set_index("item_id")
    b = pd.read_csv(AUDIT_DIR / "sheet_rater_B.csv").set_index("item_id")
    j = pd.read_csv(AUDIT_DIR / "sheet_adjudicated.csv").set_index("item_id")
    k = pd.read_csv(AUDIT_DIR / "key.csv").set_index("item_id")
    df = pd.DataFrame({"rule": a.rule, "location": a.location, "A": a.label, "B": b.label, "adj": j.label,
                       "adj_note": j.note, "source": k.source})
    assert (a.rule == b.rule).all() and (a.rule == j.rule).all() and (a.rule == k.rule).all()
    return df


def precision_table(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for rule, g in list(df.groupby("rule")) + [("pooled", df)]:
        rec = {"rule": rule, "n": len(g)}
        for who in ("A", "B", "adj"):
            lab = g[who]
            n = int(lab.isin(["TP", "FP"]).sum())
            tp = int((lab == "TP").sum())
            lo, hi = wilson(tp, n)
            clo, chi = clopper_pearson(tp, n)
            rec.update({f"tp_{who}": tp, f"n_{who}": n, f"unsure_{who}": int((lab == "UNSURE").sum()),
                        f"prec_{who}": tp / n if n else float("nan"),
                        f"wilson_lo_{who}": lo, f"wilson_hi_{who}": hi, f"cp_lo_{who}": clo, f"cp_hi_{who}": chi})
        for x, y in (("A", "B"), ("A", "adj"), ("B", "adj")):
            po, k, npair = cohen_kappa(g[x], g[y])
            rec.update({f"agree_{x}_{y}": po, f"kappa_{x}_{y}": k})
        rec["adj_equals_A_items"] = int((g.adj == g.A).sum())
        rec["adj_equals_B_items"] = int((g.adj == g.B).sum())
        rows.append(rec)
    return pd.DataFrame(rows)


def check_paper(p: pd.DataFrame) -> pd.DataFrame:
    ref = pd.read_csv(TABLES / "precision_audit_v2.csv")
    rows = []
    for _, r in ref.iterrows():
        m = p[p.rule == r.rule].iloc[0]
        rows.append({"rule": r.rule, "paper_tp_A": r.tp_A, "mine_tp_A": m.tp_A, "paper_tp_B": r.tp_B, "mine_tp_B": m.tp_B,
                     "paper_tp_adj": r.tp_adj, "mine_tp_adj": m.tp_adj,
                     "paper_agreement": r.agreement, "mine_agreement": m.agree_A_B,
                     "paper_kappa": r.kappa, "mine_kappa": m.kappa_A_B,
                     "paper_ci_B": r.ci_B, "mine_cp_B": f"[{100 * m.cp_lo_B:.1f}, {100 * m.cp_hi_B:.1f}]",
                     "match": bool(r.tp_A == m.tp_A and r.tp_B == m.tp_B and r.tp_adj == m.tp_adj
                                   and abs(r.agreement - m.agree_A_B) < 1e-9 and abs(r.kappa - m.kappa_A_B) < 1e-9
                                   and r.ci_B == f"[{100 * m.cp_lo_B:.1f}, {100 * m.cp_hi_B:.1f}]")})
    return pd.DataFrame(rows)


def keep_set_numbers(keep: set[str] | None, label: str) -> list[dict]:
    """Table 13 quantities for an arbitrary rule subset (preregistered SM-5), both gates."""
    caps = task_caps()
    progs = load_code(RUN2_E1, "baseline")
    n_files = len(progs)
    n_rows = len(load_e1_csv(RUN2_E1).query("condition == 'baseline'"))
    rows = []
    for gate_name, gate in (("lexical", sr.gate_lexical), ("structural", sr.gate_structural)):
        passing = [p for p in progs if gate(p["code"])]
        if keep is not None and not keep:
            rows.append({"rule_set": label, "rules": "(empty)", "gate": gate_name, "gate_pass": len(passing),
                         "viol_pct": float("nan"), "safe": float("nan"), "safe_over_130": "undefined",
                         "safe_over_135": "undefined", "mean": float("nan")})
            continue
        counts = [sr.firings(p["code"], caps.get(p["task_id"]), wd(), keep, False) for p in passing]
        n_viol = sum(1 for c in counts if c > 0)
        safe = len(passing) - n_viol
        rows.append({"rule_set": label, "rules": "all" if keep is None else "+".join(sorted(keep)), "gate": gate_name,
                     "gate_pass": len(passing), "viol_pct": 100 * n_viol / len(passing), "safe": safe,
                     "safe_over_130": f"{safe}/{n_files}", "safe_over_135": f"{safe}/{n_rows}",
                     "mean": sum(counts) / len(passing)})
    return rows


def rulesets(p: pd.DataFrame) -> tuple[pd.DataFrame, dict]:
    per = p[p.rule != "pooled"].set_index("rule")
    sets = {}
    for who, name in (("A", "rater A"), ("B", "rater B"), ("adj", "adjudicated")):
        sets[f"{name}, point estimate >= 90%"] = {r for r in per.index if per.loc[r, f"prec_{who}"] >= THRESH}
        sets[f"{name}, Wilson lower bound >= 90%"] = {r for r in per.index if per.loc[r, f"wilson_lo_{who}"] >= THRESH}
    rows = keep_set_numbers(None, "preregistered (all seven checks)")
    for label, keep in sets.items():
        rows += keep_set_numbers(keep, label)
    return pd.DataFrame(rows), sets


def gate_evidence(code: str) -> dict:
    """Which URScript calls admit the program through the lexical gate, and whether any of
    them sits outside a comment."""
    from enfield_llm.code_parser import URSCRIPT_CALL_PATTERN
    hits, live = [], 0
    for line in code.splitlines():
        code_part = line.split("#", 1)[0]
        for m in URSCRIPT_CALL_PATTERN.finditer(line):
            in_comment = m.start() >= len(code_part)
            hits.append(m.group(0).rstrip("( ") + (" [in comment]" if in_comment else ""))
            live += not in_comment
    return {"gate_calls": "; ".join(hits), "gate_calls_outside_comments": live}


def safe_programs(sets: dict) -> pd.DataFrame:
    progs = pd.read_csv(out("a3", "programs.csv"))
    llm = progs[(progs.set == "LLM baseline (E1, run 2, gate-passing)") & (progs.gate_lexical == 1)]
    rows = []
    for label, keep in [("preregistered (all seven checks)", set(RULES))] + list(sets.items()):
        if not keep:
            continue
        tot = llm[[f"pre_{r}" for r in sorted(keep)]].sum(axis=1)
        for _, r in llm[tot == 0].iterrows():
            code = (RUN2_E1 / "code" / r.program).read_text(errors="replace")
            rows.append({"rule_set": label, "rules": "+".join(sorted(keep)), "program": r.program, "model": r.model,
                         "task_id": r.task_id, "has_motion_call_syntax": r.has_motion_command,
                         "gate_structural": r.gate_structural, "pre_total_all_rules": r.pre_total,
                         **gate_evidence(code), "sha256": sha256((RUN2_E1 / "code" / r.program))})
    return pd.DataFrame(rows).drop_duplicates()


def sm2_units() -> tuple[pd.DataFrame, pd.DataFrame]:
    progs = pd.read_csv(out("a3", "programs.csv"))
    rows = []
    groups = [("LLM baseline E1 run 2, gate-passing", progs[(progs.set.str.startswith("LLM")) & (progs.gate_lexical == 1)])]
    groups += [(s, progs[progs.set == s]) for s in ("Translator", "Translator + preamble")]
    for name, g in groups:
        tot = g.pre_ParseError if False else g["pre_SM-2"]
        firing = g[tot > 0]
        rows.append({"set": name, "programs": len(g), "programs_firing_SM2": len(firing),
                     "firings_call_site": int(tot.sum()), "firings_global": int(g.sm2_global.sum()),
                     "firings_per_line": int(g.sm2_line.sum()),
                     "mean_per_program_all": tot.mean(),
                     "mean_per_firing_program_call_site": firing["pre_SM-2"].mean() if len(firing) else float("nan"),
                     "mean_per_firing_program_dedup": 1.0 if len(firing) else float("nan"),
                     "programs_with_global": int((g.sm2_global > 0).sum()),
                     "programs_with_per_line": int((g.sm2_line > 0).sum())})
    # Audit population, re-enumerated from the committed copies of the audited code directories.
    import precision_audit_sample as pas
    dirs = [RUN2_DIR / "E1_full" / "code", RUN2_DIR / "E2_full" / "code"]
    fir = pd.DataFrame(pas.enumerate_firings(dirs, wd(), pas.task_modes()))
    pop = fir.groupby("rule").size().reindex(RULES, fill_value=0).rename("firings").reset_index()
    pop_json = __import__("json").loads((AUDIT_DIR / "population.json").read_text())["population_by_rule"]
    pop["population_json"] = pop.rule.map(pop_json)
    pop["match"] = pop.firings == pop.population_json
    s2 = fir[fir.rule == "SM-2"]
    rows.append({"set": "audit population (E1+E2 run 2, gate-passing)", "programs": fir.source.nunique(),
                 "programs_firing_SM2": s2.source.nunique(), "firings_call_site": len(s2),
                 "firings_global": int((s2.location == "global").sum()),
                 "firings_per_line": int((s2.location != "global").sum()),
                 "mean_per_program_all": float("nan"),
                 "mean_per_firing_program_call_site": len(s2) / s2.source.nunique(),
                 "mean_per_firing_program_dedup": 1.0,
                 "programs_with_global": s2[s2.location == "global"].source.nunique(),
                 "programs_with_per_line": s2[s2.location != "global"].source.nunique()})
    return pd.DataFrame(rows), pop


def guide_quotes() -> None:
    g = REPO / "docs/review/PRECISION_AUDIT_RATER_GUIDE.md"
    lines = g.read_text().splitlines()
    md = [f"Source: `{rel(g)}` (git HEAD), verbatim.", ""]
    unit = next(i for i, l in enumerate(lines) if "one firing" in l)
    md += ["### Unit of the audit", "```"] + [f"{i + 1}: {lines[i]}" for i in range(unit - 2, unit + 5)] + ["```", ""]
    s = next(i for i, l in enumerate(lines) if l.startswith("**SM-2"))
    e = next(i for i in range(s + 1, len(lines)) if lines[i].startswith("**SM-3"))
    md += ["### SM-2 criterion", "```"] + [f"{i + 1}: {lines[i]}" for i in range(s, e)] + ["```"]
    out("a4", "rater_guide_quotes.md").write_text("\n".join(md) + "\n")


def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    df = load_sheets()
    p = precision_table(df)
    p.to_csv(out("a4", "precision.csv"), index=False)
    chk = check_paper(p)
    chk.to_csv(out("a4", "precision_check.csv"), index=False)
    dis = df[df.A != df.B].copy()
    dis["resolved_to"] = dis.apply(lambda r: "A" if r.adj == r.A else ("B" if r.adj == r.B else "other"), axis=1)
    dis.reset_index().to_csv(out("a4", "disagreements.csv"), index=False)

    loc = []
    for kind, g in (("global", df[(df.rule == "SM-2") & (df.location == "global")]),
                    ("per-line", df[(df.rule == "SM-2") & (df.location != "global")])):
        rec = {"SM-2 items": kind, "n": len(g)}
        for who in ("A", "B", "adj"):
            rec[f"tp_{who}"] = int((g[who] == "TP").sum())
        loc.append(rec)
    pd.DataFrame(loc).to_csv(out("a4", "sm2_by_location.csv"), index=False)

    rs, sets = rulesets(p)
    rs.to_csv(out("a4", "ruleset_by_rater.csv"), index=False)
    sp = safe_programs(sets)
    sp.to_csv(out("a4", "safe_programs.csv"), index=False)
    gc = []
    for cond in ("baseline", "safety"):
        for pr in load_code(RUN2_E1, cond):
            if sr.gate_lexical(pr["code"]):
                ev = gate_evidence(pr["code"])
                gc.append({"condition": cond, "program": pr["file"].name, "model": pr["model_disp"], **ev})
    gc = pd.DataFrame(gc)
    gc.to_csv(out("a4", "gate_calls.csv"), index=False)
    print(gc.groupby("condition").apply(lambda g: pd.Series({"gate_passing": len(g),
          "admitted_only_by_commented_call": int((g.gate_calls_outside_comments == 0).sum())})))
    units, pop = sm2_units()
    units.to_csv(out("a4", "sm2_units.csv"), index=False)
    pop.to_csv(out("a4", "audit_population.csv"), index=False)
    guide_quotes()

    pd.set_option("display.width", 250)
    cols = ["rule", "n", "tp_A", "prec_A", "wilson_lo_A", "wilson_hi_A", "tp_B", "prec_B", "wilson_lo_B", "wilson_hi_B",
            "tp_adj", "prec_adj", "agree_A_B", "kappa_A_B", "adj_equals_A_items", "adj_equals_B_items"]
    print(p[cols].round(3).to_string(index=False))
    print(chk[["rule", "match"]].to_string(index=False))
    print(dis.groupby(["rule", "resolved_to"]).size())
    print(pd.DataFrame(loc).to_string(index=False))
    print({k: sorted(v) for k, v in sets.items()})
    print(rs.round(2).to_string(index=False))
    print(sp[["rule_set", "program", "has_motion_call_syntax", "gate_structural", "gate_calls", "gate_calls_outside_comments"]].to_string(index=False))
    print(units.round(3).to_string(index=False))
    print(pop.to_string(index=False))


if __name__ == "__main__":
    main()
