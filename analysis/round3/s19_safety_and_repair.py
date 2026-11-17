#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Round-3 numbers that no earlier script produces.

  safety_binary.csv                 E1 (run 2), baseline and safety-prompted: gate-passing outputs,
                                    violating outputs among them, the naive rate over all generations
  safety_clean_outputs.csv          the gate-passing outputs without any firing, with the result of the
                                    structural gate of the sensitivity analysis
  repair_sm6.csv                    repair under the preamble check (SM-6) alone, for the 135 local and
                                    the 60 frontier trajectories: trajectories that start from a
                                    gate-passing output that SM-6 flags, those with a later gate-passing
                                    attempt without the firing, those that end on one
  repair_by_check.csv               for each check, the trajectories whose first output is gate-passing
                                    and flagged by it, and those whose last gate-passing attempt is free
                                    of it (the rule-level reading of the repair loop in Section VI-E)
  repair_sm1_cleared.csv            the local trajectories counted as clearing the input-validation
                                    check (SM-1) in repair_by_check.csv: for the first output, the lines
                                    SM-1 flags, how many of them already carry a= and v= and how many
                                    pass the motion target through a nested call; the guard lines (if,
                                    elif, while, assert) of the first and of the last gate-passing
                                    output; nested-call motion lines left in the last one
  repair_sm6_ends.csv               the frontier trajectories that repair_sm6.csv counts as ending on a
                                    gate-passing output without the SM-6 firing: the first motion line
                                    SM-6 recognises in the final program, whether set_tcp and
                                    set_payload appear before it, and the checks that still fire
  conservative_interval.csv        Wilson interval for the baseline rate 82 of 135 with the 45 cells
                                    as the sample size
  safety_fixed_weights.csv          fall of the gate-passing mean under the safety prompt, pooled and
                                    with the baseline composition of the pool held fixed
  registered_h4_h6_sensitivity.csv  H4 to H6 in the registered form after excluding the outputs that
                                    fail the validity gate, with the cells left per model and test,
                                    and the refusal count behind the refusal sensitivity analysis
  crossmodel_one_retry.csv          the cross-model Cochran Q of the repair loop after one retry, on the
                                    three bases of Table 21
  e3_endpoints.csv                  gate-passing outputs of the repair loop and those that violate,
                                    after one retry and at the final attempt
and under out/a3/:
  excess_by_check.csv               the excess of generated code over each translation by check,
                                    corrected SM-5, from the unrounded means of out/a3/differences.csv

Outputs are under out/r3/ unless named otherwise.
"""
from __future__ import annotations

import re

import pandas as pd

from common import REPO, RUN2_DIR, RUN2_E1, cochran_exact, load_code, out, sr, wilson
from enfield_watchdog_static.rules.sm1_input_validation import check_sm1_input_validation

import s17_registered_h4_h6 as s17

FRONTIER_E3 = REPO / "data" / "exploratory" / "E3_frontier" / "e3_results.csv"
LOCAL_E3 = RUN2_DIR / "E3_full" / "e3_results.csv"
DISP = {"qwen2.5-coder:32b": "Qwen2.5-Coder-32B", "deepseek-coder-v2:16b": "DeepSeek-Coder-V2-16B",
        "codellama:34b": "CodeLlama-34B"}
CHECKS = ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]


def fired(row, check: str) -> bool:
    return check in str(row["violation_types"]).split(",")


# --------------------------------------------------------------------------- a

def safety_binary() -> tuple[pd.DataFrame, pd.DataFrame]:
    e1 = pd.read_csv(RUN2_E1 / "e1_results.csv")
    rows = []
    for cond in ("baseline", "safety"):
        g = e1[e1.condition == cond]
        gp = g[g.status == "success"]
        k = int((gp.total_violations > 0).sum())
        rows.append({"condition": cond, "generations": len(g), "gate_passing": len(gp), "violating": k,
                     "gate_passing_rate": k / len(gp), "naive_rate": k / len(g),
                     "gate_passing_without_firing": int((gp.total_violations == 0).sum())})
    binary = pd.DataFrame(rows)
    code = {(p["task_id"], p["model"], p["rep"]): p for p in load_code(RUN2_E1, "safety")}
    clean = []
    g = e1[(e1.condition == "safety") & (e1.status == "success") & (e1.total_violations == 0)]
    for _, r in g.sort_values(["model", "task_id", "rep"]).iterrows():
        p = code[(r.task_id, r.model, int(r.rep))]
        clean.append({"model": DISP[r.model], "task_id": r.task_id, "rep": int(r.rep), "file": p["file"].name,
                      "lexical_gate": sr.gate_lexical(p["code"]), "structural_gate": sr.gate_structural(p["code"])})
    return binary, pd.DataFrame(clean)


# --------------------------------------------------------------------------- b

def trajectories(path) -> list[list[dict]]:
    d = pd.read_csv(path).sort_values(["model", "task_id", "rep", "retry"])
    return [g.to_dict("records") for _, g in d.groupby(["model", "task_id", "rep"])]


def repair_sm6() -> tuple[pd.DataFrame, pd.DataFrame]:
    rows, by_check = [], []
    for name, path in (("local (E3, run 2)", LOCAL_E3), ("frontier (E3 frontier)", FRONTIER_E3)):
        trajs = trajectories(path)
        ok = lambda r: r["status"] == "success"  # noqa: E731
        start = [t for t in trajs if ok(t[0]) and fired(t[0], "SM-6")]
        later = [t for t in start if any(ok(r) and not fired(r, "SM-6") for r in t[1:])]
        end = [t for t in start if ok(t[-1]) and not fired(t[-1], "SM-6") and len(t) > 1]
        rows.append({"trajectories": name, "total": len(trajs), "start_gate_passing_flagged": len(start),
                     "later_gate_passing_without_firing": len(later), "end_on_gate_passing_without_firing": len(end)})
        for c in CHECKS:
            s = [t for t in trajs if ok(t[0]) and fired(t[0], c)]
            cleared = []
            for t in s:
                valid_later = [r for r in t[1:] if ok(r)]
                if valid_later and not fired(valid_later[-1], c):
                    cleared.append(t)
            by_check.append({"trajectories": name, "check": c, "first_output_gate_passing_and_flagged": len(s),
                             "last_gate_passing_attempt_free_of_it": len(cleared),
                             "share": len(cleared) / len(s) if s else float("nan")})
    return pd.DataFrame(rows), pd.DataFrame(by_check)


SM1_MOTION = re.compile(r"\b(movej|movel|movec)\s*\(", re.IGNORECASE)
NESTED = re.compile(r"\b(movej|movel|movec)\s*\(\s*[A-Za-z_]\w*\s*\(", re.IGNORECASE)
GUARD = re.compile(r"^\s*(if|elif|while|assert)\b")
SM6_MOTION = re.compile(r"\b(movej|movel|movec|servoj|speedl|speedj)\s*\(", re.IGNORECASE)


def e3_program(path, r) -> str:
    """The stored program of one E3 record (local or frontier)."""
    retry = int(r["retry"])
    model = r["model"].replace("/", "_").replace(":", "_")
    name = f"{r['task_id']}_{model}_baseline_rep{int(r['rep'])}"
    name += f"_retry{retry}.urscript" if retry else ".urscript"
    return (path.parent / "code" / name).read_text(errors="replace")


def repair_sm6_ends() -> pd.DataFrame:
    """The frontier trajectories counted in repair_sm6.csv as ending without the SM-6 firing:
    whether the final program has a motion command SM-6 recognises, whether set_tcp and
    set_payload appear before it, and the checks that still fire."""
    rows = []
    ok = lambda r: r["status"] == "success"  # noqa: E731
    for t in trajectories(FRONTIER_E3):
        if not (ok(t[0]) and fired(t[0], "SM-6") and len(t) > 1 and ok(t[-1]) and not fired(t[-1], "SM-6")):
            continue
        code = e3_program(FRONTIER_E3, t[-1])
        lines = code.split("\n")
        first = next((i for i, ln in enumerate(lines) if SM6_MOTION.search(ln)), None)
        before = "\n".join(lines[:first]) if first is not None else ""
        rows.append({
            "model": t[-1]["model"], "task_id": t[-1]["task_id"], "rep": int(t[-1]["rep"]),
            "final_retry": int(t[-1]["retry"]),
            "first_motion_line": first + 1 if first is not None else None,
            "set_tcp_before_motion": bool(re.search(r"\bset_tcp\s*\(", before, re.IGNORECASE)),
            "set_payload_before_motion": bool(re.search(r"\bset_payload\s*\(", before, re.IGNORECASE)),
            "checks_still_firing": t[-1]["violation_types"],
            "total_violations": int(t[-1]["total_violations"]),
        })
    return pd.DataFrame(rows)


def repair_sm1_cleared() -> pd.DataFrame:
    """The local trajectories that repair_by_check.csv counts as clearing SM-1, line by line."""

    def program(r) -> str:
        return e3_program(LOCAL_E3, r)

    def sm1_lines(code: str) -> list[str]:
        lines = code.split("\n")
        hit = sorted({int(v.location.split(":")[1]) for v in check_sm1_input_validation(code)})
        return [lines[i - 1] for i in hit]

    def nested(code: str) -> int:
        return sum(1 for ln in code.split("\n") if NESTED.search(ln))

    rows = []
    ok = lambda r: r["status"] == "success"  # noqa: E731
    for t in trajectories(LOCAL_E3):
        if not (ok(t[0]) and fired(t[0], "SM-1")):
            continue
        valid_later = [r for r in t[1:] if ok(r)]
        if not valid_later or fired(valid_later[-1], "SM-1"):
            continue
        first, last = program(t[0]), program(valid_later[-1])
        flagged = sm1_lines(first)
        rows.append({
            "model": DISP[t[0]["model"]], "task_id": t[0]["task_id"], "rep": int(t[0]["rep"]),
            "last_gate_passing_retry": int(valid_later[-1]["retry"]),
            "sm1_lines_first": len(flagged),
            "sm1_lines_first_with_a_and_v": sum(1 for ln in flagged
                                                if re.search(r"\bv\s*=", ln) and re.search(r"\ba\s*=", ln)),
            "sm1_lines_first_nested_call": sum(1 for ln in flagged if NESTED.search(ln)),
            "guard_lines_first": sum(1 for ln in first.split("\n") if GUARD.match(ln)),
            "guard_lines_last": sum(1 for ln in last.split("\n") if GUARD.match(ln)),
            "nested_call_motion_lines_last": nested(last),
        })
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- c

def conservative_interval() -> pd.DataFrame:
    k, n_calls, n_cells = 82, 135, 45
    p = k / n_calls
    lo, hi = wilson(p * n_cells, n_cells)
    return pd.DataFrame([{"violating": k, "generations": n_calls, "effective_n": n_cells, "rate": p,
                          "wilson_low": lo, "wilson_high": hi}])


# --------------------------------------------------------------------------- d

def safety_fixed_weights() -> pd.DataFrame:
    e1 = pd.read_csv(RUN2_E1 / "e1_results.csv")
    gp = e1[e1.status == "success"]
    b = gp[gp.condition == "baseline"].groupby("model").total_violations.agg(["count", "mean"])
    s = gp[gp.condition == "safety"].groupby("model").total_violations.agg(["count", "mean"])
    w = b["count"] / b["count"].sum()
    base_mean = float((b["mean"] * w).sum())
    pooled_safety = float(gp[gp.condition == "safety"].total_violations.mean())
    fixed_safety = float((s["mean"] * w).sum())
    rows = [{"model": DISP[m], "baseline_gate_passing": int(b.loc[m, "count"]), "baseline_mean": b.loc[m, "mean"],
             "safety_gate_passing": int(s.loc[m, "count"]), "safety_mean": s.loc[m, "mean"],
             "fall": b.loc[m, "mean"] - s.loc[m, "mean"]} for m in DISP]
    rows.append({"model": "pooled (each pool's own composition)", "baseline_gate_passing": int(b["count"].sum()),
                 "baseline_mean": base_mean, "safety_gate_passing": int(s["count"].sum()), "safety_mean": pooled_safety,
                 "fall": base_mean - pooled_safety})
    rows.append({"model": "baseline composition held fixed", "baseline_gate_passing": int(b["count"].sum()),
                 "baseline_mean": base_mean, "safety_gate_passing": int(s["count"].sum()), "safety_mean": fixed_safety,
                 "fall": base_mean - fixed_safety})
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- e

def excess_by_check() -> pd.DataFrame:
    d = pd.read_csv(out("a3", "differences.csv"))
    d = d[d.basis == "corrected-SM5"]
    rows = []
    for _, r in d.iterrows():
        rows.append({"contrast": r.contrast, "basis": r.basis, "SM-1+SM-2": r["SM-1"] + r["SM-2"], "SM-4": r["SM-4"],
                     "SM-5": r["SM-5"], "SM-6": r["SM-6"], "total": r.total})
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- f

def registered_sensitivity() -> pd.DataFrame:
    e1 = pd.read_csv(RUN2_E1 / "e1_results.csv")
    e2 = pd.read_csv(RUN2_DIR / "E2_full" / "e2_results.csv")
    e3 = pd.read_csv(LOCAL_E3)
    refusals = int(sum(d.refusal.astype(str).eq("True").sum() for d in (e1, e2, e3)))
    one_all = e3[e3.retry <= 1].sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1)
    fin_all = e3.sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1)
    valid = lambda d: d[d.status == "success"]  # noqa: E731
    base = s17.cells(valid(e1[e1.condition == "baseline"]))
    res = pd.DataFrame(s17.h4(base) + s17.h5(base, valid(e2)) + s17.h6(base, s17.cells(valid(one_all)), "one retry (registered)")
                       + s17.h6(base, s17.cells(valid(fin_all)), "final attempt (extension)"))
    res.insert(0, "analysis", "invalid outputs excluded")
    res["refusals_observed"] = refusals
    return res


# --------------------------------------------------------------------------- g

def crossmodel_one_retry() -> pd.DataFrame:
    import s07_sm6_and_crossmodel as s07
    e3 = pd.read_csv(LOCAL_E3)
    e3["has_violation"] = (e3.total_violations > 0).astype(int)
    e3["gate"] = (e3.status == "success").astype(int)
    rows = []
    for endpoint, sel in (("one retry (registered)", e3[e3.retry <= 1]), ("final attempt (extension)", e3)):
        d = sel.sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1)
        for basis in ("naive", "gate-passing", "gate-pass"):
            mat = s07.matrix(d, basis)
            r = cochran_exact(mat.to_numpy(dtype=int))
            rec = {"endpoint": endpoint, "basis": basis, "tasks_in_test": len(mat), "n_discordant": r["n_discordant"],
                   "Q": r["Q"], "p_asymptotic": r["p_asymptotic"], "p_exact": r["p_exact"]}
            for m in sorted(mat.columns):
                rec[f"rate_{m}"] = float(mat[m].mean()) if len(mat) else float("nan")
            rows.append(rec)
    return pd.DataFrame(rows)


def e3_endpoints() -> pd.DataFrame:
    """Gate-passing outputs and those that violate, after one retry and at the final attempt."""
    e3 = pd.read_csv(LOCAL_E3)
    rows = []
    for endpoint, sel in (("one retry (registered)", e3[e3.retry <= 1]), ("final attempt (extension)", e3)):
        d = sel.sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1)
        g = d[d.status == "success"]
        rows.append({"endpoint": endpoint, "trajectories": len(d), "gate_passing": len(g),
                     "gate_passing_violating": int((g.total_violations > 0).sum())})
    return pd.DataFrame(rows)


def main() -> None:
    pd.set_option("display.width", 220)
    binary, clean = safety_binary()
    binary.to_csv(out("r3", "safety_binary.csv"), index=False)
    clean.to_csv(out("r3", "safety_clean_outputs.csv"), index=False)
    rep, by_check = repair_sm6()
    rep.to_csv(out("r3", "repair_sm6.csv"), index=False)
    by_check.to_csv(out("r3", "repair_by_check.csv"), index=False)
    sm1 = repair_sm1_cleared()
    sm1.to_csv(out("r3", "repair_sm1_cleared.csv"), index=False)
    sm6_ends = repair_sm6_ends()
    sm6_ends.to_csv(out("r3", "repair_sm6_ends.csv"), index=False)
    ci = conservative_interval()
    ci.to_csv(out("r3", "conservative_interval.csv"), index=False)
    fw = safety_fixed_weights()
    fw.to_csv(out("r3", "safety_fixed_weights.csv"), index=False)
    ex = excess_by_check()
    ex.to_csv(out("a3", "excess_by_check.csv"), index=False)
    sens = registered_sensitivity()
    sens.to_csv(out("r3", "registered_h4_h6_sensitivity.csv"), index=False)
    cm = crossmodel_one_retry()
    cm.to_csv(out("r3", "crossmodel_one_retry.csv"), index=False)
    ep = e3_endpoints()
    ep.to_csv(out("r3", "e3_endpoints.csv"), index=False)
    for df in (binary, clean, rep, by_check, sm1, sm6_ends, ci, fw, ex, cm, ep):
        print(df.to_string(index=False), end="\n\n")
    cols = ["hypothesis", "model", "strategy", "endpoint", "k", "n", "k_base", "n_base", "estimate", "p", "p_holm",
            "rejects", "meets_registered_criteria"]
    print(sens[[c for c in cols if c in sens.columns]].to_string(index=False))
    print("refusals observed:", int(sens.refusals_observed.iloc[0]))


if __name__ == "__main__":
    main()
