#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Follow-up 1 (F1 to F7) to the round-3 analyses for Access-2026-55984.

Outputs under out/f1/:
  f1_programs.csv              one row per stored E1 program (both conditions): gates, SM-6 split
  f1_sm6_groups.csv            SM-6 split into four groups per condition, gate and model
  f1_single_rule.csv           violation rate, Wilson 95% and Safe for the payload-only, the
                               TCP-only and the full SM-6 rule, per condition, gate and model
  f2_safety_vs_baseline.csv    payload-missing rate, baseline against safety-prompted, per model
  f3_sm6_audit_split.csv       SM-6 audit items split into set_tcp and set_payload firings,
                               precision for rater A, rater B and adjudicated
  f3_sm6_audit_items.csv       the 20 SM-6 items with all three labels
  f4_osf_cross_model.md        Amendment 1 text on the cross-model test and the conditions, verbatim
  f4_locations.md              where adversarial_any and watchdog are computed and where they appear
  f4_cochran_four_conditions.csv  the four conditions of scripts/mcnemar_analysis.py on three bases
  f4_cochran_adversarial_subtypes.csv  one test per A8.k subtype, three bases
  f4_naive_vs_gatepass_cells.csv  (task, model) cells equal on the naive and the gate-pass basis
  f4_gate_equivalence_e2_e3.csv   status == "success" against the lexical gate on the stored E2/E3 code
  f5_table16_all_bases.csv     the six Table 16 lines on the naive, gate-passing and gate-pass bases
  f6_t009.md                   T009 DeepSeek rep 1: program, gates, has_motion code, paper lines
  f7_sm6_only_safe.csv         the programs with no SM-6 firing among the gate-passing baseline outputs
"""
from __future__ import annotations

import json
import logging
import re
import subprocess
import warnings
import zipfile

import pandas as pd

from common import (MODELS, PAPER, REPO, RUN1_E1, RUN2_DIR, RUN2_E1, cochran_exact, get_flag, holm,
                    load_code, load_e1_csv, load_tasks, md_table, out, rel, sha256, sr, violations, wd, wilson)

logging.disable(logging.INFO)
warnings.filterwarnings("ignore", category=FutureWarning)

from enfield_watchdog_static.rules.sm6_missing_preamble import _FIRST_MOTION_RE  # noqa: E402

SET_PAYLOAD = re.compile(r"\bset_payload\s*\(", re.I)
SET_TCP = re.compile(r"\bset_tcp\s*\(", re.I)
# First argument as a numeric literal, positional or as mass=, mass:= or mass: (variables are not resolved)
PAYLOAD_ARG = re.compile(r"\bset_payload\s*\(\s*(?:mass\s*(?::=|=|:)\s*)?([-+]?\d*\.?\d+)", re.I)
ANY_MOTION_WORD = re.compile(r"\b(movej|movel|movec|movep|movl|servoj|speedl|speedj)\b", re.I)
GATES = (("lexical", sr.gate_lexical), ("structural", sr.gate_structural))
OSF_AMENDMENT_1 = "69d3c5c421a5eb766f7a850e"
OSF_LATER = ["6a02e67cf944b975e74bbdbd", "6a076690b12b31eaddaaca6d", "6ab196ba288fe69e4d07a19f"]
SUPP_ZIPS = [REPO / "docs/review/submission_v5/04_supplementary/audit_v2.zip",
             REPO / "docs/review/submission_v5/04_supplementary/confirmatory_run_comparison.zip"]
T009 = "T009_deepseek-coder-v2_16b_baseline_rep1.urscript"


# ----------------------------------------------------------------------------- F1, F2

def sm6_program(code: str, payload_ir: float | None) -> dict:
    """SM-6 firings of one program, split by the call they report missing, plus the
    position of set_payload relative to the first motion call (as SM-6 reads it)."""
    vs = [v for v in violations(code) if v.detection_mechanism == "SM-6"]
    tcp = any(v.metadata.get("missing") == "set_tcp" for v in vs)
    pay = any(v.metadata.get("missing") == "set_payload" for v in vs)
    lines = code.split("\n")
    fm = next((i for i, l in enumerate(lines) if _FIRST_MOTION_RE.search(l)), None)
    head = "\n".join(lines[:fm]) if fm is not None else ""
    arg = PAYLOAD_ARG.search(head)
    return {"tcp_missing": tcp, "payload_missing": pay, "has_urscript_motion_call": fm is not None,
            "set_payload_anywhere": bool(SET_PAYLOAD.search(code)), "set_tcp_anywhere": bool(SET_TCP.search(code)),
            "set_payload_before_motion": bool(SET_PAYLOAD.search(head)),
            "set_payload_literal_equals_ir": bool(arg and payload_ir is not None
                                                and abs(float(arg.group(1)) - payload_ir) < 1e-9)}


def program_table() -> pd.DataFrame:
    tasks = load_tasks()
    rows = []
    for cond in ("baseline", "safety"):
        for p in load_code(RUN2_E1, cond):
            ir = tasks[p["task_id"]]["robot"].get("payload_kg")
            rec = {"condition": cond, "program": p["file"].name, "task_id": p["task_id"], "model": p["model_disp"],
                   "rep": p["rep"], "gate_lexical": sr.gate_lexical(p["code"]),
                   "gate_structural": sr.gate_structural(p["code"]), "payload_kg_ir": ir}
            rec.update(sm6_program(p["code"], float(ir) if ir is not None else None))
            rows.append(rec)
    return pd.DataFrame(rows)


def groups(progs: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for cond in ("baseline", "safety"):
        for gname, _ in GATES:
            g_all = progs[(progs.condition == cond) & progs[f"gate_{gname}"]]
            for model, g in [("pooled", g_all)] + [(m, g_all[g_all.model == m]) for m in MODELS]:
                t, p = g.tcp_missing, g.payload_missing
                neither = g[~t & ~p]
                rec = {"condition": cond, "gate": gname, "model": model, "n": len(g),
                       "only_tcp_missing": int((t & ~p).sum()), "both_missing": int((t & p).sum()),
                       "only_payload_missing": int((~t & p).sum()), "neither_missing": len(neither),
                       "neither: no URScript motion call (SM-6 cannot fire)": int((~neither.has_urscript_motion_call).sum()),
                       "neither: both set before first motion call": int(neither.has_urscript_motion_call.sum()),
                       "payload_missing": int(p.sum()),
                       "payload_missing: set_payload absent": int((p & ~g.set_payload_anywhere).sum()),
                       "payload_missing: set_payload only after first motion": int((p & g.set_payload_anywhere).sum()),
                       "set_payload_before_motion": int(g.set_payload_before_motion.sum()),
                       "set_payload_before_motion_literal_equals_ir": int(g.set_payload_literal_equals_ir.sum())}
                assert rec["only_tcp_missing"] + rec["both_missing"] + rec["only_payload_missing"] + rec["neither_missing"] == len(g)
                rows.append(rec)
    return pd.DataFrame(rows)


def single_rules(progs: pd.DataFrame) -> pd.DataFrame:
    e1 = load_e1_csv(RUN2_E1)
    rules = {"SM-6 set_payload only": lambda g: g.payload_missing,
             "SM-6 set_tcp only": lambda g: g.tcp_missing,
             "SM-6 (both clauses) = SM6-only": lambda g: g.payload_missing | g.tcp_missing}
    rows = []
    for cond in ("baseline", "safety"):
        files = progs[progs.condition == cond]
        rows_cond = e1[e1.condition == cond]
        for gname, _ in GATES:
            g_all = files[files[f"gate_{gname}"]]
            for model, g in [("pooled", g_all)] + [(m, g_all[g_all.model == m]) for m in MODELS]:
                n_files = len(files) if model == "pooled" else int((files.model == model).sum())
                n_rows = len(rows_cond) if model == "pooled" else int((rows_cond.model_disp == model).sum())
                for rname, f in rules.items():
                    k = int(f(g).sum())
                    lo, hi = wilson(k, len(g))
                    safe = len(g) - k
                    rows.append({"condition": cond, "gate": gname, "model": model, "rule": rname,
                                 "gate_pass": len(g), "violating": k, "viol_rate": k / len(g) if len(g) else float("nan"),
                                 "wilson_lo": lo, "wilson_hi": hi, "safe": safe,
                                 "stored_programs": n_files, "calls": n_rows,
                                 "safe_over_stored": f"{safe}/{n_files}", "safe_over_calls": f"{safe}/{n_rows}"})
    df = pd.DataFrame(rows)
    # The full SM-6 rule must reproduce the SM6-only row of Table 13 (sensitivity_rescoring).
    caps = sr.task_caps()
    for gname, gate in GATES:
        passing = [p for p in load_code(RUN2_E1, "baseline") if gate(p["code"])]
        viol = sum(1 for p in passing if sr.firings(p["code"], caps.get(p["task_id"]), wd(), {"SM-6"}, False) > 0)
        mine = df[(df.condition == "baseline") & (df.gate == gname) & (df.model == "pooled")
                  & (df.rule == "SM-6 (both clauses) = SM6-only")].iloc[0]
        assert mine.violating == viol and mine.gate_pass == len(passing), (gname, mine.violating, viol)
    return df


def safety_vs_baseline(single: pd.DataFrame, progs: pd.DataFrame) -> pd.DataFrame:
    """Payload-missing rate, baseline against safety. The second denominator keeps only the
    programs that contain a motion call SM-6 can see; on the others SM-6 cannot fire."""
    from scipy.stats import fisher_exact
    rows = []
    s = single[single.rule == "SM-6 set_payload only"]
    for gname, _ in GATES:
        for model in ["pooled"] + MODELS:
            b = s[(s.condition == "baseline") & (s.gate == gname) & (s.model == model)].iloc[0]
            f = s[(s.condition == "safety") & (s.gate == gname) & (s.model == model)].iloc[0]
            _, p = fisher_exact([[b.violating, b.gate_pass - b.violating], [f.violating, f.gate_pass - f.violating]])
            mc = {}
            for cond in ("baseline", "safety"):
                g = progs[(progs.condition == cond) & progs[f"gate_{gname}"] & progs.has_urscript_motion_call]
                if model != "pooled":
                    g = g[g.model == model]
                k = int(g.payload_missing.sum())
                lo, hi = wilson(k, len(g))
                mc[cond] = (k, len(g), lo, hi)
            rows.append({"gate": gname, "model": model,
                         "baseline_motion_call_programs": f"{mc['baseline'][0]}/{mc['baseline'][1]}",
                         "baseline_motion_call_wilson": f"[{100 * mc['baseline'][2]:.1f}, {100 * mc['baseline'][3]:.1f}]",
                         "safety_motion_call_programs": f"{mc['safety'][0]}/{mc['safety'][1]}",
                         "safety_motion_call_wilson": f"[{100 * mc['safety'][2]:.1f}, {100 * mc['safety'][3]:.1f}]",
                         "baseline": f"{b.violating}/{b.gate_pass}", "baseline_rate": b.viol_rate,
                         "baseline_wilson": f"[{100 * b.wilson_lo:.1f}, {100 * b.wilson_hi:.1f}]",
                         "safety": f"{f.violating}/{f.gate_pass}", "safety_rate": f.viol_rate,
                         "safety_wilson": f"[{100 * f.wilson_lo:.1f}, {100 * f.wilson_hi:.1f}]",
                         "difference_pp": 100 * (f.viol_rate - b.viol_rate),
                         "fisher_p_two_sided_descriptive": p})
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- F3

def audit_split() -> tuple[pd.DataFrame, pd.DataFrame]:
    from s04_a4 import load_sheets
    df = load_sheets()
    key = pd.read_csv(REPO / "docs/review/audit_v2/key.csv").set_index("item_id")
    s6 = df[df.rule == "SM-6"].copy()
    s6["description"] = key.loc[s6.index, "description"]
    s6["missing"] = s6.description.map(lambda d: "set_tcp" if "set_tcp()" in d else ("set_payload" if "set_payload()" in d else "?"))
    assert (s6.missing != "?").all()
    notes_b = pd.read_csv(REPO / "docs/review/audit_v2/sheet_rater_B.csv").set_index("item_id").note
    s6["note_B"] = notes_b.loc[s6.index]
    rows = []
    for part, g in list(s6.groupby("missing")) + [("SM-6 total", s6)]:
        rec = {"subpart": part, "n": len(g)}
        for who in ("A", "B", "adj"):
            tp = int((g[who] == "TP").sum())
            n = int(g[who].isin(["TP", "FP"]).sum())
            lo, hi = wilson(tp, n)
            rec.update({f"tp_{who}": tp, f"n_{who}": n, f"prec_{who}": tp / n if n else float("nan"),
                        f"wilson_{who}": f"[{100 * lo:.1f}, {100 * hi:.1f}]"})
        rows.append(rec)
    items = s6.reset_index()[["item_id", "missing", "source", "location", "A", "B", "adj", "note_B"]]
    return pd.DataFrame(rows), items


# ----------------------------------------------------------------------------- F4

def _between(text: str, start: str, end: str | None) -> str:
    i = re.search(start, text).start()
    j = re.search(end, text[i:]).start() + i if end else text.index("\n", i) if "\n" in text[i:] else len(text)
    return text[i:j].strip().replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")


def osf_quotes() -> None:
    def resp(sid):
        return json.loads(out("osf", f"schema_response_{sid}.json").read_text())["data"]["attributes"]

    a1 = resp(OSF_AMENDMENT_1)
    later = {sid: resp(sid) for sid in OSF_LATER}
    picks = [("344-58", "the conditions", r"Prompt Condition \(within-subject", r"Watchdog Feedback"),
             ("344-62", "the model variable", r"LLM Model Identifier:", r"\n\s*\n\s*\nPrompt Condition:"),
             ("344-62", "the condition variable", r"Prompt Condition:", r"\n\s*\nThe deterministic-pipeline"),
             ("344-66", "the cross-model index", r"Cross-Model Cochran's Q \(exploratory\)", r"\n\s*\nNo other composite"),
             ("344-71", "the analysis plan", r"--- Cross-Model Comparison", r"--- Sensitivity Analyses"),
             ("344-75", "the condition labels", r"Prompt Condition: Nominal", None),
             ("344-77", "the inference criteria", r"--- Cross-Model Comparison", r"--- Summary of Corrections \(LLM")]
    md = ["# OSF Amendment 1: the cross-model test and the condition labels, verbatim", "",
          f"Source: `out/osf/schema_response_{OSF_AMENDMENT_1}.json` (registration ve5m2, Amendment 1, "
          f"created {a1['date_created'][:10]}, state {a1['reviews_state']}), field `revision_responses`. "
          "HTML entities are unescaped; nothing else is changed. The last column of each heading says whether "
          "the same text is in the three later versions "
          f"({', '.join(later[s]['date_created'][:10] for s in OSF_LATER)}).", ""]
    for field, what, start, end in picks:
        q = _between(a1["revision_responses"][field], start, end)
        same = all(q in _between(later[s]["revision_responses"][field], start, end) for s in OSF_LATER)
        md += [f"## Field {field}, {what} (unchanged later: {'yes' if same else 'NO'})", "```", q, "```", ""]
    out("f1", "f4_osf_cross_model.md").write_text("\n".join(md) + "\n")


def _git_grep(pattern: str, *paths: str) -> list[str]:
    r = subprocess.run(["git", "grep", "-n", "-I", "-E", pattern, "--", *paths], cwd=REPO,
                       capture_output=True, text=True)
    return [l for l in r.stdout.splitlines() if not l.startswith("analysis/")]


def locations() -> pd.DataFrame:
    rows = []
    for l in _git_grep(r"adversarial_any|run_cross_model_cochran_q|condition_label=\"watchdog\"|== \"watchdog\"", "."):
        f, n, txt = l.split(":", 2)
        rows.append({"where": "repository, tracked", "file": f, "line": int(n), "text": txt.strip()[:160]})
    for l in _git_grep(r"[Cc]ochran", "paper", "docs"):
        f, n, txt = l.split(":", 2)
        if f.endswith(".tex") or f.endswith(".md") or f.endswith(".csv"):
            rows.append({"where": "repository, tracked (Cochran mentions)", "file": f, "line": int(n),
                         "text": txt.strip()[:160]})
    # Untracked or ignored outputs of mcnemar_analysis.py
    for f in sorted((REPO / "results").rglob("*")):
        if f.is_file() and f.suffix in (".csv", ".md") and f.stat().st_size < 5_000_000:
            txt = f.read_text(errors="replace")
            if "adversarial_any" in txt and ("cochran" in f.name.lower() or "hypothesis_report" in f.name):
                ignored = subprocess.run(["git", "check-ignore", "-q", rel(f)], cwd=REPO).returncode == 0
                for i, line in enumerate(txt.splitlines(), 1):
                    if re.search(r"adversarial_any|^watchdog|\|\s*watchdog", line):
                        rows.append({"where": "results/, gitignored" if ignored else "results/, untracked",
                                     "file": rel(f), "line": i, "text": line.strip()[:160]})
    # Supplementary zips: the uploaded 55984 copies are not on this machine (see report)
    for z in SUPP_ZIPS:
        with zipfile.ZipFile(z) as zf:
            hit = False
            for name in zf.namelist():
                if name.endswith("/"):
                    continue
                try:
                    txt = zf.read(name).decode("utf-8")
                except UnicodeDecodeError:
                    continue
                for i, line in enumerate(txt.splitlines(), 1):
                    if re.search(r"adversarial_any|[Cc]ochran|22\.93", line):
                        hit = True
                        rows.append({"where": f"zip {rel(z)}", "file": name, "line": i, "text": line.strip()[:160]})
            if not hit:
                rows.append({"where": f"zip {rel(z)} (sha256 {sha256(z)[:12]})", "file": f"{len(zf.namelist())} members",
                             "line": 0, "text": "no line matches adversarial_any, Cochran or 22.93"})
    return pd.DataFrame(rows)


def load_all(run: str) -> pd.DataFrame:
    import mcnemar_analysis as ma
    dirs = [RUN2_DIR] if run == "run2" else [RUN1_E1, REPO / "data/confirmatory/run1_2026-04/e2_confirmatory",
                                               REPO / "data/confirmatory/run1_2026-04/e3_confirmatory"]
    df = pd.concat([ma.load_results(d) for d in dirs], ignore_index=True)
    df["gate"] = (df.status == "success").astype(int)
    return df


def matrix(sub: pd.DataFrame, basis: str) -> pd.DataFrame:
    if basis == "gate-passing":
        sub = sub[sub.gate == 1]
    col = "gate" if basis == "gate-pass" else "has_violation"
    piv = sub.groupby(["task_id", "model"])[col].max().unstack("model")
    return piv.dropna(how="any")


CONDS = {"baseline": lambda d: d.condition == "baseline", "safety": lambda d: d.condition == "safety",
         "adversarial_any": lambda d: d.condition.str.startswith("adversarial_", na=False),
         "watchdog": lambda d: d.condition == "watchdog"}


def cochran_rows(df: pd.DataFrame, conds: dict, run: str, holm_family: str) -> list[dict]:
    rows = []
    for basis in ("naive", "gate-passing", "gate-pass"):
        block = []
        for cname, sel in conds.items():
            mat = matrix(df[sel(df)], basis)
            res = cochran_exact(mat.to_numpy(dtype=int))
            rec = {"run": run, "basis": basis, "condition": cname, "tasks_in_test": len(mat),
                   "n_discordant": res["n_discordant"], "Q": res["Q"], "p_asymptotic": res["p_asymptotic"],
                   "p_exact": res["p_exact"],
                   "Q_note": "" if res["n_discordant"] else "undefined (no discordant blocks)"}
            for m in sorted(mat.columns):
                rec[f"rate_{m}"] = float(mat[m].mean()) if len(mat) else float("nan")
            block.append(rec)
        for r, a, e in zip(block, holm([r["p_asymptotic"] for r in block]), holm([r["p_exact"] for r in block])):
            r[f"p_asymptotic_holm_{holm_family}"] = a
            r[f"p_exact_holm_{holm_family}"] = e
        rows += block
    return rows


def naive_vs_gatepass(df: pd.DataFrame, run: str) -> list[dict]:
    """How many (task, model) cells have the same value on the naive and the gate-pass basis."""
    rows = []
    for cname, sel in CONDS.items():
        a, b = matrix(df[sel(df)], "naive"), matrix(df[sel(df)], "gate-pass")
        a, b = a.align(b, join="inner")
        rows.append({"run": run, "condition": cname, "cells": int(a.size), "cells_equal": int((a == b).to_numpy().sum()),
                     "naive_1_gatepass_0": int(((a == 1) & (b == 0)).to_numpy().sum()),
                     "naive_0_gatepass_1": int(((a == 0) & (b == 1)).to_numpy().sum())})
    return rows


def gate_equivalence_e2_e3() -> pd.DataFrame:
    rows = []
    for exp, csv, cdir in (("E2", "E2_full/e2_results.csv", "E2_full/code"), ("E3", "E3_full/e3_results.csv", "E3_full/code")):
        d = pd.read_csv(RUN2_DIR / csv)
        files = {f.name: f for f in (RUN2_DIR / cdir).glob("*.urscript")}
        mism = no_file = 0
        for _, r in d.iterrows():
            tok = r.model.replace(":", "_")
            cond = r.condition
            stem = f"{r.task_id}_{tok}_{cond}_rep{r.rep}" + (f"_retry{r.retry}" if exp == "E3" and r.retry > 0 else "")
            f = files.get(stem + ".urscript") or files.get(stem + ".invalid.urscript")
            if f is None:
                no_file += 1
                continue
            lex = sr.gate_lexical(f.read_text(errors="replace"))
            mism += int(lex != (r.status == "success")) + int(f.name.endswith(".invalid.urscript") == (r.status == "success"))
        rows.append({"experiment": exp, "rows": len(d), "rows_without_file": no_file,
                     "status_vs_lexical_gate_or_suffix_mismatches": mism})
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- F5

def table16_all_bases() -> pd.DataFrame:
    a = pd.read_csv(out("a2", "cochran_all.csv"))
    a = a[a.run == "run2"]
    rows = []
    for (variant, cond), g in a.groupby(["variant", "condition"], sort=False):
        rec = {"variant": variant, "condition": cond}
        for basis in ("naive", "gate-passing", "gate-pass"):
            r = g[g.basis == basis].iloc[0]
            rec[f"{basis}: tasks"] = int(r.tasks_in_test)
            rec[f"{basis}: discordant"] = int(r.n_discordant)
            rec[f"{basis}: Q"] = r.Q
            rec[f"{basis}: p exact"] = r.p_exact
        rows.append(rec)
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- F6, F7

def structural_reasons(code: str) -> str:
    from enfield_llm.code_parser import URSCRIPT_CALL_PATTERN
    r = []
    if not sr.gate_lexical(code):
        r.append("no URScript call")
    m = sr.NON_URSCRIPT.search(code)
    if m:
        r.append(f"non-URScript motion `{m.group(0)}`")
    if not sr.DEF_BODY.search(code):
        r.append("no def...end body")
    calls = sorted({m.group(0).split("(")[0].strip().lower() for m in URSCRIPT_CALL_PATTERN.finditer(code)})
    if len(calls) < 2:
        r.append(f"{len(calls)} distinct URScript call ({', '.join(calls)})")
    return "; ".join(r) if r else "passes"


def _numbered(path, start: int, end: int, mark: tuple[int, int] | None = None) -> list[str]:
    lines = path.read_text().splitlines()
    res = []
    for i in range(start, end + 1):
        flag = ">" if mark and mark[0] <= i <= mark[1] else " "
        res.append(f"{flag}{i:5d}: {lines[i - 1]}")
    return res


def _find(path, pattern: str) -> int:
    for i, l in enumerate(path.read_text().splitlines(), 1):
        if re.search(pattern, l):
            return i
    raise ValueError(pattern)


def t009_md() -> None:
    from s04_a4 import gate_evidence
    f = RUN2_E1 / "code" / T009
    code = f.read_text(errors="replace")
    e1 = load_e1_csv(RUN2_E1)
    row = e1[(e1.task_id == "T009") & (e1.model == "deepseek-coder-v2:16b") & (e1.condition == "baseline") & (e1.rep == 1)].iloc[0]
    ev = gate_evidence(code)
    cp = REPO / "enfield_llm/enfield_llm/code_parser.py"
    srp = REPO / "scripts/review/sensitivity_rescoring.py"
    pat_start = _find(cp, r"^# Pattern for URScript function-call syntax")
    pat_end = _find(cp, r"^\s*re\.IGNORECASE,") + 1
    hm_start = _find(cp, r"def _build_result")
    hm_end = _find(cp, r"is_valid_urscript=bool")
    t_start = _find(PAPER, r"This single safe output is genuine code")
    md = [f"# {T009}", "",
          f"File: `{rel(f)}`, sha256 {sha256(f)}, {len(code.splitlines())} lines.", "",
          "## Program, verbatim (line numbers added)", "```"]
    md += [f"{i:3d}: {l}" for i, l in enumerate(code.splitlines(), 1)]
    md += ["```", "", "## Row in e1_results.csv", "",
           md_table(pd.DataFrame([row[["task_id", "model", "condition", "rep", "status", "code_lines", "has_motion",
                                        "has_safety_check", "dm_violations", "sm_violations", "total_violations"]]])), "",
           "## Gates", "",
           f"- Lexical gate (`URSCRIPT_CALL_PATTERN.search`): {sr.gate_lexical(code)}. Matches: {ev['gate_calls']}; "
           f"matches outside a comment: {ev['gate_calls_outside_comments']}.",
           f"- Structural gate (`sensitivity_rescoring.gate_structural`): {sr.gate_structural(code)}. "
           f"Failing conditions: {structural_reasons(code)}.",
           f"- SM-6 motion search (`_FIRST_MOTION_RE`, call syntax): {'found' if _FIRST_MOTION_RE.search(code) else 'no match'}.",
           f"- Substring has_motion (code_parser): {any(w in code.lower() for w in ['movej', 'movel', 'movec', 'speedl', 'speedj', 'servoj', 'servoc'])}"
           f" (matched words: {sorted({w for w in ['movej', 'movel', 'movec', 'speedl', 'speedj', 'servoj', 'servoc'] if w in code.lower()})}).",
           "", f"## Gate and has_motion code, `{rel(cp)}` at git HEAD", "```"]
    md += _numbered(cp, pat_start, pat_end)
    md += ["```", "```"]
    md += _numbered(cp, hm_start, hm_end)
    md += ["```", "", f"`{rel(srp)}`:", "```"]
    md += _numbered(srp, _find(srp, r"^NON_URSCRIPT"), _find(srp, r"^DEF_BODY"))
    md += _numbered(srp, _find(srp, r"^def gate_lexical"), _find(srp, r"return bool\(DEF_BODY"))
    md += ["```", "", f"## `{rel(PAPER)}` lines {t_start}-{t_start + 2} (marked >), with two lines of context", "```"]
    md += _numbered(PAPER, t_start - 2, t_start + 3, (t_start, t_start + 2))
    md += ["```"]
    out("f1", "f6_t009.md").write_text("\n".join(md) + "\n")


def sm6_only_safe(progs: pd.DataFrame) -> pd.DataFrame:
    from s04_a4 import gate_evidence
    e1 = load_e1_csv(RUN2_E1).set_index(["task_id", "model_disp", "condition", "rep"])
    g = progs[(progs.condition == "baseline") & progs.gate_lexical & ~progs.tcp_missing & ~progs.payload_missing]
    rows = []
    for _, r in g.iterrows():
        f = RUN2_E1 / "code" / r.program
        code = f.read_text(errors="replace")
        motion_lines = [l.strip() for l in code.splitlines() if ANY_MOTION_WORD.search(l)]
        urs = bool(r.has_urscript_motion_call)
        kind = ("URScript" if urs and r.gate_structural else
                "pseudo-code" if not urs else "mixed")
        rows.append({"task_id": r.task_id, "model": r.model, "rep": r.rep, "program": r.program,
                     "status": e1.loc[(r.task_id, r.model, "baseline", r.rep)].status,
                     "urscript_motion_call": urs, "kind": kind, "gate_structural": bool(r.gate_structural),
                     "structural_gate_detail": structural_reasons(code),
                     "lexical_gate_calls": gate_evidence(code)["gate_calls"],
                     "set_tcp_before_motion": urs and not r.tcp_missing,
                     "first_motion_line": motion_lines[0][:90] if motion_lines else "(none)",
                     "sha256": sha256(f)})
    df = pd.DataFrame(rows)
    assert len(df) == 9, len(df)
    return df


# ----------------------------------------------------------------------------- main

def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)

    progs = program_table()
    progs.to_csv(out("f1", "f1_programs.csv"), index=False)
    gr = groups(progs)
    gr.to_csv(out("f1", "f1_sm6_groups.csv"), index=False)
    single = single_rules(progs)
    single.to_csv(out("f1", "f1_single_rule.csv"), index=False)
    svb = safety_vs_baseline(single, progs)
    svb.to_csv(out("f1", "f2_safety_vs_baseline.csv"), index=False)
    print(gr.to_string(index=False))
    print(single.round(4).to_string(index=False))
    print(svb.round(4).to_string(index=False))

    split, items = audit_split()
    split.to_csv(out("f1", "f3_sm6_audit_split.csv"), index=False)
    items.to_csv(out("f1", "f3_sm6_audit_items.csv"), index=False)
    print(split.to_string(index=False))
    print(items.to_string(index=False))

    osf_quotes()
    loc = locations()
    loc.to_csv(out("f1", "f4_locations.csv"), index=False)
    out("f1", "f4_locations.md").write_text(md_table(loc) + "\n")
    print(loc.to_string(index=False))
    rows, cells = [], []
    for run in ("run2", "run1"):
        df = load_all(run)
        rows += cochran_rows(df, CONDS, run, "4")
        cells += naive_vs_gatepass(df, run)
    four = pd.DataFrame(rows)
    pd.DataFrame(cells).to_csv(out("f1", "f4_naive_vs_gatepass_cells.csv"), index=False)
    print(pd.DataFrame(cells).to_string(index=False))
    four.to_csv(out("f1", "f4_cochran_four_conditions.csv"), index=False)
    print(four.round(5).to_string(index=False))
    # Naive basis must reproduce scripts/mcnemar_analysis.py (out/a2/vid_four_condition_family.csv)
    fam = pd.read_csv(out("a2", "vid_four_condition_family.csv"))
    for _, r in fam.iterrows():
        m = four[(four.run == r.run) & (four.basis == "naive") & (four.condition == r.condition)].iloc[0]
        assert abs(m.Q - r.Q) < 1e-9 and abs(m.p_asymptotic - r.p) < 1e-12 and abs(m.p_asymptotic_holm_4 - r.p_holm_4family) < 1e-12
    sub_rows = []
    for run in ("run2", "run1"):
        df = load_all(run)
        subs = sorted(c for c in df.condition.dropna().unique() if c.startswith("adversarial_"))
        sub_rows += cochran_rows(df, {c: (lambda d, c=c: d.condition == c) for c in subs}, run, str(len(subs)))
    subt = pd.DataFrame(sub_rows)
    subt.to_csv(out("f1", "f4_cochran_adversarial_subtypes.csv"), index=False)
    print(subt.round(4).to_string(index=False))
    ge = gate_equivalence_e2_e3()
    ge.to_csv(out("f1", "f4_gate_equivalence_e2_e3.csv"), index=False)
    print(ge.to_string(index=False))

    t16 = table16_all_bases()
    t16.to_csv(out("f1", "f5_table16_all_bases.csv"), index=False)
    print(t16.round(4).to_string(index=False))

    t009_md()
    s7 = sm6_only_safe(progs)
    s7.to_csv(out("f1", "f7_sm6_only_safe.csv"), index=False)
    print(s7.drop(columns=["sha256"]).to_string(index=False))


if __name__ == "__main__":
    main()
