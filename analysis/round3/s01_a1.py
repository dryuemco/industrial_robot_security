#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A1: the reference translation as a column of Table 13, the SM-6 split, and what the
model and the translator are given about TCP and payload.

Outputs under out/a1/:
  translator/{minimal,preamble}/*.urscript, translator_hashes.csv
  translator_check.csv          regenerated per-rule counts vs paper/tables/comparator_programs.csv
  table13_llm_check.csv         Table 13 LLM rows re-run vs paper/tables/sensitivity_e1_baseline.csv
  table13_with_translator.csv   every Table 13 row for LLM, Translator, Translator + preamble
  sm6_split.csv                 SM-6 firings by missing set_tcp / set_payload
  ir_tcp_payload.csv            per task: IR fields and the preamble the translator writes
  prompt_fields.csv             IR field -> read by the prompt builder? rendered? sent (log)?
  prompt_quotes.md              source lines mentioning payload / TCP in the prompts
  sm6_clause_map.md             SM-6 clause and CWE: code vs Table 18 vs traceability CSV
"""
from __future__ import annotations

import json
import re

import pandas as pd

from common import (PAPER, REPO, RUN2_E1, TABLES, TASKS_DIR, cu, get_flag, load_code, load_tasks, md_table,
                    out, rel, ruleset_firings, set_flag, sha256_text, sr, task_caps, violations, wd)

SETS = {"minimal": False, "preamble": True}
SET_NAME = {"minimal": "Translator", "preamble": "Translator + preamble"}


# ----------------------------------------------------------------------------- 1

def translator_programs() -> dict[str, list[tuple[str, str, str]]]:
    progs = {k: cu.translator_programs(v) for k, v in SETS.items()}
    rows = []
    for k, lst in progs.items():
        for tid, stem, code in lst:
            p = out("a1", "translator", k, f"{stem}.urscript")
            p.write_text(code)
            rows.append({"variant": SET_NAME[k], "task_id": tid, "program": stem,
                         "file": rel(p), "sha256": sha256_text(code)})
    pd.DataFrame(rows).to_csv(out("a1", "translator_hashes.csv"), index=False)
    return progs


def check_translator(progs) -> bool:
    ref = pd.read_csv(TABLES / "comparator_programs.csv")
    caps = cu.load_task_caps()
    cols = [c for c in ref.columns if c not in ("set", "task_id", "program")]
    rows = []
    ok = True
    for k, lst in progs.items():
        for tid, stem, code in lst:
            mine = cu.score(code, caps.get(tid), wd())
            r = ref[(ref.set == SET_NAME[k]) & (ref.program == stem)]
            if len(r) != 1:
                ok = False
                rows.append({"variant": SET_NAME[k], "program": stem, "column": "(row)", "paper_csv": "MISSING", "regenerated": ""})
                continue
            r = r.iloc[0]
            for c in cols:
                same = int(r[c]) == int(mine[c])
                ok &= same
                if not same:
                    rows.append({"variant": SET_NAME[k], "program": stem, "column": c,
                                 "paper_csv": int(r[c]), "regenerated": int(mine[c])})
    df = pd.DataFrame(rows, columns=["variant", "program", "column", "paper_csv", "regenerated"])
    df.to_csv(out("a1", "translator_check.csv"), index=False)
    print(f"translator per-rule counts identical to comparator_programs.csv: {ok} ({len(df)} differing cells)")
    return ok


def table13_rows(programs: list[tuple[str, str]], n_all: int, label: str) -> list[dict]:
    """programs: (task_id, code). Same computation as scripts/review/sensitivity_rescoring.py."""
    caps = task_caps()
    rows = []
    for gate_name, gate in (("lexical", sr.gate_lexical), ("structural", sr.gate_structural)):
        passing = [(t, c) for t, c in programs if gate(c)]
        for rs in sr.RULESETS:
            counts = [ruleset_firings(c, caps.get(t), rs) for t, c in passing]
            n_pass = len(passing)
            n_viol = sum(1 for x in counts if x > 0)
            rows.append({"set": label, "gate": gate_name, "rules": rs, "n_all": n_all, "gate_pass": n_pass,
                         "viol_pct": 100 * n_viol / n_pass if n_pass else float("nan"),
                         "safe": n_pass - n_viol, "safe_str": f"{n_pass - n_viol}/{n_all}",
                         "mean": sum(counts) / n_pass if n_pass else float("nan"),
                         "total_firings": sum(counts)})
    return rows


def a1_table13(progs) -> pd.DataFrame:
    llm = load_code(RUN2_E1, "baseline")
    llm_rows = table13_rows([(p["task_id"], p["code"]) for p in llm], len(llm), "LLM baseline (E1, run 2)")
    # Check the LLM rows against the paper's CSV.
    ref = pd.read_csv(TABLES / "sensitivity_e1_baseline.csv")
    chk = []
    for r in llm_rows:
        x = ref[(ref.gate == r["gate"]) & (ref.rules == r["rules"])].iloc[0]
        chk.append({"gate": r["gate"], "rules": r["rules"],
                    "paper_pass": int(x.gate_pass), "mine_pass": r["gate_pass"],
                    "paper_viol_pct": round(100 * x.viol_rate_gate, 1), "mine_viol_pct": round(r["viol_pct"], 1),
                    "paper_safe": int(x.substantive_safe), "mine_safe": r["safe"],
                    "paper_n_all": int(x.n_all), "mine_n_all": r["n_all"],
                    "paper_mean": round(x.mean_firings_gate, 1), "mine_mean": round(r["mean"], 1),
                    "paper_mean_raw": x.mean_firings_gate, "mine_mean_raw": r["mean"]})
    chk = pd.DataFrame(chk)
    chk["match"] = ((chk.paper_pass == chk.mine_pass) & (chk.paper_viol_pct == chk.mine_viol_pct)
                    & (chk.paper_safe == chk.mine_safe) & (chk.paper_n_all == chk.mine_n_all)
                    & ((chk.paper_mean_raw - chk.mine_mean_raw).abs() < 1e-9))
    chk.to_csv(out("a1", "table13_llm_check.csv"), index=False)
    set_flag("table13_llm_reproduced", bool(chk.match.all()))
    print(f"Table 13 LLM rows reproduced: {bool(chk.match.all())}")

    rows = list(llm_rows)
    for k, lst in progs.items():
        rows += table13_rows([(t, c) for t, _, c in lst], len(lst), SET_NAME[k])
    df = pd.DataFrame(rows)
    df.to_csv(out("a1", "table13_with_translator.csv"), index=False)
    # Wide view in the shape of Table 13 with the comparator as further columns.
    wide = df.pivot_table(index=["gate", "rules"], columns="set",
                          values=["gate_pass", "viol_pct", "safe_str", "mean"], aggfunc="first", sort=False)
    wide.columns = [f"{s} | {v}" for v, s in wide.columns]
    order = [f"{s} | {v}" for s in ["LLM baseline (E1, run 2)", "Translator", "Translator + preamble"]
             for v in ["gate_pass", "viol_pct", "safe_str", "mean"]]
    wide = wide[order].reset_index()
    wide.to_csv(out("a1", "table13_with_translator_wide.csv"), index=False)
    return df


# ----------------------------------------------------------------------------- 2

def sm6_split(progs) -> pd.DataFrame:
    llm = [p for p in load_code(RUN2_E1, "baseline") if sr.gate_lexical(p["code"])]
    groups = [("LLM baseline (E1, run 2, lexical gate)", "pooled", [p["code"] for p in llm])]
    for m in ["Qwen2.5-Coder-32B", "DeepSeek-Coder-V2-16B", "CodeLlama-34B"]:
        groups.append(("LLM baseline (E1, run 2, lexical gate)", m, [p["code"] for p in llm if p["model_disp"] == m]))
    groups.append(("LLM baseline (E1, run 2, structural gate)", "pooled",
                   [p["code"] for p in llm if sr.gate_structural(p["code"])]))
    for k, lst in progs.items():
        groups.append((SET_NAME[k], "-", [c for _, _, c in lst]))
    rows = []
    for name, model, codes in groups:
        tcp = pay = both = either = 0
        f_tcp = f_pay = 0
        any_tcp = any_pay = no_motion = tcp_before = pay_before = 0
        first_motion = re.compile(r"\b(movej|movel|movec|servoj|speedl|speedj)\s*\(", re.I)
        for code in codes:
            lines = code.split("\n")
            fm = next((i for i, l in enumerate(lines) if first_motion.search(l)), None)
            if fm is None:
                no_motion += 1
            else:
                head = "\n".join(lines[:fm])
                tcp_before += bool(re.search(r"\bset_tcp\s*\(", head, re.I))
                pay_before += bool(re.search(r"\bset_payload\s*\(", head, re.I))
            vs = [v for v in violations(code) if v.detection_mechanism == "SM-6"]
            t = sum(1 for v in vs if v.metadata.get("missing") == "set_tcp")
            pl = sum(1 for v in vs if v.metadata.get("missing") == "set_payload")
            f_tcp += t
            f_pay += pl
            tcp += t > 0
            pay += pl > 0
            both += (t > 0 and pl > 0)
            either += (t > 0 or pl > 0)
            any_tcp += bool(re.search(r"\bset_tcp\s*\(", code, re.I))
            any_pay += bool(re.search(r"\bset_payload\s*\(", code, re.I))
        n = len(codes)
        rows.append({"set": name, "model": model, "n_programs": n,
                     "programs_missing_set_tcp": tcp, "programs_missing_set_payload": pay,
                     "programs_missing_both": both, "programs_with_any_SM6": either,
                     "firings_set_tcp": f_tcp, "firings_set_payload": f_pay,
                     "mean_set_tcp_per_program": f_tcp / n if n else float("nan"),
                     "mean_set_payload_per_program": f_pay / n if n else float("nan"),
                     "mean_SM6_per_program": (f_tcp + f_pay) / n if n else float("nan"),
                     "programs_containing_set_tcp_anywhere": any_tcp,
                     "programs_containing_set_payload_anywhere": any_pay,
                     "programs_without_motion_command": no_motion,
                     "programs_set_tcp_before_first_motion": tcp_before,
                     "programs_set_payload_before_first_motion": pay_before})
    df = pd.DataFrame(rows)
    df.to_csv(out("a1", "sm6_split.csv"), index=False)
    return df


# ----------------------------------------------------------------------------- 3

def ir_table(progs) -> pd.DataFrame:
    tasks = load_tasks()
    pre = {tid: code for tid, _, code in progs["preamble"]}
    rows = []
    for tid, t in tasks.items():
        tool = t.get("tool", {}) or {}
        tcp = (tool.get("tcp_offset") or {})
        pos = tcp.get("position") or {}
        ori = tcp.get("orientation") or {}
        tp = (t.get("safety_requirements", {}) or {}).get("tool_payload", {}) or {}
        mass_keys = sorted({k for k in _leaf_paths(t) if re.search(r"mass|weight", k, re.I)})
        lines = [ln.strip() for ln in pre[tid].splitlines() if re.search(r"set_tcp|set_payload", ln)]
        rows.append({"task_id": tid, "tool_type": tool.get("type"), "tool_name": tool.get("name"),
                     "robot.payload_kg": (t.get("robot") or {}).get("payload_kg"),
                     "tool_payload.mass_true_kg": tp.get("mass_true_kg"),
                     "tool_payload.mass_min_kg": tp.get("mass_min_kg"),
                     "tool_payload.mass_max_kg": tp.get("mass_max_kg"),
                     "tool_payload.cog_true_mm": tp.get("cog_true_mm"),
                     "tcp_offset.position_mm": [pos.get("x"), pos.get("y"), pos.get("z")],
                     "tcp_offset.orientation_q": [ori.get("qx"), ori.get("qy"), ori.get("qz"), ori.get("qw")],
                     "mass_like_keys": ";".join(mass_keys),
                     "tool_mass_field_present": any(k.startswith("tool.") for k in mass_keys),
                     "preamble_set_tcp": next((l for l in lines if "set_tcp" in l), ""),
                     "preamble_set_payload": next((l for l in lines if "set_payload" in l), "")})
    df = pd.DataFrame(rows)
    df.to_csv(out("a1", "ir_tcp_payload.csv"), index=False)
    return df


def _leaf_paths(o, p=""):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _leaf_paths(v, f"{p}.{k}" if p else k)
    elif isinstance(o, list):
        if not o or not isinstance(o[0], (dict, list)):
            yield p
        for x in o:
            if isinstance(x, (dict, list)):
                yield from _leaf_paths(x, f"{p}[]")
    else:
        yield p


class _Rec(dict):
    """dict that records which keys the prompt builder asks for (path, present?)."""

    def __init__(self, data, path, log):
        super().__init__({k: _wrap(v, f"{path}.{k}" if path else k, log) for k, v in data.items()})
        self._path, self._log = path, log

    def _note(self, key):
        self._log.add((f"{self._path}.{key}" if self._path else key, key in self))

    def get(self, key, default=None):
        self._note(key)
        return super().get(key, default)

    def __getitem__(self, key):
        self._note(key)
        return super().__getitem__(key)


def _wrap(v, path, log):
    if isinstance(v, dict):
        return _Rec(v, path, log)
    if isinstance(v, list):
        return [_wrap(x, f"{path}[]", log) for x in v]
    return v


def prompt_fields() -> pd.DataFrame:
    from enfield_llm.prompt_builder import PromptBuilder, PromptMode
    builder = PromptBuilder()
    tasks = load_tasks()
    # What was actually sent: the run-2 request log.
    sent = {}
    log_lines = (RUN2_E1 / "logs" / "ollama_log.jsonl").read_text().splitlines()
    for line in log_lines:
        r = json.loads(line)
        m = re.search(r"\((T\d{3})\)", r["user_prompt"])
        sent.setdefault(m.group(1), set()).add((r["system_prompt"], r["user_prompt"]))
    read: set = set()
    rendered_equals_sent = {}
    for tid, t in tasks.items():
        log: set = set()
        rec = _Rec(t, "", log)
        s_b, u_b = builder.build(rec, PromptMode.BASELINE)
        s_s, u_s = builder.build(_Rec(t, "", set()), PromptMode.SAFETY)
        read |= log
        rendered_equals_sent[tid] = ({(s_b, u_b), (s_s, u_s)} == sent.get(tid, set()))
    set_flag("prompt_builder_reproduces_logged_prompts", all(rendered_equals_sent.values()))

    leaves = sorted({p for t in tasks.values() for p in _leaf_paths(t)})
    asked = {p for p, _ in read}
    present_asked = {p for p, present in read if present}
    sent_text = "\n".join(u for v in sent.values() for _, u in v)
    rows = []
    for leaf in leaves:
        n_tasks = sum(1 for t in tasks.values() if leaf in set(_leaf_paths(t)))
        rows.append({"ir_field": leaf, "tasks_with_field": n_tasks,
                     "read_by_prompt_builder": leaf in present_asked,
                     "used_by_translator_preamble": leaf in {"robot.payload_kg", "tool.tcp_offset.position.x",
                                                             "tool.tcp_offset.position.y", "tool.tcp_offset.position.z"}})
    for p in sorted(asked - present_asked):
        rows.append({"ir_field": p + "  (key asked for, absent in IR)", "tasks_with_field": 0,
                     "read_by_prompt_builder": False, "used_by_translator_preamble": False})
    df = pd.DataFrame(rows)
    df.to_csv(out("a1", "prompt_fields.csv"), index=False)
    # Evidence from the log itself.
    ev = {"log_file": rel(RUN2_E1 / "logs" / "ollama_log.jsonl"), "logged_requests": len(log_lines),
          "distinct_system_user_prompt_pairs": sum(len(v) for v in sent.values()),
          "rendered_prompts_equal_logged_for_all_tasks": all(rendered_equals_sent.values()),
          "user_prompt_lines_with_payload": sorted({l for l in sent_text.splitlines() if re.search("payload", l, re.I)})[:20],
          "user_prompt_lines_with_tcp": sorted({l for l in sent_text.splitlines() if re.search("tcp", l, re.I)}),
          "user_prompts_mentioning_cog_or_centre_of_gravity": int(bool(re.search(r"\bcog\b|centre of gravity|center of gravity", sent_text, re.I))),
          "user_prompts_mentioning_set_tcp_or_tool_offset": int(bool(re.search(r"set_tcp|tcp offset|tcp_offset|tool offset", sent_text, re.I)))}
    out("a1", "prompt_log_evidence.json").write_text(json.dumps(ev, indent=2))
    return df


# ----------------------------------------------------------------------------- 4

def prompt_quotes() -> None:
    src = REPO / "enfield_llm/enfield_llm/prompt_builder.py"
    lines = src.read_text().splitlines()

    def span(start_pat, end_pat):
        s = next(i for i, l in enumerate(lines) if re.search(start_pat, l))
        e = next(i for i in range(s + 1, len(lines)) if re.search(end_pat, lines[i]))
        return s, e

    parts = [("SYSTEM_PROMPT_BASELINE (baseline system prompt)", span(r"^SYSTEM_PROMPT_BASELINE", r'^"""')),
             ("SYSTEM_PROMPT_SAFETY (safety-prompted system prompt)", span(r"^SYSTEM_PROMPT_SAFETY", r'^"""')),
             ("_build_user_prompt (user prompt, identical in both conditions)",
              span(r"def _build_user_prompt", r"return \"\\n\"\.join\(parts\)"))]
    md = [f"Source: `{rel(src)}` (git HEAD). Lines matching /payload|tcp/i, verbatim, with line numbers.", ""]
    for title, (s, e) in parts:
        md.append(f"### {title}, lines {s + 1}-{e + 1}")
        hits = [(i + 1, lines[i]) for i in range(s, e + 1) if re.search(r"payload|tcp", lines[i], re.I)]
        if not hits:
            md.append("(no line mentions payload or TCP)")
        md.append("```")
        md += [f"{n}: {l}" for n, l in hits]
        md.append("```")
        md.append("")
    out("a1", "prompt_quotes.md").write_text("\n".join(md))


# ----------------------------------------------------------------------------- 5

def clause_map() -> None:
    md = []
    code = REPO / "enfield_watchdog_static/enfield_watchdog_static/rules"
    md.append("### Code (enfield_watchdog_static/rules/sm*.py): iso_clause and cwe per rule")
    md.append("| rule file | line | text |\n|---|---|---|")
    for f in sorted(code.glob("sm*.py")):
        for i, l in enumerate(f.read_text().splitlines(), 1):
            if re.search(r'iso_clause=|"cwe":|"missing":', l):
                md.append(f"| {f.name} | {i} | `{l.strip()}` |")
    tex = PAPER.read_text().splitlines()
    md.append("\n### Table 18 (paper/main_access.tex, tab:iso)")
    md.append("| line | text |\n|---|---|")
    s = next(i for i, l in enumerate(tex) if "label{tab:iso}" in l)
    for i in range(s, s + 40):
        if re.match(r"\s*(SM|DM)-\d", tex[i]):
            md.append(f"| {i + 1} | `{tex[i].strip()}` |")
        if "end{tabular}" in tex[i]:
            break
    trace = REPO / "docs/iso_10218_traceability.csv"
    md.append(f"\n### {rel(trace)}")
    md.append("| line | text |\n|---|---|")
    for i, l in enumerate(trace.read_text().splitlines(), 1):
        if i == 1 or re.match(r"(SM|DM)-", l):
            md.append(f"| {i} | `{l}` |")
    # Rules on 5.1.16 in each source.
    md.append("\n### Rules mapped to 5.1.16 in each source")
    code_5116 = sorted({f.name for f in code.glob("sm*.py") if 'iso_clause="5.1.16"' in f.read_text()})
    tab_5116 = sorted({l.split("&")[0].strip() for l in tex[s:s + 40] if "5.1.16" in l})
    tr_5116 = sorted({l.split(",")[0] for l in trace.read_text().splitlines()[1:] if ",5.1.16," in l})
    md.append(f"- code: {', '.join(code_5116)}")
    md.append(f"- Table 18: {', '.join(tab_5116)}")
    md.append(f"- traceability CSV: {', '.join(tr_5116)}")
    sm6 = {"code": sorted(set(re.findall(r'iso_clause="([^"]+)"', (code / 'sm6_missing_preamble.py').read_text()))),
           "table18": sorted({l.split("&")[2].strip(" \\") for l in tex[s:s + 40] if l.strip().startswith("SM-6")}),
           "traceability": sorted({l.split(",")[4] for l in trace.read_text().splitlines()[1:] if l.startswith("SM-6")})}
    md.append(f"\nSM-6 clauses: code {sm6['code']}, Table 18 {sm6['table18']}, traceability {sm6['traceability']}; "
              f"agree: {sm6['code'] == sm6['table18'] == sm6['traceability']}")
    # The test files the traceability CSV names, and whether they exist.
    md.append("\n### Test files named in the traceability CSV for SM rules")
    for l in trace.read_text().splitlines()[1:]:
        if l.startswith("SM-"):
            tf = l.split(",")[-1]
            md.append(f"- {l.split(',')[0]}: `{tf}` exists: {(REPO / tf).is_file() or (REPO / 'enfield_watchdog_static' / tf).is_file()}")
    out("a1", "sm6_clause_map.md").write_text("\n".join(md))


def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    progs = translator_programs()
    ok = check_translator(progs)
    set_flag("translator_matches_paper_csv", ok)
    ir_table(progs)
    prompt_fields()
    prompt_quotes()
    clause_map()
    if not ok:
        print("STOP A1.1/A1.2 (and A3): regenerated translator counts differ from comparator_programs.csv")
        return
    a1_table13(progs)
    print(sm6_split(progs).to_string(index=False))


if __name__ == "__main__":
    main()
