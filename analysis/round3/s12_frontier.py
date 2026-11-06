#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""SM-6 split for the frontier run behind Table 7 ("49.2% under SM-6 only"), and the
frontier prompts checked for a tool-centre-point offset.

Input: data/exploratory/E1_frontier/ (code/, e1_results.csv, logs/idun_log.jsonl), copied from
results/E1_frontier/ by s13_import_runs.py; no file of this run needed masking.

Outputs under out/f2/:
  frontier_programs.csv     one row per stored frontier program: model, condition, gates,
                            SM-6 split, URScript motion call present, set_tcp/set_payload
                            anywhere and before the first motion call
  frontier_sm6_groups.csv   the four SM-6 groups per condition and model (lexical gate),
                            and the SM6-only rate reproduced against paper/tables/sensitivity_e1_frontier.csv
  frontier_prompts.json     logged requests, distinct prompts, and whether any prompt mentions
                            a TCP offset, set_tcp, a tool offset or the payload
"""
from __future__ import annotations

import json
import re

import pandas as pd

from common import REPO, TABLES, get_flag, load_tasks, out, sr, violations

from enfield_watchdog_static.rules.sm6_missing_preamble import _FIRST_MOTION_RE  # noqa: E402

FRONTIER = REPO / "data" / "exploratory" / "E1_frontier"  # copied from results/ by s13_import_runs.py
NAME = re.compile(r"^(T\d{3})_(.+)_(baseline|safety)_rep(\d)(\.invalid)?\.urscript$")
SET_TCP = re.compile(r"\bset_tcp\s*\(", re.I)
SET_PAYLOAD = re.compile(r"\bset_payload\s*\(", re.I)
TCP_POSE = re.compile(r"\bset_tcp\s*\(\s*p\[\s*([^\]]*)\]", re.I)
PROMPT_PATTERNS = {"tcp offset": r"tcp[ _-]?offset", "set_tcp": r"set_tcp", "tool offset": r"tool[ _-]?offset",
                   "tool centre/center point": r"tool[ -]cent(?:re|er)[ -]point", "TCP (any use)": r"\btcp\b",
                   "payload": r"payload", "set_payload": r"set_payload"}


def tcp_literal(code: str, z_ir: float) -> tuple[str, bool]:
    """The z component of the first set_tcp(p[...]) pose, and whether it equals the IR offset."""
    m = TCP_POSE.search(code)
    if not m:
        return "", False
    parts = [x.strip() for x in m.group(1).split(",")]
    try:
        z = float(parts[2])
    except (IndexError, ValueError):
        return parts[2] if len(parts) > 2 else "", False
    return f"{z:g}", abs(z - z_ir) < 1e-6


def programs() -> pd.DataFrame:
    tasks = load_tasks()
    rows = []
    for f in sorted((FRONTIER / "code").glob("*.urscript")):
        m = NAME.match(f.name)
        if not m:
            continue
        code = f.read_text(errors="replace")
        vs = [v for v in violations(code) if v.detection_mechanism == "SM-6"]
        lines = code.split("\n")
        fm = next((i for i, l in enumerate(lines) if _FIRST_MOTION_RE.search(l)), None)
        head = "\n".join(lines[:fm]) if fm is not None else ""
        rows.append({"program": f.name, "task_id": m.group(1), "model": m.group(2), "condition": m.group(3),
                     "gate_lexical": sr.gate_lexical(code), "gate_structural": sr.gate_structural(code),
                     "tcp_missing": any(v.metadata.get("missing") == "set_tcp" for v in vs),
                     "payload_missing": any(v.metadata.get("missing") == "set_payload" for v in vs),
                     "has_urscript_motion_call": fm is not None,
                     "set_tcp_anywhere": bool(SET_TCP.search(code)), "set_payload_anywhere": bool(SET_PAYLOAD.search(code)),
                     "set_tcp_before_motion": bool(SET_TCP.search(head)),
                     "set_payload_before_motion": bool(SET_PAYLOAD.search(head)),
                     "tcp_offset_z_ir_m": tasks[m.group(1)]["tool"]["tcp_offset"]["position"]["z"] / 1000})
        z, ok = tcp_literal(code, rows[-1]["tcp_offset_z_ir_m"])
        rows[-1].update({"set_tcp_literal_z": z, "set_tcp_literal_z_equals_ir": ok})
    return pd.DataFrame(rows)


def groups(p: pd.DataFrame) -> pd.DataFrame:
    rows = []
    g_all = p[p.gate_lexical]
    sets = [("all", "pooled", g_all)]
    sets += [(c, "pooled", g_all[g_all.condition == c]) for c in ("baseline", "safety")]
    sets += [("all", m, g_all[g_all.model == m]) for m in sorted(g_all.model.unique())]
    for cond, model, g in sets:
        t, pl = g.tcp_missing, g.payload_missing
        neither = g[~t & ~pl]
        rows.append({"condition": cond, "model": model, "gate_passing": len(g),
                     "only_tcp_missing": int((t & ~pl).sum()), "both_missing": int((t & pl).sum()),
                     "only_payload_missing": int((~t & pl).sum()), "neither_missing": len(neither),
                     "neither: no URScript motion call": int((~neither.has_urscript_motion_call).sum()),
                     "neither: both set before first motion call": int(neither.has_urscript_motion_call.sum()),
                     "sm6_flagged": int((t | pl).sum()),
                     "sm6_rate": float((t | pl).mean()) if len(g) else float("nan"),
                     "programs_with_urscript_motion_call": int(g.has_urscript_motion_call.sum()),
                     "tcp_missing_among_motion_call": int((t & g.has_urscript_motion_call).sum()),
                     "payload_missing_among_motion_call": int((pl & g.has_urscript_motion_call).sum()),
                     "set_tcp_before_motion": int(g.set_tcp_before_motion.sum()),
                     "set_tcp_literal_pose": int((g.set_tcp_literal_z.astype(str) != "").sum()),
                     "set_tcp_literal_z_equals_ir": int(g.set_tcp_literal_z_equals_ir.sum())})
    df = pd.DataFrame(rows)
    ref = pd.read_csv(TABLES / "sensitivity_e1_frontier.csv")
    r = ref[(ref.gate == "lexical") & (ref.rules == "SM6-only")].iloc[0]
    mine = df[(df.condition == "all") & (df.model == "pooled")].iloc[0]
    assert mine.gate_passing == r.gate_pass and abs(mine.sm6_rate - r.viol_rate_gate) < 1e-12, (mine, r)
    return df


def prompts() -> dict:
    reqs = [json.loads(l) for l in (FRONTIER / "logs" / "idun_log.jsonl").read_text().splitlines() if l.strip()]
    pairs = {(r.get("system_prompt") or "", r.get("user_prompt") or "") for r in reqs}
    res = {"log_lines": len(reqs), "distinct_system_user_prompt_pairs": len(pairs), "mentions": {}}
    for name, pat in PROMPT_PATTERNS.items():
        rx = re.compile(pat, re.I)
        hits = sorted({line.strip() for s, u in pairs for line in (s + "\n" + u).splitlines() if rx.search(line)})
        res["mentions"][name] = {"prompt_pairs": sum(1 for s, u in pairs if rx.search(s) or rx.search(u)),
                                 "distinct_lines": hits[:12], "n_distinct_lines": len(hits)}
    return res


def main() -> None:
    if get_flag("data_mismatch"):
        print("STOP: data_mismatch flag set by s00_provenance.py")
        return
    if not (FRONTIER / "code").is_dir():
        print(f"SKIP s12: {FRONTIER.relative_to(REPO)} is not in this checkout")
        return
    p = programs()
    p.to_csv(out("f2", "frontier_programs.csv"), index=False)
    g = groups(p)
    g.to_csv(out("f2", "frontier_sm6_groups.csv"), index=False)
    pr = prompts()
    out("f2", "frontier_prompts.json").write_text(json.dumps(pr, indent=2, ensure_ascii=False) + "\n")
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)
    print(g.to_string(index=False))
    print(json.dumps({k: (v["prompt_pairs"], v["n_distinct_lines"]) for k, v in pr["mentions"].items()}))


if __name__ == "__main__":
    main()
