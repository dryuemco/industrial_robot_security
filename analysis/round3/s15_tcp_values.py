#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""The set_tcp declarations of the gate-passing outputs and their values against the TCP offset of
the task representation, for the local baseline run (E1, run 2, 83 outputs) and the frontier run
(120 outputs).

For each output: whether set_tcp occurs anywhere, whether it occurs before the first motion
call (as SM-6 reads it), the first set_tcp pose when it is written as a literal p[...], and
whether its z component (and the whole pose) equals the offset of the task representation
(position in mm / 1000, orientation identity, so the expected pose is p[0, 0, z, 0, 0, 0]).

Outputs under out/r3/: tcp_values_programs.csv (one row per output with a set_tcp call) and
tcp_values_summary.csv (counts per run, all outputs and before the first motion call).
"""
from __future__ import annotations

import re

import pandas as pd

from common import REPO, RUN2_E1, load_code, load_tasks, out, sr

from enfield_watchdog_static.rules.sm6_missing_preamble import _FIRST_MOTION_RE  # noqa: E402

FRONTIER = REPO / "data" / "exploratory" / "E1_frontier" / "code"
NAME = re.compile(r"^(T\d{3})_(.+)_(baseline|safety)_rep(\d)(\.invalid)?\.urscript$")
SET_TCP = re.compile(r"\bset_tcp\s*\(", re.I)
POSE = re.compile(r"\bset_tcp\s*\(\s*p\[\s*([^\]]*)\]", re.I)


def pose_check(code: str, z_ir: float) -> dict:
    m = POSE.search(code)
    if not m:
        return {"literal_pose": "", "z_equals_ir": False, "pose_equals_ir": False}
    parts = [x.strip() for x in m.group(1).split(",")]
    try:
        vals = [float(x) for x in parts]
    except ValueError:
        return {"literal_pose": m.group(1).strip(), "z_equals_ir": False, "pose_equals_ir": False}
    expected = [0.0, 0.0, z_ir, 0.0, 0.0, 0.0]
    z_ok = len(vals) > 2 and abs(vals[2] - z_ir) < 1e-6
    pose_ok = len(vals) == 6 and all(abs(a - b) < 1e-6 for a, b in zip(vals, expected))
    return {"literal_pose": ", ".join(f"{v:g}" for v in vals), "z_equals_ir": z_ok, "pose_equals_ir": pose_ok}


def rows_for(run: str, items) -> list[dict]:
    tasks = load_tasks()
    rows = []
    for name, task, model, cond, code in items:
        if not sr.gate_lexical(code) or not SET_TCP.search(code):
            continue
        lines = code.split("\n")
        fm = next((i for i, l in enumerate(lines) if _FIRST_MOTION_RE.search(l)), None)
        head = "\n".join(lines[:fm]) if fm is not None else ""
        z_ir = tasks[task]["tool"]["tcp_offset"]["position"]["z"] / 1000
        rows.append({"run": run, "program": name, "task_id": task, "model": model, "condition": cond,
                     "before_first_motion": bool(SET_TCP.search(head)), "tcp_offset_z_ir_m": z_ir,
                     **pose_check(code, z_ir)})
    return rows


def main() -> None:
    local = [(p["file"].name, p["task_id"], p["model_disp"], "baseline", p["code"]) for p in load_code(RUN2_E1, "baseline")]
    n_local = sum(1 for *_, c in local if sr.gate_lexical(c))
    front = []
    for f in sorted(FRONTIER.glob("*.urscript")):
        m = NAME.match(f.name)
        if m:
            front.append((f.name, m.group(1), m.group(2), m.group(3), f.read_text(errors="replace")))
    n_front = sum(1 for *_, c in front if sr.gate_lexical(c))
    progs = pd.DataFrame(rows_for("local E1 baseline", local) + rows_for("frontier E1", front))
    progs.to_csv(out("r3", "tcp_values_programs.csv"), index=False)
    summ = []
    for run, n in (("local E1 baseline", n_local), ("frontier E1", n_front)):
        g = progs[progs.run == run]
        for scope, s in (("anywhere", g), ("before the first motion call", g[g.before_first_motion])):
            lit = s[s.literal_pose.astype(str) != ""]
            summ.append({"run": run, "gate_passing_outputs": n, "set_tcp": scope, "outputs": len(s),
                         "literal_pose": len(lit), "z_equals_ir": int(lit.z_equals_ir.sum()),
                         "pose_equals_ir": int(lit.pose_equals_ir.sum())})
    summ = pd.DataFrame(summ)
    summ.to_csv(out("r3", "tcp_values_summary.csv"), index=False)
    pd.set_option("display.width", 200)
    print(summ.to_string(index=False))
    print(progs[progs.run == "local E1 baseline"][["program", "before_first_motion", "literal_pose", "tcp_offset_z_ir_m",
                                                   "z_equals_ir"]].to_string(index=False))


if __name__ == "__main__":
    main()
