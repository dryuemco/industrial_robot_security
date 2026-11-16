#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Numbers behind the presentation items B1 to B9 of Reviewer 1 (Access-2026-55984).

B1  Table 8 counts the outputs on which each security check fires. Every such output passes
    the validity gate, so the counts are over the 83 baseline and 108 safety-prompted
    gate-passing outputs of run 2.
B2  The severity a check assigns to one firing, read from the rule sources, and the values
    observed on the stored E1 outputs of run 2.
B3  The four frontier models: identifiers as served, number of calls and dates of the calls,
    for every additional run that used them.
B5  Table 5 counts cells as the distinct model, task and condition combinations, and the
    repair loop as trajectories, one per cell and repetition.
B7  Genuine speed-cap breaches (move commands whose literal speed exceeds the task's own cap,
    category C of scripts/sm5_audit.py) in the control probe, as breaches and as programs
    with at least one breach, from scratch and after the edit.

Outputs under out/b/: b1_perrule_by_gate.csv, b2_severity.csv, b3_frontier_models.csv,
b5_cells.csv, b7_breaches.csv.
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict

import pandas as pd

from common import REPO, RUN2_DIR, RUN2_E1, RULES, out, violations

sys.path.insert(0, str(REPO / "scripts"))
import sm5_audit  # noqa: E402

RULE_DIR = REPO / "enfield_watchdog_static" / "enfield_watchdog_static" / "rules"
RULE_FILES = {
    "SM-1": "sm1_input_validation.py", "SM-2": "sm2_unchecked_return.py",
    "SM-3": "sm3_protection_failure.py", "SM-4": "sm4_unusual_conditions.py",
    "SM-5": "sm5_hardcoded_values.py", "SM-6": "sm6_missing_preamble.py",
    "SM-7": "sm7_prompt_marker.py",
}
LITERAL_SEVERITY = re.compile(r"severity\s*=\s*([0-9]*\.[0-9]+)")
EXPLORATORY = REPO / "data" / "exploratory"
FRONTIER_RUNS = ["E1_frontier", "E3_frontier", "E3_rich_probe", "temp_robustness",
                 "e4_template_probe", "e5_repr_fragility"]
PROBE_CODE = EXPLORATORY / "e4_template_probe" / "code"


def b1() -> pd.DataFrame:
    d = pd.read_csv(RUN2_E1 / "e1_results.csv")
    rows = []
    for cond in ["baseline", "safety"]:
        s = d[d.condition == cond]
        gate = s.status == "success"
        types = s.violation_types.fillna("")
        for rule in RULES:
            fires = types.str.contains(rf"{rule}\b", regex=True)
            rows.append({"condition": cond, "check": rule, "generations": len(s),
                         "gate_passing": int(gate.sum()),
                         "outputs_firing": int(fires.sum()),
                         "outputs_firing_gate_passing": int((fires & gate).sum()),
                         "outputs_firing_not_gate_passing": int((fires & ~gate).sum())})
    return pd.DataFrame(rows)


def b2() -> pd.DataFrame:
    observed = defaultdict(set)
    for f in sorted((RUN2_E1 / "code").glob("*.urscript")):
        if f.name.endswith(".invalid.urscript"):
            continue
        for v in violations(f.read_text(errors="replace")):
            if v.detection_mechanism in RULE_FILES:
                observed[v.detection_mechanism].add(round(float(v.severity), 3))
    rows = []
    for rule, name in RULE_FILES.items():
        src = (RULE_DIR / name).read_text()
        literals = sorted({float(x) for x in LITERAL_SEVERITY.findall(src)})
        obs = sorted(observed.get(rule, set()))
        rows.append({"check": rule,
                     "severity_in_source": " ".join(f"{x:g}" for x in literals)
                     or "computed: min((value - limit) / limit, 1)",
                     "observed_min": min(obs) if obs else "",
                     "observed_max": max(obs) if obs else "",
                     "observed_distinct": len(obs)})
    return pd.DataFrame(rows)


def b3() -> pd.DataFrame:
    rows = []
    for run in FRONTIER_RUNS:
        for log in sorted((EXPLORATORY / run).rglob("*.jsonl")):
            by_model = defaultdict(list)
            for line in log.read_text().splitlines():
                r = json.loads(line).get("response", {})
                if r.get("model"):
                    by_model[r["model"]].append(r.get("timestamp", ""))
            for model, ts in sorted(by_model.items()):
                ts = sorted(t for t in ts if t)
                rows.append({"run": run, "log": str(log.relative_to(EXPLORATORY / run)), "model": model, "calls": len(ts),
                             "first": ts[0][:10] if ts else "", "last": ts[-1][:10] if ts else ""})
    return pd.DataFrame(rows)


def b5() -> pd.DataFrame:
    rows = []
    for family, path, condition in [("baseline", RUN2_E1 / "e1_results.csv", "condition"),
                                    ("adversarial", RUN2_DIR / "E2_full" / "e2_results.csv",
                                     "adversarial_type"),
                                    ("repair_in_loop", RUN2_DIR / "E3_full" / "e3_results.csv",
                                     "condition")]:
        d = pd.read_csv(path)
        key = ["model", "task_id", condition]
        rows.append({"family": family, "rows": len(d),
                     "cells": d.groupby(key).ngroups,
                     "cell_repetitions": d.groupby(key + ["rep"]).ngroups,
                     "models": d.model.nunique(), "conditions": d[condition].nunique(),
                     "gate_pass": int((d.status == "success").sum())})
    return pd.DataFrame(rows)


def b7() -> pd.DataFrame:
    caps = sm5_audit.task_cap_map()
    agg = defaultdict(lambda: {"stored_programs": 0, "genuine_breaches": 0,
                               "programs_with_breach": 0})
    for f in sorted(PROBE_CODE.glob("*.urscript")):
        group = "from_scratch" if "_baseline_" in f.name else "edited"
        c = sm5_audit.classify(f.read_text(errors="replace"),
                               caps.get(sm5_audit._parse_task_id(f.name), 0.25))["real_movel"]
        a = agg[group]
        a["stored_programs"] += 1
        a["genuine_breaches"] += c
        a["programs_with_breach"] += int(c > 0)
    res = pd.read_csv(EXPLORATORY / "e4_template_probe" / "e4_results.csv")
    gens = {"from_scratch": int((res.condition == "A_control").sum()),
            "edited": int(res.condition.str.startswith("B_").sum())}
    return pd.DataFrame([{"group": g, "generations": gens[g], **v} for g, v in agg.items()])


def main() -> None:
    for name, frame in [("b1_perrule_by_gate.csv", b1()), ("b2_severity.csv", b2()),
                        ("b3_frontier_models.csv", b3()), ("b5_cells.csv", b5()),
                        ("b7_breaches.csv", b7())]:
        frame.to_csv(out("b", name), index=False)
        print(f"out/b/{name}")
        print(frame.to_string(index=False))


if __name__ == "__main__":
    main()
