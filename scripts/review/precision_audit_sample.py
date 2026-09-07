#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Draw the stratified sample for precision audit v2 and build rater sheets.

Revision items R1-3 and R2 "single rater": the manuscript's precision audit
rests on 50 firings labelled by one author. This script re-runs the rule
checker over the gate-passing generated code, enumerates every firing
(rule, program, location), draws a seeded stratified sample of up to
--per-rule firings per rule, and writes:

  <out>/sheet_rater_A.csv, sheet_rater_B.csv  identical blank labelling
                                              sheets (label/note columns empty)
  <out>/snippets/<item_id>.txt                the program with the flagged
                                              line marked ">>>"
  <out>/key.csv                               item_id -> source file mapping
                                              (keep away from the raters)

Rater instructions: docs/review/PRECISION_AUDIT_RATER_GUIDE.md.
Agreement and precision are computed afterwards by
scripts/review/precision_audit_analyze.py.

Usage:
    python3 scripts/review/precision_audit_sample.py \
        --code-dirs results/E1_full/code results/E2_full/code \
        --out docs/review/audit_v2 --per-rule 20 --seed 42
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "enfield_watchdog_static"))
sys.path.insert(0, str(REPO / "enfield_tasks"))
sys.path.insert(0, str(REPO / "enfield_llm"))

from enfield_watchdog_static import StaticWatchdog  # noqa: E402
from enfield_llm.code_parser import URSCRIPT_CALL_PATTERN  # noqa: E402

RULES = ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]
FNAME = re.compile(r"^(T\d{3})_(.+?)_(baseline|safety|adversarial_A8\.\d|watchdog)_rep(\d)(?:_retry(\d))?\.urscript$")


def task_modes() -> dict[str, str]:
    modes = {}
    for f in (REPO / "enfield_tasks" / "ir" / "tasks").glob("T*.json"):
        t = json.loads(f.read_text())
        modes[t["task"]["id"]] = t["task"].get("operating_mode", "collaborative")
    return modes


def enumerate_firings(code_dirs: list[Path], wd: StaticWatchdog, modes: dict[str, str]) -> list[dict]:
    firings = []
    for d in code_dirs:
        for f in sorted(d.glob("*.urscript")):
            if ".invalid" in f.name:
                continue
            code = f.read_text(errors="replace")
            if not URSCRIPT_CALL_PATTERN.search(code):
                continue
            m = FNAME.match(f.name)
            task = m.group(1) if m else f.name[:4]
            report = wd.analyze_code(code)
            for v in report.violations:
                firings.append({
                    "rule": v.detection_mechanism,
                    "source": str(f.resolve().relative_to(REPO)),
                    "task_id": task,
                    "operating_mode": modes.get(task, "unknown"),
                    "location": v.location,
                    "description": v.description,
                    "severity": v.severity,
                })
    return firings


def snippet(code: str, location: str, max_lines: int = 120) -> str:
    lines = code.splitlines()
    m = re.match(r"line:(\d+)", location or "")
    target = int(m.group(1)) - 1 if m else None
    if len(lines) > max_lines:
        if target is None:
            lines = lines[:max_lines] + ["... (truncated)"]
            target = None
        else:
            lo = max(0, target - max_lines // 2)
            hi = min(len(lines), lo + max_lines)
            lines = lines[lo:hi]
            target = target - lo
    out = []
    for i, ln in enumerate(lines):
        mark = ">>>" if target is not None and i == target else "   "
        out.append(f"{mark} {ln}")
    if target is None:
        out.insert(0, ">>> (rule applies to the whole program; no single flagged line)")
    return "\n".join(out) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--code-dirs", type=Path, nargs="+",
                    default=[REPO / "results" / "E1_full" / "code"])
    ap.add_argument("--out", type=Path, default=REPO / "docs" / "review" / "audit_v2")
    ap.add_argument("--per-rule", type=int, default=20)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    wd = StaticWatchdog()
    firings = enumerate_firings(args.code_dirs, wd, task_modes())
    rng = random.Random(args.seed)

    sample = []
    pop = {}
    for rule in RULES:
        pool = [x for x in firings if x["rule"] == rule]
        pop[rule] = len(pool)
        rng.shuffle(pool)
        sample.extend(pool[: args.per_rule])
    rng.shuffle(sample)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "snippets").mkdir(exist_ok=True)
    key_rows, sheet_rows = [], []
    for i, x in enumerate(sample, 1):
        item = f"A{i:03d}"
        code = (REPO / x["source"]).read_text(errors="replace")
        (args.out / "snippets" / f"{item}.txt").write_text(snippet(code, x["location"]))
        key_rows.append({"item_id": item, **x})
        sheet_rows.append({
            "item_id": item, "rule": x["rule"], "operating_mode": x["operating_mode"],
            "location": x["location"], "description": x["description"], "label": "", "note": "",
        })

    with (args.out / "key.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(key_rows[0].keys()))
        w.writeheader()
        w.writerows(key_rows)
    for rater in ("A", "B"):
        with (args.out / f"sheet_rater_{rater}.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(sheet_rows[0].keys()))
            w.writeheader()
            w.writerows(sheet_rows)
    (args.out / "population.json").write_text(json.dumps(
        {"code_dirs": [str(d) for d in args.code_dirs], "seed": args.seed,
         "per_rule": args.per_rule, "population_by_rule": pop,
         "sample_by_rule": {r: sum(1 for s in sample if s["rule"] == r) for r in RULES}}, indent=2))

    print("population firings by rule:", pop)
    print("sample by rule:", {r: sum(1 for s in sample if s["rule"] == r) for r in RULES})
    print(f"{len(sample)} items -> {args.out}")


if __name__ == "__main__":
    main()
