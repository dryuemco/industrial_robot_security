#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Track-2 Probe B1: edit-intent generalization (exploratory; NOT used by RA-L).

Generalizes E5's single PERF edit across edit intents. For each
(intent x representation), apply one benign edit to a safety-annotated program
and measure whether a working guard SURVIVES (dissolution) and whether the
result can breach the cap. General dissolution vs performance-specific are
both publishable; no claim is encoded. Scoring via cap_oracle.analyze_cap.
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import sys
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
for _p in ("enfield_llm", "enfield_tasks", "enfield_watchdog_static", "scripts"):
    _d = str(REPO_DIR / _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)

import llm_experiment_runner as R  # noqa: E402
from enfield_llm.prompt_builder import PromptMode, EditMode  # noqa: E402
from enfield_llm.template_variants import (  # noqa: E402
    Representation, make_variant, parse_cap,
)
import probe_transforms as PT  # noqa: E402
from cap_oracle import analyze_cap  # noqa: E402

logger = logging.getLogger("probe_b1")

_INTENT = {
    "perf": EditMode.PERF,
    "readability": EditMode.READABILITY,
    "feature": EditMode.FEATURE,
    "bugfix": EditMode.BUGFIX,
}
_REPR = {"R4_guard": Representation.R4_GUARD, "R3_named": Representation.R3_NAMED}


def _b1_cell(client, builder, parser, watchdog, task_ir, baseline, cap, rep,
             intent, repr_name, code_dir, sleep, provider):
    task_id = task_ir["task"]["id"]
    variant = make_variant(baseline, _REPR[repr_name], cap)
    if _INTENT[intent] == EditMode.BUGFIX:
        variant = PT.inject_duplicate_waypoint(variant)  # non-safety defect to fix

    r = R.run_single_call(
        client, builder, parser, watchdog, task_ir, PromptMode.BASELINE, None,
        code_dir / f"{intent}_{repr_name}", experiment="ProbeB1", rep=rep,
        edit_mode=_INTENT[intent], template=variant, attach_code=True)
    ed = r.pop("_generated_code", "")
    base = {"intent": intent, "representation": repr_name, "model": client.model,
            "task_id": task_id, "rep": rep}
    if sleep and provider != "mock":
        time.sleep(sleep)
    if r.get("status") != "success" or not ed:
        return [{**base, "status": r.get("status"), "protected": None, "breach": None}]
    res = analyze_cap(ed, cap)
    return [{**base, "status": "success", "protected": res.protected,
             "breach": res.breach}]


def run_probe_b1(tasks, reps, output_dir, provider="idun",
                 intents=("perf", "readability", "feature", "bugfix"),
                 representations=("R4_guard",), mock_seed=None, max_tokens=4096,
                 models=None, sleep=0.0, timeout=300.0):
    model_table = R._models_for_provider(provider, models)
    total = (len(model_table) * len(tasks) * reps * len(intents)
             * len(representations))
    logger.info("PROBE B1 | models=%d tasks=%d reps=%d intents=%s repr=%s | ~%d calls",
                len(model_table), len(tasks), reps, ",".join(intents),
                ",".join(representations), total)

    code_dir = output_dir / "code"
    builder, parser, watchdog = R.PromptBuilder(), R.CodeParser(), R.StaticWatchdog()
    results: list[dict] = []
    for _name, model_id in model_table.items():
        client = R.create_client(provider, model=model_id,
                                 log_dir=output_dir / "logs", max_tokens=max_tokens,
                                 timeout=timeout, seed=mock_seed)
        for task_ir in tasks:
            baseline = R.load_template(task_ir["task"]["id"])
            cap = parse_cap(baseline)
            for rep in range(1, reps + 1):
                for intent in intents:
                    for repr_name in representations:
                        results += _b1_cell(client, builder, parser, watchdog,
                                            task_ir, baseline, cap, rep, intent,
                                            repr_name, code_dir, sleep, provider)
    return results


def write_results(results, output_dir, name="probe_b1"):
    output_dir.mkdir(parents=True, exist_ok=True)
    extra = []
    for r in results:
        for k in r.keys():
            if k not in R.CSV_FIELDS and k not in extra and not k.startswith("_"):
                extra.append(k)
    fieldnames = list(R.CSV_FIELDS) + extra
    csv_path = output_dir / f"{name}_results.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(results)
    summary = PT.summarize_probe_b1(results)
    with open(output_dir / f"{name}_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    logger.info("CSV: %s (%d rows) | summary: %s_summary.json",
                csv_path, len(results), name)
    return summary


def main():
    ap = argparse.ArgumentParser(description="Track-2 Probe B1 (edit-intent).")
    ap.add_argument("--tasks", default=None)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--provider", default="idun")
    ap.add_argument("--models", default=None)
    ap.add_argument("--intents", default="perf,readability,feature,bugfix")
    ap.add_argument("--representations", default="R4_guard")
    ap.add_argument("--max-tokens", type=int, default=4096)
    ap.add_argument("--mock-seed", type=int, default=42)
    ap.add_argument("--sleep", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--output", default=None)
    a = ap.parse_args()

    tasks = R.load_tasks(a.tasks)
    models = [m.strip() for m in a.models.split(",")] if a.models else None
    intents = tuple(x.strip() for x in a.intents.split(","))
    reprs = tuple(x.strip() for x in a.representations.split(","))
    out = Path(a.output) if a.output else REPO_DIR / "results" / "ICSE" / "probe_b1"
    res = run_probe_b1(tasks, a.reps, out, provider=a.provider, intents=intents,
                       representations=reprs, mock_seed=a.mock_seed,
                       max_tokens=a.max_tokens, models=models, sleep=a.sleep,
                       timeout=a.timeout)
    print(json.dumps(write_results(res, out), indent=2))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%H:%M:%S")
    main()
