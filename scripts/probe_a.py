#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Track-2 Probe A: latent-harm probe (exploratory; NOT used by RA-L/main_v6).

Turns E5's 'constructs dissolve under a benign edit' observation into a
measured safety question: when the TCP-cap guard is absent, does a downstream
benign speed-up edit actually drive the TCP over the cap?

  natural arm    -- Stage-1 benign perf edit of an R4-guarded program, bucket by
                    whether a (any-form) guard survived, then Stage-2 speed-up
                    edit; breach scored by the hardened cap oracle.
  controlled arm -- deterministic V_present (R4) vs V_absent (guard stripped,
                    cap constant kept) from the SAME baseline; same Stage-2
                    speed-up edit applied to both; isolates guard enforcement.

No claim is encoded; both directions (dissolution harmful vs benign) are
reported. Sim-only: code-level + watchdog/oracle score, no physical execution.
Orchestration mirrors run_e5; all scoring goes through cap_oracle.analyze_cap.
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

import llm_experiment_runner as R  # noqa: E402  (reuse runner surface verbatim)
from enfield_llm.prompt_builder import PromptMode, EditMode  # noqa: E402
from enfield_llm.template_variants import (  # noqa: E402
    Representation, make_variant, parse_cap,
)
import probe_transforms as PT  # noqa: E402
from cap_oracle import analyze_cap  # noqa: E402

logger = logging.getLogger("probe_a")


def _natural_cell(client, builder, parser, watchdog, task_ir, baseline, cap,
                  rep, code_dir, sleep, provider):
    task_id = task_ir["task"]["id"]
    variant = make_variant(baseline, Representation.R4_GUARD, cap)
    base = {"arm": "natural", "model": client.model, "task_id": task_id, "rep": rep}

    r1 = R.run_single_call(
        client, builder, parser, watchdog, task_ir, PromptMode.BASELINE, None,
        code_dir / "natural_stage1", experiment="ProbeA", rep=rep,
        edit_mode=EditMode.PERF, template=variant, attach_code=True)
    s1 = r1.pop("_generated_code", "")
    if r1.get("status") != "success" or not s1:
        return [{**base, "stage": "stage1", "status": r1.get("status"),
                 "bucket": None, "breach": None}]
    bucket = PT.stage1_bucket(s1, cap)

    r2 = R.run_single_call(
        client, builder, parser, watchdog, task_ir, PromptMode.BASELINE, None,
        code_dir / "natural_stage2", experiment="ProbeA", rep=rep,
        edit_mode=EditMode.SPEEDUP, template=s1, attach_code=True)
    s2 = r2.pop("_generated_code", "")
    if sleep and provider != "mock":
        time.sleep(sleep)
    if r2.get("status") != "success" or not s2:
        return [{**base, "stage": "stage2", "status": r2.get("status"),
                 "bucket": bucket, "breach": None}]
    return [{**base, "stage": "stage2", "status": "success",
             "bucket": bucket, "breach": analyze_cap(s2, cap).breach}]


def _controlled_cell(client, builder, parser, watchdog, task_ir, baseline, cap,
                     rep, code_dir, sleep, provider):
    task_id = task_ir["task"]["id"]
    present, absent = PT.make_controlled_pair(baseline, cap)
    rows = []
    for vname, vcode in (("present", present), ("absent", absent)):
        r = R.run_single_call(
            client, builder, parser, watchdog, task_ir, PromptMode.BASELINE, None,
            code_dir / f"controlled_{vname}", experiment="ProbeA", rep=rep,
            edit_mode=EditMode.SPEEDUP, template=vcode, attach_code=True)
        ed = r.pop("_generated_code", "")
        base = {"arm": "controlled", "variant": vname, "model": client.model,
                "task_id": task_id, "rep": rep}
        if r.get("status") != "success" or not ed:
            rows.append({**base, "status": r.get("status"), "breach": None,
                         "guard_survived_stage2": None})
        else:
            res = analyze_cap(ed, cap)
            rows.append({**base, "status": "success", "breach": res.breach,
                         "guard_survived_stage2": res.protected})
        if sleep and provider != "mock":
            time.sleep(sleep)
    return rows


def run_probe_a(tasks, reps, output_dir, provider="idun",
                arms=("natural", "controlled"), mock_seed=None, max_tokens=4096,
                models=None, sleep=0.0, timeout=300.0):
    model_table = R._models_for_provider(provider, models)
    per_cell = (1 if "natural" in arms else 0) + (2 if "controlled" in arms else 0)
    total = len(model_table) * len(tasks) * reps * per_cell
    logger.info("PROBE A | models=%d tasks=%d reps=%d arms=%s | ~%d calls",
                len(model_table), len(tasks), reps, ",".join(arms), total)

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
                if "natural" in arms:
                    results += _natural_cell(client, builder, parser, watchdog,
                                             task_ir, baseline, cap, rep, code_dir,
                                             sleep, provider)
                if "controlled" in arms:
                    results += _controlled_cell(client, builder, parser, watchdog,
                                                task_ir, baseline, cap, rep, code_dir,
                                                sleep, provider)
    return results


def write_results(results, output_dir, name="probe_a"):
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
    summary = PT.summarize_probe_a(results)
    with open(output_dir / f"{name}_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    logger.info("CSV: %s (%d rows) | summary: %s_summary.json",
                csv_path, len(results), name)
    return summary


def main():
    ap = argparse.ArgumentParser(description="Track-2 Probe A (latent-harm).")
    ap.add_argument("--tasks", default=None, help="e.g. T001-T015")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--provider", default="idun")
    ap.add_argument("--models", default=None, help="comma-separated model ids")
    ap.add_argument("--arms", default="natural,controlled")
    ap.add_argument("--max-tokens", type=int, default=4096)
    ap.add_argument("--mock-seed", type=int, default=42)
    ap.add_argument("--sleep", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--output", default=None)
    a = ap.parse_args()

    tasks = R.load_tasks(a.tasks)
    models = [m.strip() for m in a.models.split(",")] if a.models else None
    arms = tuple(x.strip() for x in a.arms.split(","))
    out = Path(a.output) if a.output else REPO_DIR / "results" / "ICSE" / "probe_a"
    res = run_probe_a(tasks, a.reps, out, provider=a.provider, arms=arms,
                      mock_seed=a.mock_seed, max_tokens=a.max_tokens, models=models,
                      sleep=a.sleep, timeout=a.timeout)
    print(json.dumps(write_results(res, out), indent=2))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%H:%M:%S")
    main()
