#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Re-run and merge the cells of a quantization-control run that timed out.

The runner has a per-request timeout (300 s by default); on this host the
DeepSeek MoE model occasionally exceeds it, which the confirmatory run on the
original host never did. Such cells are recorded with status "error" and
zero violations, which would bias the control comparison. This tool

  1. lists the (task, condition, rep) cells with status == error in RUN/e1_results.csv,
  2. re-runs the affected tasks with a longer timeout into RUN_rerun/ (the
     runner re-runs whole tasks: both conditions, all reps),
  3. replaces only the errored rows (and their code files) in RUN with the
     matching rows from the re-run, leaving every non-errored row untouched,
  4. rewrites e1_results.csv and refreshes the archived copy under paper/data/.

Usage:
    python3 scripts/review/rerun_errored_cells.py --run results/X10_deepseek_q4_K_M \
        --model deepseek-coder-v2:16b-lite-instruct-q4_K_M --timeout 1500
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd

REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--timeout", type=int, default=1500)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    csv = args.run / "e1_results.csv"
    df = pd.read_csv(csv)
    err = df[df.status == "error"]
    if err.empty:
        print("no errored cells; nothing to do")
        return
    tasks = sorted(err.task_id.unique())
    print("errored cells:", err[["task_id", "condition", "rep"]].to_dict("records"))
    print("tasks to re-run:", tasks)
    if args.dry_run:
        return

    rerun = args.run.parent / (args.run.name + "_rerun")
    env = dict(os.environ)
    env["ENFIELD_TASKS_DIR"] = str(REPO / "paper" / "data" / "tasks_as_run")
    env["PYTHONPATH"] = ":".join(str(REPO / p) for p in ("enfield_llm", "enfield_tasks", "enfield_watchdog_static", "enfield_translators"))
    cmd = [sys.executable, "scripts/llm_experiment_runner.py", "--experiment", "E1", "--provider", "ollama",
           "--models", args.model, "--reps", "3", "--max-tokens", "4096", "--temperature", "0.0",
           "--timeout", str(args.timeout), "--tasks", ",".join(tasks), "--output", str(rerun)]
    print("running:", " ".join(cmd))
    subprocess.run(cmd, cwd=REPO, env=env, check=True)

    new = pd.read_csv(rerun / "e1_results.csv")
    key = ["task_id", "condition", "rep"]
    replaced = 0
    for _, row in err.iterrows():
        m = new[(new.task_id == row.task_id) & (new.condition == row.condition) & (new.rep == row.rep)]
        if m.empty:
            print("WARNING: no re-run row for", row[key].to_dict())
            continue
        nr = m.iloc[0]
        if nr.status == "error":
            print("WARNING: re-run also errored for", row[key].to_dict())
        idx = df.index[(df.task_id == row.task_id) & (df.condition == row.condition) & (df.rep == row.rep)][0]
        df.loc[idx, new.columns] = nr[new.columns].values
        df.loc[idx, "rerun_timeout_s"] = args.timeout
        replaced += 1
        # code files: copy whichever file the re-run produced for this cell
        for f in (rerun / "code").glob(f"{row.task_id}_*_{row.condition}_rep{row.rep}*.urscript"):
            shutil.copy2(f, args.run / "code" / f.name)
    df.to_csv(csv, index=False)
    print(f"replaced {replaced} rows; status now:", df.status.value_counts().to_dict())
    arch = REPO / "paper" / "data" / args.run.name
    if arch.is_dir():
        shutil.copy2(csv, arch / "e1_results.csv")
        (arch / "RERUN_NOTE.txt").write_text(
            f"{replaced} cells that hit the {300}s request timeout were re-run with --timeout {args.timeout} "
            f"on tasks {tasks} and merged; see column rerun_timeout_s.\n")
        print("archived to", arch)


if __name__ == "__main__":
    main()
