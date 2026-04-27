#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Watchdog static-analysis latency microbenchmark.

Measures wall-clock time for StaticWatchdog.analyze_combined over
the 15 baseline Task IR files paired with qwen2.5-coder rep=1
retry=0 URScript outputs. Used to back the detection-latency
claim in paper Section V.D.

Output: prints mean / std / 95% CI in milliseconds, and a
'paper_block' JSON summary on stdout for downstream patching.
"""
from __future__ import annotations

import json
import statistics
import sys
import time
from pathlib import Path

from enfield_watchdog_static import StaticWatchdog


REPO = Path(__file__).resolve().parents[1]
TASKS_DIR = REPO / "enfield_tasks" / "ir" / "tasks"
CODE_DIR = REPO / "results" / "e3_confirmatory" / "code"

WARMUP = 5
ITER = 100


def find_urscript_for_task(task_id: str) -> Path | None:
    """Best-effort URScript pair for the given task IR.

    Prefers qwen rep=1 retry=0 baseline; falls back to any other
    .urscript file matching the task id.
    """
    candidates = list(CODE_DIR.glob(f"{task_id}_qwen2.5-coder*baseline*rep1.urscript"))
    candidates = [p for p in candidates if "retry" not in p.name]
    if candidates:
        return candidates[0]
    candidates = list(CODE_DIR.glob(f"{task_id}_*.urscript"))
    candidates = [p for p in candidates if "invalid" not in p.name]
    return candidates[0] if candidates else None


def main() -> int:
    pairs: list[tuple[str, dict, str]] = []
    for task_path in sorted(TASKS_DIR.glob("T*.json")):
        task_id = task_path.stem.split("_")[0]
        urscript_path = find_urscript_for_task(task_id)
        if not urscript_path:
            print(f"  SKIP {task_id}: no URScript pair", file=sys.stderr)
            continue
        with open(task_path) as f:
            task_dict = json.load(f)
        urscript = urscript_path.read_text()
        pairs.append((task_id, task_dict, urscript))

    if not pairs:
        print("FATAL: no pairs found", file=sys.stderr)
        return 1
    print(f"loaded {len(pairs)} (task, urscript) pairs")

    wd = StaticWatchdog()

    # Warm-up
    for task_id, task_dict, urscript in pairs[:WARMUP]:
        wd.analyze_combined(task_dict, urscript)

    # Steady-state
    durations_ms: list[float] = []
    for _ in range(ITER):
        for task_id, task_dict, urscript in pairs:
            t0 = time.perf_counter_ns()
            wd.analyze_combined(task_dict, urscript)
            t1 = time.perf_counter_ns()
            durations_ms.append((t1 - t0) / 1e6)

    n = len(durations_ms)
    mean = statistics.fmean(durations_ms)
    sd = statistics.stdev(durations_ms) if n > 1 else 0.0
    # 95% CI on the mean (two-sided)
    ci_half = 1.96 * (sd / (n ** 0.5))
    ci_low = mean - ci_half
    ci_high = mean + ci_half
    p50 = statistics.median(durations_ms)
    p95 = sorted(durations_ms)[int(n * 0.95)]
    p99 = sorted(durations_ms)[int(n * 0.99)]

    print()
    print("=" * 60)
    print(f"Watchdog analyze_combined microbenchmark")
    print("=" * 60)
    print(f"  n_pairs     : {len(pairs)}")
    print(f"  iter        : {ITER}")
    print(f"  total_calls : {n}")
    print(f"  mean        : {mean:.3f} ms")
    print(f"  sd          : {sd:.3f} ms")
    print(f"  95% CI mean : [{ci_low:.3f}, {ci_high:.3f}] ms")
    print(f"  median      : {p50:.3f} ms")
    print(f"  p95         : {p95:.3f} ms")
    print(f"  p99         : {p99:.3f} ms")
    print()

    summary = {
        "n_pairs": len(pairs),
        "iter": ITER,
        "total_calls": n,
        "mean_ms": round(mean, 3),
        "sd_ms": round(sd, 3),
        "ci95_low_ms": round(ci_low, 3),
        "ci95_high_ms": round(ci_high, 3),
        "median_ms": round(p50, 3),
        "p95_ms": round(p95, 3),
        "p99_ms": round(p99, 3),
    }
    print("paper_block " + json.dumps(summary))
    return 0


if __name__ == "__main__":
    sys.exit(main())
