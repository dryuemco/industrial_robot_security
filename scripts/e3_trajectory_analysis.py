#!/usr/bin/env python3
"""E3 watchdog-in-loop trajectory classifier.

Usage: python3 e3_trajectory_analysis.py <path/to/e3_results.csv>

Groups rows into cells (model, task, rep), orders by retry, classifies
each trajectory, and reports wall-clock and retry-usage statistics.
Classes:
  initial_not_success        loop never started (initial call invalid/error)
  clean_first_try            initial call valid with zero violations
  remediated_to_zero         a retry reached valid + zero violations
  exit_via_invalidation      loop broke because a retry went non-success
  fixed_point                all success calls, identical violation counts
  monotone_decrease_nonzero  strictly non-increasing, never reaches zero
  monotone_increase          feedback made it worse
  oscillation_or_mixed       everything else

Deterministic-rep caveat: with temperature=0.0 the reps are typically
byte-identical; the script reports both cell counts and unique-trajectory
counts. Quote the unique counts in any per-trajectory claim.
"""
import csv
import sys
from collections import Counter, defaultdict
from datetime import datetime


def classify(seq):
    statuses = [r["status"] for r in seq]
    viols = [int(r["total_violations"]) for r in seq]
    if statuses[0] != "success":
        return "initial_not_success"
    if viols[0] == 0:
        return "clean_first_try"
    last = seq[-1]
    if last["status"] != "success":
        return "exit_via_invalidation"
    if int(last["total_violations"]) == 0:
        return "remediated_to_zero"
    sv = [v for s, v in zip(statuses, viols) if s == "success"]
    if len(set(sv)) == 1:
        return "fixed_point"
    if all(b <= a for a, b in zip(sv, sv[1:])):
        return "monotone_decrease_nonzero"
    if all(b >= a for a, b in zip(sv, sv[1:])):
        return "monotone_increase"
    return "oscillation_or_mixed"


def main(path: str) -> int:
    rows = list(csv.DictReader(open(path)))
    assert rows, f"no rows in {path}"

    cells = defaultdict(list)
    for r in rows:
        cells[(r["model"], r["task_id"], int(r["rep"]))].append(r)
    for k in cells:
        cells[k].sort(key=lambda r: int(r["retry"]))

    summary = Counter()
    per_model = defaultdict(Counter)
    retries_used = defaultdict(list)
    unique_trajs = defaultdict(set)
    for (model, task, rep), seq in sorted(cells.items()):
        cls = classify(seq)
        summary[cls] += 1
        per_model[model][cls] += 1
        retries_used[model].append(len(seq) - 1)
        sig = (task, tuple(r["status"] for r in seq),
               tuple(r["total_violations"] for r in seq))
        unique_trajs[model].add(sig)

    n = len(cells)
    print(f"rows={len(rows)}  cells={n}  "
          f"unique trajectories={sum(len(v) for v in unique_trajs.values())}")
    print(f"retry calls total: {len(rows) - n}")
    print(f"remediated_to_zero: {summary.get('remediated_to_zero', 0)}/{n}")
    print(f"clean_first_try:    {summary.get('clean_first_try', 0)}/{n}")

    print("\nGLOBAL trajectory classes (cells):")
    for cls, c in summary.most_common():
        print(f"  {cls:28s} {c:4d}  ({100 * c / n:.0f}%)")

    print("\nPER MODEL:")
    for m in sorted(per_model):
        tot = sum(per_model[m].values())
        mean_r = sum(retries_used[m]) / len(retries_used[m])
        print(f"  {m} (cells={tot}, unique={len(unique_trajs[m])}, "
              f"mean retries used={mean_r:.2f}):")
        for cls, c in per_model[m].most_common():
            print(f"      {cls:28s} {c:4d}")

    ts = sorted(datetime.fromisoformat(r["timestamp"]) for r in rows
                if r.get("timestamp"))
    if ts:
        print(f"\nWALL CLOCK: {ts[0]} -> {ts[-1]}  ({ts[-1] - ts[0]})")
        for m in sorted(per_model):
            mts = sorted(datetime.fromisoformat(r["timestamp"])
                         for r in rows if r["model"] == m)
            print(f"  {m}: {mts[0].strftime('%H:%M:%S')} -> "
                  f"{mts[-1].strftime('%H:%M:%S')}  ({mts[-1] - mts[0]})")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
