#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Row-by-row comparison of the two confirmatory runs (Appendix A of the paper).

The confirmatory matrix (E1, E2, E3) was generated twice: a first run on
2026-04-15/16 (results/e1_confirmatory_session14, e2_confirmatory,
e3_confirmatory) and a second run on 2026-05-15/16 (results/E1_full, E2_full,
E3_full). The paper reports the second run. This script reproduces every number
of the "Data collection and the two confirmatory runs" paragraph: row and file
identity per model, the preregistered quantities, and the count-level values
that differ.

It reads result CSVs and generated-code directories only; it calls no model.

Usage:
    python3 scripts/review/compare_confirmatory_runs.py \
        --run1 results/e1_confirmatory_session14 results/e2_confirmatory results/e3_confirmatory \
        --run2 results/E1_full results/E2_full results/E3_full \
        --out-dir docs/confirmatory_results
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import pandas as pd

REPO = Path(__file__).resolve().parents[2]
RUN1 = ["results/e1_confirmatory_session14", "results/e2_confirmatory", "results/e3_confirmatory"]
RUN2 = ["results/E1_full", "results/E2_full", "results/E3_full"]
KEY = ["model", "task_id", "condition", "adversarial_type", "rep", "retry"]
RULES = [f"SM-{i}" for i in range(1, 8)]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(run_dirs: list[Path]) -> dict[str, pd.DataFrame]:
    out = {}
    for d, exp in zip(run_dirs, ("e1", "e2", "e3")):
        df = pd.read_csv(d / f"{exp}_results.csv")
        df["adversarial_type"] = df["adversarial_type"].fillna("")
        df["violation_types"] = df["violation_types"].fillna("")
        out[exp] = df
    return out


def gate(df: pd.DataFrame) -> pd.DataFrame:
    return df[df.status == "success"]


def fires(df: pd.DataFrame, rule: str) -> int:
    return int(df.violation_types.apply(lambda t: rule in [x.strip() for x in t.split(",")]).sum())


def manifest(run_dirs: list[Path]) -> dict:
    """SHA-256 manifest of one run, same shape as docs/confirmatory_results/MANIFEST.json."""
    exps = {}
    for d, exp in zip(run_dirs, ("e1", "e2", "e3")):
        csv = d / f"{exp}_results.csv"
        df = pd.read_csv(csv)
        per_model = {m: {**g.status.value_counts().to_dict(), "_total": len(g)} for m, g in df.groupby("model")}
        exps[exp.upper()] = {
            "directory": str(d),
            "csv_sha256": sha256(csv),
            "log_sha256": sha256(d / "logs" / "ollama_log.jsonl"),
            "data_rows": len(df),
            "code_files": len(list((d / "code").glob("*"))),
            "first_row_utc": df.timestamp.min(),
            "last_row_utc": df.timestamp.max(),
            "per_model": per_model,
        }
    return {"experiments": exps}


def identity(run1_dirs, run2_dirs, r1, r2) -> list[dict]:
    rows = []
    for (d1, d2), exp in zip(zip(run1_dirs, run2_dirs), ("e1", "e2", "e3")):
        j = r1[exp].merge(r2[exp], on=KEY, suffixes=("_1", "_2"), how="outer", indicator=True)
        assert (j._merge == "both").all(), f"{exp}: row keys differ between runs"
        j["same_row"] = (
            (j.status_1 == j.status_2)
            & (j.total_violations_1 == j.total_violations_2)
            & (j.violation_types_1 == j.violation_types_2)
        )
        j["same_status"] = j.status_1 == j.status_2
        j["same_verdict"] = (j.total_violations_1 > 0) == (j.total_violations_2 > 0)
        for model, g in j.groupby("model"):
            tag = model.replace(":", "_")
            files1 = sorted((d1 / "code").glob(f"*{tag}*"))
            same_files = sum(
                1 for f in files1 if (d2 / "code" / f.name).is_file() and f.read_bytes() == (d2 / "code" / f.name).read_bytes()
            )
            rows.append({
                "experiment": exp.upper(), "model": model, "rows": len(g),
                "rows_identical": int(g.same_row.sum()), "status_identical": int(g.same_status.sum()),
                "verdict_identical": int(g.same_verdict.sum()),
                "code_files": len(files1), "code_files_byte_identical": same_files,
            })
    return rows


def headline(r: dict[str, pd.DataFrame]) -> dict:
    e1, e2, e3 = r["e1"], r["e2"], r["e3"]
    b, s = e1[e1.condition == "baseline"], e1[e1.condition == "safety"]
    gb, gs, g2 = gate(b), gate(s), gate(e2)
    safe = gb[gb.total_violations == 0]
    strat = g2.groupby("adversarial_type").total_violations.mean()
    out = {
        "baseline gate-passing outputs": f"{len(gb)}/{len(b)}",
        "adversarial gate-passing outputs": f"{len(g2)}/{len(e2)}",
        "naive baseline violation rate (%)": round((b.total_violations > 0).mean() * 100, 1),
        "gate-passing baseline violation rate (%)": round((gb.total_violations > 0).mean() * 100, 1),
        "substantive-safe baseline programs": "; ".join(f"{x.model} {x.task_id} rep{x.rep}" for x in safe.itertuples()),
        "refusals (E1+E2+E3)": int(e1.refusal.sum() + e2.refusal.sum() + e3.refusal.sum()),
        "mean violations, gate-passing baseline": round(gb.total_violations.mean(), 2),
        "mean violations, gate-passing safety prompt": round(gs.total_violations.mean(), 2),
        "mean violations, all baseline outputs": round(b.total_violations.mean(), 2),
        "adversarial uplift, gate-passing, pooled": round(g2.total_violations.mean() - gb.total_violations.mean(), 2),
        "adversarial uplift, all outputs, pooled": round(e2.total_violations.mean() - b.total_violations.mean(), 2),
        "strongest strategy (gate-passing uplift)": f"{strat.idxmax()} {strat.max() - gb.total_violations.mean():+.2f}",
    }
    for k, v in strat.items():
        out[f"gate-passing mean, {k}"] = round(v, 2)
    for m, g in gb.groupby("model"):
        out[f"gate-passing baseline mean, {m}"] = round(g.total_violations.mean(), 2)
    for rule in RULES:
        out[f"{rule} firings, baseline / safety"] = f"{fires(b, rule)} / {fires(s, rule)}"
    last = e3.sort_values("retry").groupby(["model", "task_id", "rep"]).tail(1)
    for m, g in last.groupby("model"):
        out[f"E3 cells valid at last row, {m}"] = f"{int((g.status == 'success').sum())}/{len(g)}"
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run1", type=Path, nargs=3, default=[REPO / p for p in RUN1], help="E1 E2 E3 directories of the first run")
    ap.add_argument("--run2", type=Path, nargs=3, default=[REPO / p for p in RUN2], help="E1 E2 E3 directories of the second run")
    ap.add_argument("--out-dir", type=Path, default=REPO / "docs" / "confirmatory_results")
    args = ap.parse_args()

    r1, r2 = load(args.run1), load(args.run2)
    ident = pd.DataFrame(identity(args.run1, args.run2, r1, r2))
    h1, h2 = headline(r1), headline(r2)
    head = pd.DataFrame({"quantity": list(h1), "run1_april": list(h1.values()), "run2_may": [h2[k] for k in h1]})
    head["identical"] = head.run1_april.astype(str) == head.run2_may.astype(str)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    ident.to_csv(args.out_dir / "run_comparison_identity.csv", index=False)
    head.to_csv(args.out_dir / "run_comparison_headline.csv", index=False)
    m1 = manifest([p.relative_to(REPO) if p.is_absolute() else p for p in args.run1])
    (args.out_dir / "MANIFEST_run1_april.json").write_text(json.dumps(m1, indent=2, default=str) + "\n")

    tot = ident[["rows", "rows_identical", "status_identical", "verdict_identical", "code_files", "code_files_byte_identical"]].sum()
    print(ident.to_string(index=False))
    print(f"\nall rows: {tot.rows}; status identical {tot.status_identical}; verdict identical {tot.verdict_identical}; "
          f"rows identical {tot.rows_identical}; code files byte-identical {tot.code_files_byte_identical}/{tot.code_files}\n")
    print(head.to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
