#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Provenance: input hashes, committed data vs results/ copies, gate equivalence.

Writes out/provenance_inputs.csv, out/provenance_copies.csv,
out/gate_equivalence.csv and sets flag data_mismatch.
"""
from __future__ import annotations

import hashlib

import pandas as pd

from common import (AUDIT_DIR, OSF_MIRROR, PAPER, REPO, RUN1_E1, RUN2_DIR, RUN2_E1, TASKS_DIR, X10, load_code,
                    load_e1_csv, out, rel, set_flag, sha256, sr)

COPIES = [
    ("data/confirmatory/run2_2026-05/E1_full", "results/E1_full"),
    ("data/confirmatory/run2_2026-05/E2_full", "results/E2_full"),
    ("data/confirmatory/run2_2026-05/E3_full", "results/E3_full"),
    ("data/confirmatory/run1_2026-04/e1_confirmatory_session14", "results/e1_confirmatory_session14"),
]


def tree_digest(root) -> tuple[str, int]:
    """sha256 over (relative path, file sha256) of every data file, logs and pid files excluded."""
    files = sorted(p for p in root.rglob("*") if p.is_file() and "logs" not in p.parts
                   and p.suffix in {".csv", ".json", ".urscript"})
    h = hashlib.sha256()
    for f in files:
        h.update(f"{f.relative_to(root)}\t{sha256(f)}\n".encode())
    return h.hexdigest(), len(files)


def main() -> None:
    rows = []
    for a, b in COPIES:
        pa, pb = REPO / a, REPO / b
        da, na = tree_digest(pa)
        if pb.is_dir():
            db, nb = tree_digest(pb)
        else:
            db, nb = "MISSING", 0
        rows.append({"committed": a, "results_copy": b, "files_committed": na, "files_copy": nb,
                     "digest_committed": da, "digest_copy": db,
                     "identical": da == db if db != "MISSING" else None})
    copies = pd.DataFrame(rows)
    if (copies.digest_copy == "MISSING").any():
        # results/ is outside version control; in a fresh clone the copies are absent and the
        # committed comparison (made where they exist) is kept rather than overwritten.
        print("SKIP copies check: results/ copies not in this checkout; out/provenance_copies.csv left as committed")
        copies = pd.read_csv(out("provenance_copies.csv"))
    else:
        copies.to_csv(out("provenance_copies.csv"), index=False)
    mismatch = bool((copies.identical == False).any())  # noqa: E712
    set_flag("data_mismatch", mismatch)

    inputs = [RUN2_E1 / "e1_results.csv", RUN1_E1 / "e1_results.csv",
              *[v[0] for v in X10.values()],
              *sorted(TASKS_DIR.glob("T*.json")),
              *sorted(AUDIT_DIR.glob("*.csv")), AUDIT_DIR / "population.json",
              REPO / "docs/review/PRECISION_AUDIT_RATER_GUIDE.md",
              REPO / "paper/tables/comparator_programs.csv",
              REPO / "paper/tables/sensitivity_e1_baseline.csv",
              REPO / "paper/tables/quantization_cochran.csv",
              REPO / "paper/tables/precision_audit_v2.csv",
              PAPER,  # as submitted (common.R2_COMMIT)
              OSF_MIRROR]  # as submitted (common.R2_COMMIT)
    pd.DataFrame([{"file": rel(p), "sha256": sha256(p)} for p in inputs]).to_csv(
        out("provenance_inputs.csv"), index=False)

    # Gate equivalence: status == success  <=>  lexical gate on the stored program
    # <=> no ".invalid" suffix on the file name.
    eq = []
    for run, d in (("run2", RUN2_E1), ("run1", RUN1_E1)):
        csv = load_e1_csv(d)
        for cond in ("baseline", "safety"):
            progs = {(p["task_id"], p["model"], p["rep"]): p for p in load_code(d, cond)}
            sub = csv[csv.condition == cond]
            n_lex = n_inv = n_both = n_nofile = n_mismatch = 0
            for _, r in sub.iterrows():
                p = progs.get((r.task_id, r.model, int(r.rep)))
                if p is None:
                    n_nofile += 1
                    if r.status == "success":
                        n_mismatch += 1
                    continue
                lex = sr.gate_lexical(p["code"])
                ok_status = r.status == "success"
                n_lex += lex
                n_inv += p["invalid_suffix"]
                if lex != ok_status or lex == p["invalid_suffix"]:
                    n_mismatch += 1
                n_both += 1
            eq.append({"run": run, "condition": cond, "csv_rows": len(sub),
                       "status_success": int((sub.status == "success").sum()),
                       "rows_with_program_file": n_both, "rows_without_file": n_nofile,
                       "lexical_gate_pass": n_lex, "invalid_suffix": n_inv,
                       "mismatches": n_mismatch,
                       "rows_without_file_status": ";".join(
                           f"{k}={v}" for k, v in sub[~sub.apply(
                               lambda r: (r.task_id, r.model, int(r.rep)) in progs, axis=1)]
                           .status.value_counts().items())})
    pd.DataFrame(eq).to_csv(out("gate_equivalence.csv"), index=False)
    print(copies.to_string(index=False))
    print(pd.DataFrame(eq).to_string(index=False))


if __name__ == "__main__":
    main()
