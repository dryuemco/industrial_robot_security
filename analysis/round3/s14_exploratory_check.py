#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Recompute, from the run copies in data/ (s13_import_runs.py), the exploratory results of the
paper that have a committed reference, and compare.

  - Table 7 and the frontier sensitivity table: scripts/review/sensitivity_rescoring.py on
    data/exploratory/E1_frontier/code against paper/tables/sensitivity_e1_frontier.csv and .tex
  - the representation probe figures: scripts/figures/fig_representation_breach.py and
    fig_construct_removal.py on data/exploratory/e5_repr_fragility/e5_results.csv, compared by
    their text (pdftotext) with paper/figures/
  - the frontier summaries: data/exploratory/E1_frontier/e1_summary.json and
    E3_frontier/e3_summary.json against docs/frontier_spotcheck/

Output: out/r3/exploratory_check.csv (one row per comparison, match true/false).
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pandas as pd

from common import REPO, out

DATA = REPO / "data" / "exploratory"


def _pdftext(p: Path) -> str:
    return subprocess.run(["pdftotext", str(p), "-"], capture_output=True, text=True, check=True).stdout


def main() -> None:
    if not DATA.is_dir():
        print("SKIP s14: data/exploratory is not in this checkout")
        return
    rows = []
    with tempfile.TemporaryDirectory(prefix="round3_s14_") as tmp:
        tmp = Path(tmp)
        subprocess.run([sys.executable, str(REPO / "scripts/review/sensitivity_rescoring.py"),
                        "--code-dir", str(DATA / "E1_frontier" / "code"), "--condition", "all",
                        "--label", "E1 frontier (all conditions)", "--out-dir", str(tmp),
                        "--out-name", "sensitivity_e1_frontier"], cwd=REPO, check=True, capture_output=True)
        for ext in ("csv", "tex"):
            ref = REPO / "paper" / "tables" / f"sensitivity_e1_frontier.{ext}"
            rows.append({"result": f"frontier sensitivity table ({ext})", "reference": f"paper/tables/{ref.name}",
                         "match": (tmp / ref.name).read_bytes() == ref.read_bytes()})
        have_pdftotext = shutil.which("pdftotext") is not None
        for fig in ("fig_representation_breach", "fig_construct_removal"):
            subprocess.run([sys.executable, str(REPO / f"scripts/figures/{fig}.py"),
                            "--e5", str(DATA / "e5_repr_fragility" / "e5_results.csv"), "--out", str(tmp / f"{fig}.pdf")],
                           cwd=REPO, check=True, capture_output=True)
            ref = REPO / "paper" / "figures" / f"{fig}.pdf"
            rows.append({"result": f"{fig} (text of the PDF)", "reference": f"paper/figures/{ref.name}",
                         "match": (_pdftext(tmp / f"{fig}.pdf") == _pdftext(ref)) if have_pdftotext else "SKIP (no pdftotext)"})
    for run, name, ref in (("E1_frontier", "e1_summary.json", "e1_frontier_summary.json"),
                           ("E3_frontier", "e3_summary.json", "e3_frontier_summary.json")):
        a, b = DATA / run / name, REPO / "docs" / "frontier_spotcheck" / ref
        rows.append({"result": f"{run}/{name}", "reference": f"docs/frontier_spotcheck/{ref}",
                     "match": a.read_bytes() == b.read_bytes()})
    df = pd.DataFrame(rows)
    df.to_csv(out("r3", "exploratory_check.csv"), index=False)
    print(df.to_string(index=False))
    if not all(m is True for m in df.match if not isinstance(m, str)):
        sys.exit("exploratory check: a recomputed result differs from its committed reference")


if __name__ == "__main__":
    main()
