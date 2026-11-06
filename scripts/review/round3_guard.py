# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Keep the round-2 table generators from overwriting the round-3 tables.

From the round-3 revision (Access-2026-55984) on, some files in paper/tables/ are written
by analysis/round3/s09_tables.py. The round-2 generators that wrote earlier versions of
those files call write_tex(); when the target's first line names the round-3 script, the
round-2 version is written next to it as <name>.r2.tex instead, and a notice is printed.
"""
from __future__ import annotations

from pathlib import Path

ROUND3_MARKER = "analysis/round3/s09_tables.py"


def write_tex(path: Path, text: str) -> Path:
    path = Path(path)
    if path.is_file():
        with path.open() as fh:
            first = fh.readline()
        if ROUND3_MARKER in first:
            alt = path.with_name(path.stem + ".r2.tex")
            print(f"{path} is written by {ROUND3_MARKER}; the round-2 version goes to {alt} instead")
            path = alt
    path.write_text(text)
    return path
