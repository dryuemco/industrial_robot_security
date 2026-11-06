#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Build the round-3 supplementary archive round3_analysis.zip deterministically.

Members: the tracked files under analysis/round3/, read from the working tree, which must equal
the committed version of those files (otherwise the script stops). Members are sorted and carry a
fixed timestamp, so the same commit always gives the same SHA-256.

Usage: python3 scripts/review/build_round3_supplementary.py [--out PATH] [--members PATH]
Prints "ZIP <path> <bytes> <sha256> members=<n>"; --members writes one line per member
(path, bytes, sha256) as tab-separated text.
"""
from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO / "docs/review/submission_v10/04_supplementary/round3_analysis.zip"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--members", type=Path)
    args = ap.parse_args()
    files = subprocess.run(["git", "ls-files", "analysis/round3"], cwd=REPO, capture_output=True, text=True,
                           check=True).stdout.split()
    dirty = subprocess.run(["git", "diff", "--name-only", "HEAD", "--", *files], cwd=REPO, capture_output=True,
                           text=True, check=True).stdout.split()
    if dirty:
        sys.exit(f"uncommitted changes in {dirty}; commit them first")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    with zipfile.ZipFile(args.out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for f in sorted(files):
            data = (REPO / f).read_bytes()
            info = zipfile.ZipInfo(f, date_time=(2026, 11, 6, 0, 0, 0))
            info.external_attr = (0o755 if f.endswith(".sh") else 0o644) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            zf.writestr(info, data)
            rows.append((f, len(data), hashlib.sha256(data).hexdigest()))
    digest = hashlib.sha256(args.out.read_bytes()).hexdigest()
    if args.members:
        args.members.write_text("".join(f"{f}\t{n}\t{h}\n" for f, n, h in rows))
    print(f"ZIP {args.out} {args.out.stat().st_size} {digest} members={len(rows)}")


if __name__ == "__main__":
    main()
