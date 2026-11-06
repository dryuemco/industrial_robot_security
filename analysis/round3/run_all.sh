#!/usr/bin/env bash
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
# Reruns every round-3 analysis (Access-2026-55984, Reviewer 1 A1-A4, Follow-up 1) from committed inputs.
# Writes only under analysis/round3/out/. OSF responses are read from out/osf/ when cached;
# pass --refresh-osf to re-read api.osf.io (read-only, no login).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
export PYTHONDONTWRITEBYTECODE=1
OSF_FLAG=""
[[ "${1:-}" == "--refresh-osf" ]] && OSF_FLAG="--refresh"

python3 s00_provenance.py
python3 s01_a1.py
python3 s02_a2.py
python3 s06_osf.py $OSF_FLAG
python3 s03_a3.py
python3 s04_a4.py
python3 s05_impact.py
python3 s07_followup1.py
python3 s08_paper_numbers.py
python3 s09_tables.py --out-dir out/tables
for f in out/tables/*.tex; do
  cmp -s "$f" "../../paper/tables/$(basename "$f")" && echo "same as paper/tables: $(basename "$f")" \
    || echo "DIFFERS from paper/tables: $(basename "$f")"
done
python3 -m pytest test_round3.py -q -p no:cacheprovider
echo "flags:"; cat out/flags.json
