#!/usr/bin/env bash
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
# Build docs/review/submission_v10 (resubmission of Access-2026-55984, round 3) from the
# committed sources of review/access-r3, in the layout of the earlier packages:
#   01_main_manuscript  clean PDF, source/ with LaTeX comments removed, source.zip
#   02_highlighted_pdf  latexdiff against 797aeb3 (the source of Access-2026-55984)
#   03_response_to_reviewers  left empty until the response letter is written
#   04_supplementary    round3_analysis.zip (the two earlier zips are added at upload)
#   MANIFEST.md         sizes and SHA-256 of every file, and of every member of the zip
# portal_notes.md is kept as it is. Run from anywhere: bash scripts/review/package_round3.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/docs/review/submission_v10"
P=Access-2026-55984_R1
BASE=797aeb3
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
DOCKER=(docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp)

test -z "$(git -C "$REPO" status --porcelain -- paper analysis/round3)" || { echo "paper/ or analysis/round3/ differ from HEAD"; exit 1; }
COMMIT="$(git -C "$REPO" rev-parse --short=7 HEAD)"

# 1. clean PDF from the committed paper/ tree
mkdir -p "$WORK/build"
git -C "$REPO" archive HEAD paper | tar -x -C "$WORK/build" --strip-components=1
"${DOCKER[@]}" -v "$WORK/build":/work -w /work texlive/texlive:latest \
  sh -c 'latexmk -pdf -interaction=nonstopmode main_access.tex > latexmk.out 2>&1' || true
echo "clean pages: $(pdfinfo "$WORK/build/main_access.pdf" | awk '/^Pages/{print $2}')"
echo "clean errors: $(grep -c '^!' "$WORK/build/main_access.log" || true)"
echo "clean undefined refs/cites: $(grep -c -E 'Reference .* undefined|Citation .* undefined' "$WORK/build/main_access.log" || true)"

# 2. highlighted PDF against the source of Access-2026-55984
bash "$REPO/paper/build_diff.sh" "$BASE" > "$WORK/build_diff.out" 2>&1
echo "highlighted pages: $(pdfinfo "$REPO/paper/diff/main_access_diff.pdf" | awk '/^Pages/{print $2}')"
echo "highlighted errors: $(grep -c '^!' "$REPO/paper/diff/main_access_diff.log" || true)"

# 3. layout (portal_notes.md is kept)
for d in 01_main_manuscript 02_highlighted_pdf 03_response_to_reviewers 04_supplementary; do rm -rf "${OUT:?}/$d"; done
mkdir -p "$OUT"/{01_main_manuscript,02_highlighted_pdf,03_response_to_reviewers,04_supplementary}
cp "$WORK/build/main_access.pdf" "$OUT/01_main_manuscript/${P}_main.pdf"
cp "$REPO/paper/diff/main_access_diff.pdf" "$OUT/02_highlighted_pdf/${P}_highlighted.pdf"
printf 'The response letter goes here once it is written (%s_response.docx and .md).\n' "$P" > "$OUT/03_response_to_reviewers/README.md"

# 4. source/ with comments removed, then source.zip
S="$OUT/01_main_manuscript/source"
mkdir -p "$S/figures" "$S/tables"
cd "$WORK/build"
cp main_access.tex ieeeaccess.cls IEEEtran.cls logo.png notaglinelogo.png bullet.png photo1.jpg photo2.jpg photo3.jpg "$S/"
for f in pipeline fig_violation_rates fig_rule_heatmap fig_repair_trajectory fig_representation_breach fig_construct_removal; do
  cp "figures/$f.pdf" "$S/figures/"
done
grep -o '^[^%]*\\input{[^}]*}' main_access.tex | sed 's/.*\\input{//;s/}$//' > "$WORK/inputs.txt"
while read -r t; do cp "$t" "$S/$t"; done < "$WORK/inputs.txt"
python3 -I "$REPO/paper/strip_comments.py" "$S/main_access.tex" "$S"/tables/*.tex
(cd "$S" && find . -exec touch -d '2026-11-06 00:00:00' {} + && zip -q -X -r ../source.zip .)

# 5. standalone compile check of source.zip against the clean PDF
mkdir -p "$WORK/zipcheck"
unzip -q "$OUT/01_main_manuscript/source.zip" -d "$WORK/zipcheck"
"${DOCKER[@]}" -v "$WORK/zipcheck":/work -w /work texlive/texlive:latest \
  sh -c 'latexmk -pdf -interaction=nonstopmode main_access.tex > latexmk.out 2>&1' || true
pdftotext "$WORK/zipcheck/main_access.pdf" "$WORK/zip.txt"
pdftotext "$OUT/01_main_manuscript/${P}_main.pdf" "$WORK/clean.txt"
cmp -s "$WORK/zip.txt" "$WORK/clean.txt" && echo "source.zip compiles to the same text: yes" || echo "source.zip compiles to the same text: NO"

# 6. supplementary archive
python3 "$REPO/scripts/review/build_round3_supplementary.py" --out "$OUT/04_supplementary/round3_analysis.zip" \
  --members "$WORK/members.tsv"

# 7. MANIFEST.md
python3 - "$OUT" "$COMMIT" "$BASE" "$WORK/members.tsv" "$WORK/inputs.txt" <<'PY'
import hashlib, sys
from pathlib import Path
out, commit, base, members, inputs = Path(sys.argv[1]), sys.argv[2], sys.argv[3], Path(sys.argv[4]), Path(sys.argv[5])
files = sorted(p for p in out.rglob("*") if p.is_file() and p.name != "MANIFEST.md")
rows = "\n".join(f"| {p.relative_to(out)} | {p.stat().st_size} | {hashlib.sha256(p.read_bytes()).hexdigest()} |" for p in files)
mem = "\n".join("| " + " | ".join(line.split("\t")) + " |" for line in members.read_text().splitlines())
n_inputs = len(inputs.read_text().split())
(out / "MANIFEST.md").write_text(f"""# Submission package v10: resubmission of Access-2026-55984 (round 3)

Built from the committed sources of branch review/access-r3 at {commit} with
scripts/review/package_round3.sh. The highlighted PDF is latexdiff against {base}, the source of
Access-2026-55984 (paper/build_diff.sh, with paper/mark_new_floats.py for new and revised floats).
source/ holds main_access.tex and its {n_inputs} table files with LaTeX comments removed
(paper/strip_comments.py), the class files, logos, photos and figures; source.zip compiles on its
own to the same text as the clean PDF.

03_response_to_reviewers/ is empty until the response letter is written. 04_supplementary/ holds
round3_analysis.zip: the tracked files under analysis/round3/ of the branch, the same files as on
the branch, built deterministically by scripts/review/build_round3_supplementary.py (rebuilding
from the same commit gives the same SHA-256). The two supplementary files of the earlier rounds,
audit_v2.zip and confirmatory_run_comparison.zip, are kept on the Windows machine and are added
there; whether they are unchanged from the previous upload is to be checked there.

## Files

| File | Size (bytes) | SHA-256 |
|---|---|---|
{rows}

## Members of 04_supplementary/round3_analysis.zip

| Member | Size (bytes) | SHA-256 |
|---|---|---|
{mem}
""")
print(f"MANIFEST: {len(files)} files, {len(members.read_text().splitlines())} zip members")
PY
