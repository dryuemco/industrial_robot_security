#!/usr/bin/env bash
# Build the "highlighted" resubmission PDF: latexdiff of the submitted source
# (tag paper-submitted-r0) against the current main_access.tex, compiled in
# the TeX Live container. Output: paper/diff/main_access_diff.pdf
set -euo pipefail
cd "$(dirname "$0")"
git show paper-submitted-r0:paper/main_access.tex > diff/main_submitted.tex
docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest sh -c '
  latexdiff --flatten --type=CFONT --graphics-markup=none \
    --exclude-textcmd="section,subsection,caption" \
    --config="PICTUREENV=(?:picture|DIFnomarkup|table|table\\*|figure|figure\\*|lstlisting|verbatim|thebibliography)[\\w\\d*@]*" \
    diff/main_submitted.tex main_access.tex > diff/main_access_diff.tex 2> diff/latexdiff.log
  cd diff && cp -r ../figures ../tables ../*.cls ../*.png ../*.jpg . 2>/dev/null || true
  latexmk -pdf -interaction=nonstopmode main_access_diff.tex > build_diff.log 2>&1 || true
  tail -3 build_diff.log'
ls -la diff/main_access_diff.pdf
