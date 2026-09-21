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
  # IEEE Access asks for yellow highlighting of changes: additions on a yellow
  # background (soul), deletions as small red strike-through text (ulem).
  sed -i \
    -e "s|\\\\RequirePackage{color}\\\\definecolor{RED}|\\\\RequirePackage{color,soul}\\\\RequirePackage[normalem]{ulem}\\\\sethlcolor{yellow}\\\\soulregister{\\\\ref}{7}\\\\soulregister{\\\\cite}{7}\\\\soulregister{\\\\pageref}{7}\\\\soulregister{\\\\eqref}{7}\\\\soulregister{\\\\texttt}{1}\\\\soulregister{\\\\emph}{1}\\\\soulregister{\\\\textbf}{1}\\\\soulregister{\\\\textit}{1}\\\\soulregister{\\\\label}{1}\\\\soulregister{\\\\ldots}{0}\\\\definecolor{RED}|" \
    -e "s|\\\\providecommand{\\\\DIFaddtex}\\[1\\]{{\\\\protect\\\\color{blue} \\\\sf #1}}|\\\\providecommand{\\\\DIFaddtex}[1]{\\\\hl{#1}}|" \
    -e "s|\\\\providecommand{\\\\DIFdeltex}\\[1\\]{{\\\\protect\\\\color{red} \\\\scriptsize #1}}|\\\\providecommand{\\\\DIFdeltex}[1]{{\\\\protect\\\\color{red}\\\\scriptsize\\\\sout{#1}}}|"  \
    diff/main_access_diff.tex'
# latexdiff leaves table/figure environments unmarked; flag floats that are new
# or whose graphics changed since the submitted version (runs on the host).
python3 mark_new_floats.py diff/main_access_diff.tex
docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest sh -c '
  cd diff && cp -r ../figures ../tables ../*.cls ../*.png ../*.jpg . 2>/dev/null || true
  latexmk -pdf -interaction=nonstopmode main_access_diff.tex > build_diff.log 2>&1 || true
  tail -3 build_diff.log'
ls -la diff/main_access_diff.pdf
