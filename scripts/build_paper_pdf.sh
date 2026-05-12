#!/usr/bin/env bash
# =============================================================================
# ENFIELD - Paper PDF Build (draft only, not RA-L submission camera-ready)
# =============================================================================
# Builds a PDF preview of paper/draft_v0.1.md for editorial use during the
# NTNU Visit 1 paper-editing window. xelatex backend, DejaVu Serif body,
# DejaVu Sans Mono code, output written to paper/build/draft_v0.1.pdf.
#
# This script produces a draft preview PDF, NOT the IEEE RA-L camera-ready
# submission. The RA-L submission requires IEEEtran.cls typesetting and the
# pre-submission anonymization pass on the submission/ral-2026 branch; this
# script is for editing-loop convenience on the main branch.
#
# Usage:
#   ./scripts/build_paper_pdf.sh
#
# Output:
#   paper/build/draft_v0.1.pdf
#
# Dependencies (verified at runtime):
#   - pandoc (>=2.9)
#   - xelatex (TeX Live 2022 or newer)
#   - DejaVu Serif font family
#   - DejaVu Sans Mono font
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PAPER_SRC="$REPO_DIR/paper/draft_v0.1.md"
BUILD_DIR="$REPO_DIR/paper/build"
OUT_PDF="$BUILD_DIR/draft_v0.1.pdf"

GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

err() {
    echo -e "${RED}ERROR:${NC} $*" >&2
    exit 1
}

info() {
    echo -e "${CYAN}>>${NC} $*"
}

# -----------------------------------------------------------------------------
# Pre-flight checks
# -----------------------------------------------------------------------------
info "Pre-flight checks..."

[[ -f "$PAPER_SRC" ]] || err "Paper source not found: $PAPER_SRC"

command -v pandoc >/dev/null 2>&1 || err "pandoc not installed. Install with: sudo apt install pandoc"
command -v xelatex >/dev/null 2>&1 || err "xelatex not installed. Install with: sudo apt install texlive-xetex"

PANDOC_VERSION="$(pandoc --version | head -1 | awk '{print $2}')"
XELATEX_VERSION="$(xelatex --version | head -1 | awk '{print $2}')"

# Font availability check (DejaVu Serif + DejaVu Sans Mono).
# Use bash built-in pattern match instead of an echo/grep pipeline: pipefail
# + SIGPIPE on the 81 KB fc-list output combined with grep -q early-exit
# produces a false-positive "not installed" error non-deterministically.
# Built-in [[ ]] has no subprocess and no pipe, so the issue cannot recur.
FONT_LIST="$(fc-list)"
if [[ "${FONT_LIST,,}" != *"dejavu serif"* ]]; then
    err "DejaVu Serif font not installed. Install with: sudo apt install fonts-dejavu"
fi
if [[ "${FONT_LIST,,}" != *"dejavu sans mono"* ]]; then
    err "DejaVu Sans Mono font not installed. Install with: sudo apt install fonts-dejavu"
fi

info "pandoc $PANDOC_VERSION / xelatex $XELATEX_VERSION / DejaVu fonts present"

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
mkdir -p "$BUILD_DIR"

info "Building $OUT_PDF ..."

# Pinned settings:
#   --pdf-engine=xelatex     UTF-8 safe (§, , -> all render correctly)
#   --resource-path          let pandoc find paper/figures/*.pdf inline
#   mainfont/monofont        deterministic visual output across replicators
#   geometry margin=1in      standard editing margin (NOT RA-L 2-column)
#   --toc --toc-depth=3      include the ## / ### structure as ToC
#   colorlinks=true          clickable section refs and external URLs
#   --top-level-division=section   force H1 -> \section (paper has its own H1)
pandoc "$PAPER_SRC" \
    --output "$OUT_PDF" \
    --pdf-engine=xelatex \
    --resource-path="$REPO_DIR/paper:$REPO_DIR/paper/figures" \
    --variable mainfont="DejaVu Serif" \
    --variable monofont="DejaVu Sans Mono" \
    --variable geometry:margin=1in \
    --variable colorlinks=true \
    --variable linkcolor=blue \
    --variable urlcolor=blue \
    --variable toccolor=blue \
    --top-level-division=section \
    --toc \
    --toc-depth=3 \
    --highlight-style=tango

if [[ ! -f "$OUT_PDF" ]]; then
    err "Build completed without error but output PDF is missing: $OUT_PDF"
fi

PDF_SIZE="$(stat -c%s "$OUT_PDF" 2>/dev/null || stat -f%z "$OUT_PDF")"

# Sanity bound: a successful build should produce >50 KB; smaller suggests
# silent xelatex failure that pandoc swallowed.
if (( PDF_SIZE < 50000 )); then
    err "Output PDF suspiciously small ($PDF_SIZE bytes). Inspect $OUT_PDF and re-run with --verbose for diagnostics."
fi

echo
echo -e "${GREEN}OK:${NC} $OUT_PDF ($PDF_SIZE bytes)"
echo
echo "Open with: xdg-open '$OUT_PDF'"
