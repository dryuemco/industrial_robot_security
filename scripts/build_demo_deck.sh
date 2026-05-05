#!/usr/bin/env bash
# scripts/build_demo_deck.sh
#
# Builds the Sub-lane C demo deck (S25 Lane 1) from Markdown to PDF via pandoc.
# Self-checks toolchain. xelatex preferred for unicode + font flexibility,
# pdflatex used as fallback.
#
# Usage:  bash scripts/build_demo_deck.sh
# Output: docs/build/georgios_s25_demo.pdf

set -euo pipefail

SOURCE="docs/georgios_s25_demo.md"
OUT_DIR="docs/build"
OUT_PDF="${OUT_DIR}/georgios_s25_demo.pdf"

# --- Source presence guard ---
if [ ! -f "$SOURCE" ]; then
  echo "ERROR: source markdown not found: $SOURCE" >&2
  exit 1
fi

# --- Toolchain guard: pandoc ---
if ! command -v pandoc >/dev/null 2>&1; then
  echo "ERROR: pandoc not installed." >&2
  echo "Install:  sudo apt install pandoc" >&2
  exit 2
fi

# --- Toolchain guard: PDF engine ---
if command -v xelatex >/dev/null 2>&1; then
  PDF_ENGINE="xelatex"
elif command -v pdflatex >/dev/null 2>&1; then
  PDF_ENGINE="pdflatex"
else
  echo "ERROR: neither xelatex nor pdflatex found." >&2
  echo "Install:  sudo apt install texlive-xetex texlive-fonts-recommended" >&2
  exit 3
fi

mkdir -p "$OUT_DIR"

echo "[build_demo_deck] pandoc engine: ${PDF_ENGINE}"
echo "[build_demo_deck] source       : ${SOURCE}"
echo "[build_demo_deck] output       : ${OUT_PDF}"

pandoc "$SOURCE" \
  --from=markdown \
  --pdf-engine="$PDF_ENGINE" \
  --resource-path=".:paper/figures" \
  --standalone \
  --output="$OUT_PDF"

echo "[build_demo_deck] OK"
ls -lh "$OUT_PDF"
