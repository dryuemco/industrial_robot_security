#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Render the ENFIELD architecture diagram from DOT source.
# Output: paper/figures/architecture.{pdf,png}
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DOT_SRC="$SCRIPT_DIR/architecture.dot"
OUT_DIR="$REPO_ROOT/paper/figures"

mkdir -p "$OUT_DIR"

if ! command -v dot >/dev/null 2>&1; then
    echo "FATAL: graphviz 'dot' not in PATH" >&2
    echo "  install with: sudo apt install graphviz" >&2
    exit 1
fi

echo "rendering PDF..."
dot -Tpdf "$DOT_SRC" -o "$OUT_DIR/architecture.pdf"

echo "rendering PNG (300 DPI)..."
dot -Tpng -Gdpi=300 "$DOT_SRC" -o "$OUT_DIR/architecture.png"

ls -la "$OUT_DIR/architecture."{pdf,png}
echo "done"
