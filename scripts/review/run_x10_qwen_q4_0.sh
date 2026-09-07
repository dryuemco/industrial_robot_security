#!/usr/bin/env bash
# X10: re-run the E1 baseline/safety experiment for Qwen2.5-Coder-32B at Q4_0
# (the confirmatory run used Q4_K_M) on the experiment-time task snapshot, so
# that the cross-model heterogeneity result can be read against a same-model,
# different-quantization control. 15 tasks x 2 conditions x 3 reps = 90 calls.
#
# Requires a local Ollama with the model pulled:
#   ollama pull qwen2.5-coder:32b-instruct-q4_0
# Usage: scripts/review/run_x10_qwen_q4_0.sh [output_dir]
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${1:-$REPO/results/X10_qwen_q4_0}"
export OLLAMA_HOST="${OLLAMA_HOST:-http://localhost:11434}"
export ENFIELD_TASKS_DIR="$REPO/paper/data/tasks_as_run"
export PYTHONPATH="$REPO/enfield_llm:$REPO/enfield_tasks:$REPO/enfield_watchdog_static:$REPO/enfield_translators"
cd "$REPO"
python3 scripts/llm_experiment_runner.py \
  --experiment E1 \
  --provider ollama \
  --models qwen2.5-coder:32b-instruct-q4_0 \
  --reps 3 \
  --max-tokens 4096 \
  --temperature 0.0 \
  --output "$OUT"
# Record the served digest and quantization for the replication record.
curl -s "$OLLAMA_HOST/api/tags" | python3 -c '
import sys, json
for m in json.load(sys.stdin)["models"]:
    if m["name"].startswith("qwen2.5-coder:32b"):
        print(m["name"], m["details"].get("quantization_level"), m["digest"])' > "$OUT/model_digest.txt"
cat "$OUT/model_digest.txt"
python3 scripts/review/quantization_compare.py \
  --ref results/E1_full/e1_results.csv --ref-model qwen2.5-coder:32b \
  --new "$OUT/e1_results.csv" --new-model qwen2.5-coder:32b-instruct-q4_0 \
  --out-dir paper/tables
