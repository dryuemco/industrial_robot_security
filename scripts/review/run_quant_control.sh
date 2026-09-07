#!/usr/bin/env bash
# Quantization control (X10 family): re-run the E1 baseline/safety experiment
# for one model at a different quantization level than the confirmatory run,
# on the experiment-time task snapshot, then compare with the confirmatory
# rows of the same model. 15 tasks x 2 conditions x 3 reps = 90 calls.
#
# Usage: scripts/review/run_quant_control.sh <ollama_tag> <ref_model_id> <out_dir> <table_suffix>
#   e.g. scripts/review/run_quant_control.sh qwen2.5-coder:32b-instruct-q4_0 \
#            qwen2.5-coder:32b results/X10_qwen_q4_0 qwen
#        scripts/review/run_quant_control.sh deepseek-coder-v2:16b-lite-instruct-q4_K_M \
#            deepseek-coder-v2:16b results/X10_deepseek_q4_K_M deepseek
#        scripts/review/run_quant_control.sh codellama:34b-instruct-q4_K_M \
#            codellama:34b results/X10_codellama_q4_K_M codellama
set -euo pipefail
TAG="$1"; REF="$2"; OUT="$3"; SUFFIX="$4"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
export OLLAMA_HOST="${OLLAMA_HOST:-http://localhost:11434}"
export ENFIELD_TASKS_DIR="$REPO/paper/data/tasks_as_run"
export PYTHONPATH="$REPO/enfield_llm:$REPO/enfield_tasks:$REPO/enfield_watchdog_static:$REPO/enfield_translators"
cd "$REPO"
python3 scripts/llm_experiment_runner.py \
  --experiment E1 --provider ollama --models "$TAG" --reps 3 \
  --max-tokens 4096 --temperature 0.0 --output "$OUT"
curl -s "$OLLAMA_HOST/api/tags" | python3 -c '
import sys, json
tag = sys.argv[1]
for m in json.load(sys.stdin)["models"]:
    if m["name"] == tag:
        print(m["name"], m["details"].get("quantization_level"), m["digest"])' "$TAG" > "$OUT/model_digest.txt"
cat "$OUT/model_digest.txt"
# Archive the small artefacts next to the paper (results/ is git-ignored).
mkdir -p "paper/data/$(basename "$OUT")"
cp "$OUT/e1_results.csv" "$OUT/e1_summary.json" "$OUT/model_digest.txt" "paper/data/$(basename "$OUT")/"
# Confirmatory quantization label for the reference model, for the table caption.
case "$REF" in
  qwen2.5-coder:32b) REF_LABEL='Q4\_K\_M'; NEW_LABEL='Q4\_0' ;;
  *)                 REF_LABEL='Q4\_0';   NEW_LABEL='Q4\_K\_M' ;;
esac
python3 scripts/review/quantization_compare.py \
  --ref results/E1_full/e1_results.csv --ref-model "$REF" --ref-label "$REF_LABEL" \
  --new "$OUT/e1_results.csv" --new-model "$TAG" --new-label "$NEW_LABEL" \
  --out-dir paper/tables --out-name "quantization_compare_$SUFFIX"
