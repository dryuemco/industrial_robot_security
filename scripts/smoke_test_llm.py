#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""ENFIELD Smoke Test — end-to-end LLM pipeline verification.

Runs a single task (T001) through the full pipeline:
  Task IR → PromptBuilder → LLM (Ollama) → CodeParser → StaticWatchdog (SM-1..7)

Usage:
    # Default: uses Ollama at localhost:11434
    python3 scripts/smoke_test_llm.py

    # Custom Ollama host (e.g. Windows host from VM)
    OLLAMA_HOST=http://192.168.1.100:11434 python3 scripts/smoke_test_llm.py

    # Specific model only
    python3 scripts/smoke_test_llm.py --model qwen2.5-coder:32b
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from pathlib import Path

# Add package paths
REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "enfield_llm"))
sys.path.insert(0, str(REPO_DIR / "enfield_tasks"))
sys.path.insert(0, str(REPO_DIR / "enfield_watchdog_static"))

from enfield_llm.factory import create_client, EXPERIMENT_MODELS
from enfield_llm.prompt_builder import PromptBuilder, PromptMode, AdversarialType
from enfield_llm.code_parser import CodeParser
from enfield_llm.base_client import ResponseStatus
from enfield_watchdog_static.watchdog import StaticWatchdog

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

TASK_FILE = REPO_DIR / "enfield_tasks" / "ir" / "tasks" / "T001_pick_place_collab.json"
LOG_DIR = REPO_DIR / "results" / "smoke_test"

# Conditions to test
CONDITIONS = [
    ("baseline", PromptMode.BASELINE, None),
    ("safety", PromptMode.SAFETY, None),
    ("adversarial_A6.6", PromptMode.ADVERSARIAL, AdversarialType.A6_6_SPEC_AMBIGUITY),
]


def run_condition(client, builder, parser, watchdog, task_ir, task_id, label, mode, adv_type):
    """Run a single experimental condition and return result dict."""
    logger.info("")
    logger.info("=" * 60)
    logger.info("CONDITION: %s", label)
    logger.info("=" * 60)

    # Build prompts
    system_prompt, user_prompt = builder.build(
        task_ir, mode=mode, adversarial_type=adv_type
    )
    logger.info("System prompt: %d chars", len(system_prompt))
    logger.info("User prompt: %d chars", len(user_prompt))

    # Call LLM
    logger.info("Calling %s...", client.model)
    response = client.generate(system_prompt, user_prompt)
    logger.info("Status: %s", response.status.value)
    logger.info("Tokens: %d in, %d out", response.prompt_tokens, response.completion_tokens)
    logger.info("Latency: %.0f ms", response.latency_ms)

    if response.status == ResponseStatus.REFUSAL:
        logger.warning("LLM refused: %s", response.refusal_reason)
        return {
            "condition": label, "model": client.model,
            "status": "refusal", "refusal_reason": response.refusal_reason,
            "safety_violations": 0, "security_violations": 0,
        }

    if response.status == ResponseStatus.ERROR:
        logger.error("API error: %s", response.raw_response[:200])
        return {
            "condition": label, "model": client.model,
            "status": "error", "error": response.raw_response[:200],
        }

    # Parse code
    parse_result = parser.extract(response.raw_response)
    logger.info("Parse: method=%s, confidence=%.1f, lines=%d",
                 parse_result.extraction_method,
                 parse_result.confidence,
                 parse_result.line_count)

    if not parse_result.code:
        logger.warning("No code extracted from response")
        return {"condition": label, "model": client.model, "status": "parse_failure"}

    # Show first 10 lines
    code_lines = parse_result.code.split("\n")
    logger.info("Generated code (%d lines):", len(code_lines))
    for line in code_lines[:10]:
        logger.info("  | %s", line)
    if len(code_lines) > 10:
        logger.info("  | ... (%d more lines)", len(code_lines) - 10)

    # Watchdog security analysis
    sec_report = watchdog.analyze_code(parse_result.code, task_id=task_id)
    logger.info("Security analysis: %d checks, %d violations",
                 sec_report.checks_run, sec_report.violation_count)

    for v in sec_report.violations:
        logger.info("  [%s] %s (severity=%.2f)", v.attack_type, v.description, v.severity)

    # Save generated code
    code_file = LOG_DIR / f"{task_id}_{client.model.replace(':', '_')}_{label}_code.urscript"
    code_file.parent.mkdir(parents=True, exist_ok=True)
    code_file.write_text(parse_result.code)
    logger.info("Code saved: %s", code_file)

    return {
        "condition": label, "model": client.model,
        "status": "success",
        "code_lines": parse_result.line_count,
        "has_motion": parse_result.has_motion_command,
        "has_safety_check": parse_result.has_safety_check,
        "security_violations": sec_report.violation_count,
        "violation_types": [v.attack_type for v in sec_report.violations],
        "tokens_in": response.prompt_tokens,
        "tokens_out": response.completion_tokens,
        "latency_ms": round(response.latency_ms),
    }


def main() -> int:
    """Run smoke test and return exit code."""
    arg_parser = argparse.ArgumentParser(description="ENFIELD LLM Smoke Test")
    arg_parser.add_argument("--model", type=str, default=None,
                            help="Specific model to test (default: all 3)")
    arg_parser.add_argument("--provider", type=str, default="ollama",
                            help="Provider (default: ollama)")
    args = arg_parser.parse_args()

    # Check Ollama connectivity
    ollama_host = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
    logger.info("Ollama host: %s", ollama_host)

    # Load task
    logger.info("Loading task: %s", TASK_FILE.name)
    with open(TASK_FILE) as f:
        task_ir = json.load(f)

    task_id = task_ir["task"]["id"]
    task_name = task_ir["task"]["name"]
    logger.info("Task: %s — %s", task_id, task_name)

    # Determine which models to test
    if args.model:
        models_to_test = {"custom": args.model}
    else:
        models_to_test = dict(EXPERIMENT_MODELS)

    builder = PromptBuilder()
    parser = CodeParser()
    watchdog = StaticWatchdog()

    all_results = []
    any_success = False

    for model_name, model_id in models_to_test.items():
        logger.info("\n" + "=" * 60)
        logger.info("MODEL: %s (%s)", model_name, model_id)
        logger.info("=" * 60)

        client = create_client(args.provider, model=model_id, log_dir=LOG_DIR)

        for label, mode, adv_type in CONDITIONS:
            result = run_condition(
                client, builder, parser, watchdog,
                task_ir, task_id, label, mode, adv_type
            )
            all_results.append(result)
            if result["status"] == "success":
                any_success = True

    # ---------------------------------------------------------------------------
    # Summary
    # ---------------------------------------------------------------------------
    logger.info("\n" + "=" * 60)
    logger.info("SMOKE TEST SUMMARY")
    logger.info("=" * 60)

    for r in all_results:
        model = r.get("model", "?")
        if r["status"] == "success":
            logger.info("  ✓ %s / %s: %d lines, %d violations, %dms",
                         model, r["condition"],
                         r["code_lines"], r["security_violations"],
                         r["latency_ms"])
        elif r["status"] == "refusal":
            logger.info("  ⊘ %s / %s: REFUSED (%s)",
                         model, r["condition"], r.get("refusal_reason", "?"))
        else:
            logger.info("  ✗ %s / %s: %s",
                         model, r["condition"], r["status"])

    # Save results
    results_file = LOG_DIR / "smoke_test_results.json"
    results_file.parent.mkdir(parents=True, exist_ok=True)
    with open(results_file, "w") as f:
        json.dump(all_results, f, indent=2)
    logger.info("\nResults: %s", results_file)

    if any_success:
        logger.info("\n✓ SMOKE TEST PASSED — pipeline works end-to-end")
        return 0
    else:
        logger.error("\n✗ SMOKE TEST FAILED — no successful generations")
        return 1


if __name__ == "__main__":
    sys.exit(main())
