#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""ENFIELD Smoke Test — end-to-end LLM pipeline verification.

Runs a single task (T001) through the full pipeline:
  Task IR → PromptBuilder → LLM API → CodeParser → StaticWatchdog (SM-1..7)

Usage:
    export OPENAI_API_KEY="sk-..."
    python3 scripts/smoke_test_llm.py
"""

from __future__ import annotations

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

from enfield_llm.factory import create_client
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


def main() -> int:
    """Run smoke test and return exit code."""

    # Check API key
    provider = "openai"
    env_var = "OPENAI_API_KEY"

    if not os.environ.get(env_var):
        logger.error("%s not set. Export it and retry.", env_var)
        return 1

    # Load task
    logger.info("Loading task: %s", TASK_FILE.name)
    with open(TASK_FILE) as f:
        task_ir = json.load(f)

    task_id = task_ir["task"]["id"]
    task_name = task_ir["task"]["name"]
    logger.info("Task: %s — %s", task_id, task_name)

    # Initialize components
    client = create_client(provider, log_dir=LOG_DIR)
    builder = PromptBuilder()
    parser = CodeParser()
    watchdog = StaticWatchdog()

    logger.info("LLM: %s (provider: %s)", client.model, provider)
    logger.info("Log dir: %s", LOG_DIR)

    # ---------------------------------------------------------------------------
    # Run each condition
    # ---------------------------------------------------------------------------
    results = []

    for label, mode, adv_type in CONDITIONS:
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
            results.append({
                "condition": label,
                "status": "refusal",
                "refusal_reason": response.refusal_reason,
                "safety_violations": 0,
                "security_violations": 0,
            })
            continue

        if response.status == ResponseStatus.ERROR:
            logger.error("API error: %s", response.raw_response[:200])
            results.append({
                "condition": label,
                "status": "error",
                "error": response.raw_response[:200],
            })
            continue

        # Parse code
        parse_result = parser.extract(response.raw_response)
        logger.info("Parse: method=%s, confidence=%.1f, lines=%d",
                     parse_result.extraction_method,
                     parse_result.confidence,
                     parse_result.line_count)
        logger.info("Has motion: %s, Has safety: %s",
                     parse_result.has_motion_command,
                     parse_result.has_safety_check)

        if not parse_result.code:
            logger.warning("No code extracted from response")
            results.append({
                "condition": label,
                "status": "parse_failure",
            })
            continue

        # Show first 10 lines of generated code
        code_lines = parse_result.code.split("\n")
        logger.info("Generated code (%d lines):", len(code_lines))
        for line in code_lines[:10]:
            logger.info("  | %s", line)
        if len(code_lines) > 10:
            logger.info("  | ... (%d more lines)", len(code_lines) - 10)

        # Watchdog security analysis
        sec_report = watchdog.analyze_code(
            parse_result.code, task_id=task_id
        )
        logger.info("Security analysis: %d checks, %d violations",
                     sec_report.checks_run, sec_report.violation_count)

        for v in sec_report.violations:
            logger.info("  [%s] %s (severity=%.2f)", v.attack_type, v.description, v.severity)

        results.append({
            "condition": label,
            "status": "success",
            "code_lines": parse_result.line_count,
            "has_motion": parse_result.has_motion_command,
            "has_safety_check": parse_result.has_safety_check,
            "security_violations": sec_report.violation_count,
            "violation_types": [v.attack_type for v in sec_report.violations],
            "tokens_in": response.prompt_tokens,
            "tokens_out": response.completion_tokens,
            "latency_ms": round(response.latency_ms),
        })

        # Save generated code
        code_file = LOG_DIR / f"{task_id}_{label}_code.urscript"
        code_file.parent.mkdir(parents=True, exist_ok=True)
        code_file.write_text(parse_result.code)
        logger.info("Code saved: %s", code_file)

    # ---------------------------------------------------------------------------
    # Summary
    # ---------------------------------------------------------------------------
    logger.info("")
    logger.info("=" * 60)
    logger.info("SMOKE TEST SUMMARY")
    logger.info("=" * 60)

    all_ok = True
    for r in results:
        status_icon = "✓" if r["status"] == "success" else "✗"
        if r["status"] == "success":
            logger.info("  %s %s: %d lines, %d security violations, %dms",
                         status_icon, r["condition"],
                         r["code_lines"], r["security_violations"],
                         r["latency_ms"])
        elif r["status"] == "refusal":
            logger.info("  %s %s: REFUSED (%s)",
                         status_icon, r["condition"], r.get("refusal_reason", "?"))
        else:
            logger.info("  %s %s: %s", status_icon, r["condition"], r["status"])
            all_ok = False

    # Save results JSON
    results_file = LOG_DIR / "smoke_test_results.json"
    results_file.parent.mkdir(parents=True, exist_ok=True)
    with open(results_file, "w") as f:
        json.dump(results, f, indent=2)
    logger.info("\nResults saved: %s", results_file)

    if all_ok:
        logger.info("\n✓ SMOKE TEST PASSED — pipeline works end-to-end")
        return 0
    else:
        logger.error("\n✗ SMOKE TEST FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
