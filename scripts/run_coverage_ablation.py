# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""ICSE coverage-ablation experiment -- the headline coverage-ablation experiment of the ICSE study.

Runs the project's existing watchdog-in-loop (E3) repair logic once per
(model, task, coverage-variant), but with two watchdogs:

    feedback oracle  = the VARIANT (exposes a subset of the Tier-S classes)
                       -> drives the loop's stop condition and feedback.
    evaluation oracle = the FULL watchdog
                       -> scores every emitted program against ALL classes.

For each iteration it emits one JSONL record (schema documented in
``migration_index_analyzer.py``) capturing what the loop SAW (subset) and what
the full evaluation found. The analyzer then tests whether residual mass
migrates into the hidden classes beyond a no-migration null.

This script ADDS no detector and MODIFIES no tested code: it imports
``run_single_call`` / ``build_feedback`` / ``load_tasks`` / ``create_client``
from ``llm_experiment_runner`` and reuses them verbatim.

Mock-smoke (no Ollama / HPC needed):
    python scripts/run_coverage_ablation.py --provider mock --tasks T001-T001 \
        --ablation-set A1 A2 A5 --reps 1 --max-retries 1 --out /tmp/h2_mock.jsonl
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_DIR / "scripts"))

import llm_experiment_runner as R  # noqa: E402  (does its own sys.path setup)
from coverage_variant_driver import (  # noqa: E402
    CoverageVariant, build_variants, full_watchdog,
)


def _family(variant_name: str) -> str:
    return variant_name.split("_", 1)[0]  # full | loo | nested


def make_record(
    row: dict[str, Any],
    *,
    variant: CoverageVariant,
    ablation_set: list[str],
    task_ir: dict,
    model_id: str,
    rep: int,
    iteration: int,
    eval_wd,
) -> dict[str, Any]:
    """Build one migration record by scoring the iteration's code with the full
    (evaluation) oracle. Non-success rows have no emitted code -> parseable=False
    (exit-via-invalidation / refusal candidate; completeness-gap escape)."""
    parseable = (row.get("status") == "success") and ("_generated_code" in row)
    if parseable:
        report = eval_wd.analyze_combined(task_ir, row["_generated_code"])
        full_classes = sorted({v.attack_type for v in report.violations})
    else:
        full_classes = []
    return {
        "variant": variant.name,
        "family": _family(variant.name),
        "ablation_set": list(ablation_set),
        "exposed": list(variant.exposed),
        "hidden": list(variant.hidden),
        "model": model_id,
        "task": task_ir["task"]["id"],
        "rep": rep,
        "iteration": iteration,
        "is_final": False,
        "parseable": parseable,
        "status": row.get("status"),
        "feedback_violation_count": row.get("total_violations", 0),
        "full_violation_classes": full_classes,
    }


def run_cell(
    client, builder, parser, fb_wd, eval_wd, code_dir,
    *, variant, ablation_set, task_ir, model_id, rep, max_retries, feedback_mode,
) -> list[dict[str, Any]]:
    """One (model, task, variant, rep) repair trajectory -> list of records."""
    capture = feedback_mode == "rich"
    recs: list[dict[str, Any]] = []

    row = R.run_single_call(
        client, builder, parser, fb_wd, task_ir,
        R.PromptMode.BASELINE, None, code_dir,
        experiment="H2", rep=rep, retry=0,
        capture_violations=capture, attach_code=True,
    )
    vdetails = row.pop("_violation_details", None)
    recs.append(make_record(
        row, variant=variant, ablation_set=ablation_set, task_ir=task_ir,
        model_id=model_id, rep=rep, iteration=0, eval_wd=eval_wd,
    ))

    for retry in range(1, max_retries + 1):
        # NB: total_violations here is the VARIANT (subset) verdict -- the loop
        # declares victory when the EXPOSED classes are satisfied, which is the
        # mechanism under study, not a bug.
        if row.get("total_violations", 0) == 0 or row.get("status") != "success":
            break
        feedback = R.build_feedback(row, vdetails, feedback_mode)
        row = R.run_single_call(
            client, builder, parser, fb_wd, task_ir,
            R.PromptMode.BASELINE, None, code_dir,
            experiment="H2", rep=rep, retry=retry, feedback=feedback,
            capture_violations=capture, attach_code=True,
        )
        vdetails = row.pop("_violation_details", None)
        recs.append(make_record(
            row, variant=variant, ablation_set=ablation_set, task_ir=task_ir,
            model_id=model_id, rep=rep, iteration=retry, eval_wd=eval_wd,
        ))

    recs[-1]["is_final"] = True
    return recs


def run_ablation(
    ablation_set: list[str],
    *,
    families: list[str],
    tasks_filter: str | None,
    reps: int,
    max_retries: int,
    provider: str,
    models: list[str] | None,
    seed: int | None,
    max_tokens: int,
    timeout: float,
    feedback_mode: str,
    out_jsonl: Path,
) -> list[dict[str, Any]]:
    variants = build_variants(ablation_set, families)
    eval_wd = full_watchdog()
    tasks = R.load_tasks(tasks_filter)
    # Guard: the default model table (EXPERIMENT_MODELS) holds Ollama ids. For
    # idun / cloud gateways those ids do not exist -> require explicit --models.
    if provider not in ("ollama", "mock") and not models:
        raise SystemExit(
            f"--models is required for provider '{provider}'. The default model "
            f"table holds Ollama ids that are absent on this gateway. "
            f"e.g. --models zai-org/GLM-4.7-FP8 <second-model-id>"
        )
    model_table = R._models_for_provider(provider, models)
    builder, parser = R.PromptBuilder(), R.CodeParser()
    out_dir = out_jsonl.parent
    code_dir = out_dir / "code"

    print(f"[ICSE-ablation] models={list(model_table)} tasks={len(tasks)} "
          f"variants={[v.name for v in variants]} reps={reps} "
          f"max_retries={max_retries} feedback={feedback_mode}")

    all_records: list[dict[str, Any]] = []
    for model_name, model_id in model_table.items():
        client = R.create_client(
            provider, model=model_id, log_dir=out_dir / "logs",
            max_tokens=max_tokens, timeout=timeout, seed=seed,
        )
        for task_ir in tasks:
            for variant in variants:
                fb_wd = variant.feedback_watchdog()
                for rep in range(1, reps + 1):
                    all_records.extend(run_cell(
                        client, builder, parser, fb_wd, eval_wd, code_dir,
                        variant=variant, ablation_set=ablation_set,
                        task_ir=task_ir, model_id=model_id, rep=rep,
                        max_retries=max_retries, feedback_mode=feedback_mode,
                    ))

    out_jsonl.parent.mkdir(parents=True, exist_ok=True)
    with open(out_jsonl, "w") as f:
        for rec in all_records:
            f.write(json.dumps(rec) + "\n")
    print(f"[ICSE-ablation] wrote {len(all_records)} records -> {out_jsonl}")
    return all_records


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ablation-set", nargs="+", required=True,
                   help="Pinned Tier-S rule-class IDs, e.g. A1 A2 A5")
    p.add_argument("--families", nargs="+", default=["full", "loo", "nested"],
                   choices=["full", "loo", "nested"])
    p.add_argument("--tasks", default=None, help="e.g. T001-T015 (default: all)")
    p.add_argument("--reps", type=int, default=3)
    p.add_argument("--max-retries", type=int, default=3)
    p.add_argument("--provider", default="ollama",
                   choices=["ollama", "mock", "anthropic", "openai", "xai", "idun"],
                   help="idun = NTNU Idun HPC gateway (needs IDUN_BASE_URL + "
                        "IDUN_API_KEY, NTNU VPN, and explicit --models)")
    p.add_argument("--models", nargs="+", default=None)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--max-tokens", type=int, default=4096)
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--feedback-mode", default="minimal",
                   choices=["minimal", "rich"])
    p.add_argument("--out", type=Path, required=True,
                   help="output migration JSONL path")
    args = p.parse_args()

    run_ablation(
        args.ablation_set,
        families=args.families,
        tasks_filter=args.tasks,
        reps=args.reps,
        max_retries=args.max_retries,
        provider=args.provider,
        models=args.models,
        seed=args.seed,
        max_tokens=args.max_tokens,
        timeout=args.timeout,
        feedback_mode=args.feedback_mode,
        out_jsonl=args.out,
    )


if __name__ == "__main__":
    main()
