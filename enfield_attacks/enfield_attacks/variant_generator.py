# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Attack variant generator — produces A1–A8 adversarial variants from baseline tasks."""

from __future__ import annotations

import copy
import json
import logging
from pathlib import Path
from typing import Any

from enfield_attacks.mutations import MUTATIONS, Mutation, MutationResult

logger = logging.getLogger(__name__)


def generate_variant_id(task_id: str, attack_id: str) -> str:
    """Build a variant ID, e.g. T001_A1."""
    return f"{task_id}_{attack_id}"


def generate_variant(
    baseline: dict[str, Any],
    attack_id: str,
    seed: int = 42,
    **mutation_kwargs: Any,
) -> tuple[dict[str, Any], MutationResult]:
    """Apply a single A1–A8 mutation to a baseline task.

    Args:
        baseline: Parsed baseline task dict (will NOT be modified).
        attack_id: One of "A1"–"A8".
        seed: Random seed for deterministic mutations.
        **mutation_kwargs: Extra kwargs forwarded to the Mutation constructor.

    Returns:
        (mutated_task, result) tuple.

    Raises:
        KeyError: If attack_id is not in A1–A8.
    """
    mutation_cls = MUTATIONS[attack_id]
    mutation = mutation_cls(seed=seed, **mutation_kwargs)

    task_copy = copy.deepcopy(baseline)
    mutated, result = mutation.apply(task_copy)

    # Stamp variant metadata into the task
    task_meta = mutated.get("task", {})
    original_id = task_meta.get("id", "T000")
    variant_id = generate_variant_id(original_id, attack_id)

    task_meta["id"] = variant_id
    task_meta["name"] = f"[{attack_id}] {task_meta.get('name', '')}"
    task_meta["description"] = (
        f"ADVERSARIAL VARIANT ({attack_id}): {result.description}. "
        f"Original: {task_meta.get('description', '')}"
    )

    # Add attack metadata tag
    tags = task_meta.get("tags", [])
    tags.extend(["adversarial", attack_id.lower()])
    task_meta["tags"] = tags

    return mutated, result


def generate_all_variants(
    baseline: dict[str, Any],
    seed: int = 42,
    attack_ids: list[str] | None = None,
) -> list[tuple[dict[str, Any], MutationResult]]:
    """Generate variants for all (or specified) attack types.

    Args:
        baseline: Parsed baseline task dict.
        seed: Deterministic seed.
        attack_ids: Subset of attacks to generate (default: all A1–A8).

    Returns:
        List of (mutated_task, result) tuples.
    """
    ids = attack_ids or list(MUTATIONS.keys())
    results = []
    for aid in ids:
        try:
            variant, result = generate_variant(baseline, aid, seed=seed)
            results.append((variant, result))
            logger.info(
                "Generated %s variant for %s: %s",
                aid,
                baseline.get("task", {}).get("id", "?"),
                result.description,
            )
        except Exception as exc:
            logger.error("Failed to generate %s variant: %s", aid, exc)
    return results


def batch_generate(
    tasks_dir: str | Path,
    output_dir: str | Path,
    seed: int = 42,
    attack_ids: list[str] | None = None,
) -> list[dict[str, Any]]:
    """Generate A1–A8 variants for all baseline tasks in a directory.

    Args:
        tasks_dir: Directory containing baseline T00*.json files.
        output_dir: Directory to write variant JSON files.
        seed: Deterministic seed.
        attack_ids: Subset of attacks (default: all).

    Returns:
        List of manifest entries (dicts with variant metadata).
    """
    tasks_path = Path(tasks_dir)
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    baseline_files = sorted(tasks_path.glob("T[0-9][0-9][0-9]_*.json"))
    manifest: list[dict[str, Any]] = []
    ids = attack_ids or list(MUTATIONS.keys())

    for bf in baseline_files:
        with open(bf) as f:
            baseline = json.load(f)

        task_id = baseline["task"]["id"]
        logger.info("Processing baseline: %s (%s)", task_id, bf.name)

        for aid in ids:
            try:
                variant, result = generate_variant(baseline, aid, seed=seed)

                # Relax schema: variant IDs like T001_A1 don't match ^T[0-9]{3}$
                # We accept this — variants are adversarial, not schema-compliant baselines.

                variant_id = variant["task"]["id"]
                filename = f"{variant_id}_{bf.stem.split('_', 1)[1]}.json"
                out_file = out_path / filename

                with open(out_file, "w") as f:
                    json.dump(variant, f, indent=2, ensure_ascii=False)

                entry = {
                    "variant_id": variant_id,
                    "baseline_id": task_id,
                    "attack_type": aid,
                    "iso_clause": result.iso_clause,
                    "description": result.description,
                    "severity": result.severity_estimate,
                    "mutated_fields": result.mutated_fields,
                    "file": str(out_file.name),
                }
                manifest.append(entry)
                logger.info("  → %s written", filename)

            except Exception as exc:
                logger.error("  ✗ %s/%s failed: %s", task_id, aid, exc)

    # Write manifest
    manifest_file = out_path / "manifest.json"
    with open(manifest_file, "w") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    logger.info("Manifest written: %s (%d variants)", manifest_file, len(manifest))

    return manifest


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """CLI: generate attack variants from baseline tasks."""
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(
        description="Generate A1–A8 adversarial variants from baseline Task IR files."
    )
    parser.add_argument(
        "--tasks-dir",
        type=str,
        default="ir/tasks",
        help="Directory containing baseline T00*.json files.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="generated/variants",
        help="Output directory for variant JSON files.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for deterministic generation.",
    )
    parser.add_argument(
        "--attacks",
        type=str,
        nargs="+",
        default=None,
        help="Subset of attack IDs (e.g. A1 A3 A5). Default: all.",
    )

    args = parser.parse_args()
    manifest = batch_generate(
        tasks_dir=args.tasks_dir,
        output_dir=args.output_dir,
        seed=args.seed,
        attack_ids=args.attacks,
    )
    print(f"\nDone: {len(manifest)} variants generated.")


if __name__ == "__main__":
    main()
