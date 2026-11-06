#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Copy the generation runs reported in the paper from results/ (outside version control)
into data/, masking local file paths, user names and the inference endpoint.

Runs copied (results/<src> -> data/<dst>):
  smoke_test                       the single-task probe of VI-A (three local models, T001)
  E1_frontier, E3_frontier         the frontier runs of VI-E (Table 7 and the repair loop)
  E1_frontier_probe, E1_frontier_probe2, E3_frontier_probe
                                   the preliminary frontier probes run before them
  E3_rich_probe, E3_rich_mini      the enriched-feedback probe
  exploratory/temp_robustness      the two sampling temperatures
  exploratory/e4_template_probe    the control probe of the constraint-representation analysis
  exploratory/e4_canary, e4_smoke  its preliminary checks
  exploratory/e5_repr_fragility    the representation probe
  X10_*                            the quantization controls of VI-D (code and logs)

Masking replaces only the matched substring: a home directory with <HOME>, a user name with
<USER>, the inference endpoint with <IDUN_ENDPOINT>. A match inside a prompt sent to a model
or inside a model response (the system_prompt, user_prompt and response fields of the JSONL
logs, and the stored .urscript programs) is never masked; the script stops instead. A key or
token pattern anywhere also stops it. The model-written address 192.168.1.100 in one frontier
program is part of a model response and is kept.

  python3 s13_import_runs.py            copy and mask (refuses to overwrite an existing target)
  python3 s13_import_runs.py --check    recompute the masked bytes from results/ and compare with data/

Outputs: out/r3/import_masking.csv (one row per file that was changed: counts per mask) and
out/r3/import_manifest.csv (every copied file with its source and target SHA-256).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import pandas as pd

from common import REPO, out

RUNS = [("smoke_test", "exploratory/smoke_test"),
        ("E1_frontier", "exploratory/E1_frontier"), ("E1_frontier_probe", "exploratory/E1_frontier_probe"),
        ("E1_frontier_probe2", "exploratory/E1_frontier_probe2"), ("E3_frontier", "exploratory/E3_frontier"),
        ("E3_frontier_probe", "exploratory/E3_frontier_probe"), ("E3_rich_probe", "exploratory/E3_rich_probe"),
        ("E3_rich_mini", "exploratory/E3_rich_mini"), ("exploratory/temp_robustness", "exploratory/temp_robustness"),
        ("exploratory/e4_template_probe", "exploratory/e4_template_probe"),
        ("exploratory/e4_canary", "exploratory/e4_canary"), ("exploratory/e4_smoke", "exploratory/e4_smoke"),
        ("exploratory/e5_repr_fragility", "exploratory/e5_repr_fragility"),
        ("X10_codellama_q4_K_M", "quantization_controls/X10_codellama_q4_K_M"),
        ("X10_codellama_q4_K_M_rerun", "quantization_controls/X10_codellama_q4_K_M_rerun"),
        ("X10_deepseek_q4_K_M", "quantization_controls/X10_deepseek_q4_K_M"),
        ("X10_deepseek_q4_K_M_rerun", "quantization_controls/X10_deepseek_q4_K_M_rerun"),
        ("X10_qwen_q4_0", "quantization_controls/X10_qwen_q4_0")]

# Order matters: the home directory is masked before the bare user name.
MASKS = [("HOME", re.compile(r"/Users/yuemco|/home/yunus"), "<HOME>"),
         ("USER", re.compile(r"\b(?:yuemco|yunus)\b"), "<USER>"),
         ("IDUN_ENDPOINT", re.compile(r"https?://llm\.hpc\.ntnu\.no(?:/v1)?"), "<IDUN_ENDPOINT>")]
STOP = re.compile(r"(?i)\b(sk-[A-Za-z0-9_-]{16,}|hf_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|ghp_[A-Za-z0-9]{20,}|"
                  r"xox[bp]-[A-Za-z0-9-]+)|\"?(api[_-]?key|access[_-]?token|auth[_-]?token|password|"
                  r"authorization|bearer)\"?\s*[:=]")
MODEL_FIELDS = ("system_prompt", "user_prompt", "response")
SKIP_NAMES = {".DS_Store"}  # file-browser metadata, not run output


class Stop(Exception):
    pass


def _mask_text(text: str, where: str) -> tuple[str, dict]:
    counts = {}
    for name, rx, repl in MASKS:
        text, n = rx.subn(repl, text)
        if n:
            counts[name] = counts.get(name, 0) + n
    return text, counts


def _any_mask(text: str) -> bool:
    return any(rx.search(text) for _, rx, _ in MASKS)


def masked(path: Path, rel: str) -> tuple[bytes, dict]:
    raw = path.read_bytes()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw, {}
    if STOP.search(text):
        raise Stop(f"key or token pattern in {rel}")
    if not _any_mask(text):
        return raw, {}
    if path.suffix == ".urscript":
        raise Stop(f"mask pattern inside a stored model program: {rel}")
    if path.suffix == ".jsonl":
        lines, counts = [], {}
        for i, line in enumerate(text.split("\n")):
            if not line.strip() or not _any_mask(line):
                lines.append(line)
                continue
            rec = json.loads(line)
            for k in MODEL_FIELDS:
                v = rec.get(k)
                if v is not None and _any_mask(v if isinstance(v, str) else json.dumps(v)):
                    raise Stop(f"mask pattern inside the model field '{k}' of {rel}, line {i + 1}")
            new, c = _mask_text(line, rel)
            for k2, n in c.items():
                counts[k2] = counts.get(k2, 0) + n
            lines.append(new)
        return "\n".join(lines).encode("utf-8"), counts
    if path.suffix == ".log":
        # client logger lines only ("HH:MM:SS LEVEL message")
        for i, line in enumerate(text.split("\n")):
            if _any_mask(line) and not re.match(r"^\d{2}:\d{2}:\d{2} [A-Z]+ ", line):
                raise Stop(f"mask pattern outside a logger line in {rel}, line {i + 1}")
        new, counts = _mask_text(text, rel)
        return new.encode("utf-8"), counts
    raise Stop(f"mask pattern in a file type without a masking rule: {rel}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    src_root, dst_root = REPO / "results", REPO / "data"
    if not all((src_root / s).is_dir() for s, _ in RUNS):
        print("SKIP s13: results/ run directories are not in this checkout (outside version control); "
              "data/ and the committed out/r3/import_*.csv are left as they are")
        return
    manifest, masking, plan = [], [], []
    try:
        for src, dst in RUNS:
            for f in sorted((src_root / src).rglob("*")):
                if not f.is_file() or f.name in SKIP_NAMES:
                    continue
                rel_src = f.relative_to(REPO).as_posix()
                target = dst_root / dst / f.relative_to(src_root / src)
                data, counts = masked(f, rel_src)
                plan.append((target, data))
                manifest.append({"source": rel_src, "target": target.relative_to(REPO).as_posix(),
                                 "bytes": len(data), "sha256_source": hashlib.sha256(f.read_bytes()).hexdigest(),
                                 "sha256_target": hashlib.sha256(data).hexdigest(), "masked": bool(counts)})
                if counts:
                    masking.append({"target": target.relative_to(REPO).as_posix(), **counts})
    except Stop as e:
        sys.exit(f"STOP: {e}; nothing was written")
    if args.check:
        bad = [t for t, d in plan if not t.is_file() or t.read_bytes() != d]
        extra = sorted({p for _, dst in RUNS for p in (dst_root / dst).rglob("*")
                        if p.is_file() and p.name not in SKIP_NAMES} - {t for t, _ in plan})
        print(f"check: {len(plan)} files, {len(bad)} differ or are missing, {len(extra)} extra in data/")
        if bad or extra:
            sys.exit(1)
        return
    existing = [t for t, _ in plan if t.exists()]
    if existing:
        sys.exit(f"STOP: {len(existing)} target files already exist (first: {existing[0]}); use --check")
    for t, d in plan:
        t.parent.mkdir(parents=True, exist_ok=True)
        t.write_bytes(d)
    pd.DataFrame(manifest).to_csv(out("r3", "import_manifest.csv"), index=False)
    cols = ["target"] + [n for n, _, _ in MASKS]
    pd.DataFrame(masking).reindex(columns=cols).fillna(0).astype({n: int for n, _, _ in MASKS}).to_csv(
        out("r3", "import_masking.csv"), index=False)
    print(f"copied {len(plan)} files into data/; {len(masking)} masked")


if __name__ == "__main__":
    main()
