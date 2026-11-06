#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Verbatim excerpts of OSF Amendment 1 for the local mirror docs/OSF_PREREGISTRATION.md (V6).

Reads the cached schema responses in out/osf/ (no network) and writes
out/r3/osf_amendment1_excerpt.md: the Amendment 1 text that defines the cross-model test,
the prompt conditions, the cell-level aggregation rule and the treatment of invalid
pseudo-code, copied verbatim (HTML entities unescaped). Each excerpt is also checked
against the three later versions of the registration.
"""
from __future__ import annotations

import html
import json
import re

from common import out

AMENDMENT_1 = "69d3c5c421a5eb766f7a850e"
LATER = ["6a02e67cf944b975e74bbdbd", "6a076690b12b31eaddaaca6d", "6ab196ba288fe69e4d07a19f"]
# (field, start pattern, end pattern or None for the end of the line)
EXCERPTS = [
    ("344-58", r"Prompt Condition \(within-subject", r"\n\s*\nWatchdog Feedback"),
    ("344-62", r"LLM Model Identifier:", r"\n\s*\n\s*\nPrompt Condition:"),
    ("344-62", r"Prompt Condition:\n", r"\n\s*\nThe deterministic-pipeline"),
    ("344-66", r"Cross-Model Cochran's Q \(exploratory\)", r"\n\s*\nNo other composite"),
    ("344-71", r"--- Cross-Model Comparison", r"\n\s*\n--- Assumptions ---"),
    ("344-75", r"Transformations — extension for LLM experiments", r"\n\s*\nNo log transformations, standardization, winsorization, or outlier adjustments are applied to LLM data"),
    ("344-77", r"--- Sensitivity Analyses \(Pre-specified, Reported", r"\n\s*\n--- Summary of Corrections \(LLM Side\) ---"),
    ("344-79", r"Invalid pseudo-code is not an exclusion\.", None),
]


def responses(sid: str) -> dict:
    return json.loads(out("osf", f"schema_response_{sid}.json").read_text())["data"]["attributes"]


def between(text: str, start: str, end: str | None) -> str:
    i = re.search(start, text).start()
    if end is None:
        j = text.find("\n", i)
        j = len(text) if j < 0 else j
    else:
        j = re.search(end, text[i:]).start() + i
    return html.unescape(text[i:j].strip())


def main() -> None:
    a1 = responses(AMENDMENT_1)
    later = [responses(s) for s in LATER]
    md = []
    for field, start, end in EXCERPTS:
        q = between(a1["revision_responses"][field], start, end)
        same = all(q in html.unescape(v["revision_responses"][field]) for v in later)
        assert same, (field, start)
        md += [f"Field `{field}`:", "", "```text", q, "```", ""]
    out("r3", "osf_amendment1_excerpt.md").write_text("\n".join(md))
    print(f"wrote {out('r3', 'osf_amendment1_excerpt.md')} ({len(EXCERPTS)} excerpts, all unchanged in the later versions)")


if __name__ == "__main__":
    main()
