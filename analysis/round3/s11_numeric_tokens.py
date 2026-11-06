#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Compare every numeric token of the round-2 paper source with the round-3 source.

The round-2 source is paper/main_access.tex and the table files it \\input's at commit
797aeb3 (review/access-r2, the Access-2026-55984 submission); the round-3 source is the
working tree. Each file is diffed line by line; for every changed block the numeric tokens
that disappear and those that appear are listed (tokens present on both sides of a block,
for example in rewrapped text, cancel out). Each block is tagged with the revision item it
belongs to and the reason, from ANNOTATIONS below.

Output: --out (default docs/review/round3/numeric_tokens.csv).
"""
from __future__ import annotations

import argparse
import csv
import difflib
import re
import subprocess
from collections import Counter
from pathlib import Path

from common import REPO

BASE = "797aeb3"
TOKEN = re.compile(r"(?<![A-Za-z_\\])\d+(?:[.,/]\d+)*")
INPUT = re.compile(r"^[^%\n]*?\\input\{([^}]*)\}", re.M)
SECTION = re.compile(r"\\(sub)?section\*?\{([^}]*)\}")

# (file, regex searched in the new or the old text of the block, item code, reason)
ANNOTATIONS = [
    ("main_access.tex", r"begin\{abstract\}|This article evaluates|reported from the second of two|checks flag 89\.2|tool-centre-point declaration that the task prompt|registered validity gate, lets|675B\) show no", "A1",
     "abstract replaced by the round-3 text"),
    ("main_access.tex", r"is 1 of the 135 baseline\s+generations under the full", "V4", "denominator named"),
    ("main_access.tex", r"one per prompt condition, was registered", "A2", "registration status of the cross-model test (V-E)"),
    ("main_access.tex", r"raggedright\\arraybackslash\}p\{0\.30", "V4", "Table 6 column widths for the k/N values"),
    ("main_access.tex", r"Substantive-safe rate \(primary\) &|82/83 \(98\.8|77/83|74/83|38/38|82/135", "V4/A4",
     "Table 6 rates as k/N; rater B primary, adjudicated sensitivity"),
    ("main_access.tex", r"syntactic commitment, Cochran", "A2", "Table 6: exact uncorrected p, gate-passing result"),
    ("main_access.tex", r"registered value is kept|stopl\(\)", "V1", "single safe output: commented call, structural gate"),
    ("main_access.tex", r"adjudicated labels equal rater A's on|non-author rater, is taken", "A4", "rater B primary"),
    ("main_access.tex", r"audit unit and the\s+count unit|111 firings on 82 programs", "A4", "SM-2 audit and count units, deduplicated value"),
    ("main_access.tex", r"at 55\\% and 80\\% under rater B", "A4", "rater B values first"),
    ("main_access.tex", r"Under rater B's labels, the primary estimate|82 of 83, or\s+98\.8", "A4/V4", "rater B primary; denominators named"),
    ("main_access.tex", r"1 of the 135 baseline generations must be read", "V4", "denominator named"),
    ("main_access.tex", r"fall of 3\.98 firings", "A3", "safety-prompt count contrast by check"),
    ("main_access.tex", r"of the 135 baseline generations \(0\.7", "V4", "denominator named"),
    ("main_access.tex", r"excess of generated code over the reference|10\.37\s+firings per program|5\.88 against 5\.73", "A3/V5",
     "per-rule decomposition replaces the twice claim; minimal translation named"),
    ("main_access.tex", r"minimal reference translation \(without the", "V5", "Figure 4 caption names the minimal translation"),
    ("main_access.tex", r"comparator_rules", "A3", "new per-rule table"),
    ("main_access.tex", r"Six of these nine are pseudo-code|between 0 and 3 of", "V2/V4", "pseudo-code safe outputs, structural gate range"),
    ("main_access.tex", r"lower end of the range rests", "A1", "what the SM-6 lower bound measures"),
    ("main_access.tex", r"8\.11 more|incremental escalation comes from the hardcoded", "A3", "adversarial count contrast by check"),
    ("main_access.tex", r"heterogeneity in syntactic commitment, that is|exact permutation \$p\$ of 0\.001", "A2/V3",
     "cross-model test relabelled; uncorrected exact p; gate-passing and gate-pass results; four-test Holm family removed"),
    ("main_access.tex", r"exact \$p = 0\.014\$ and \$0\.019\$|gate-passing outputs at Q4", "A2/V3", "uniform quantization: exact uncorrected p, three bases"),
    ("main_access.tex", r"gate-pass indicator does so in every case", "A2", "registered for sentence rewritten"),
    ("main_access.tex", r"of the gate-passing baseline outputs contain|0 to 3 of the 130\s+stored outputs under the structural gate\) are both", "V2/V4",
     "VII-A denominators and structural gate; qualification by the comparator (A1)"),
    ("main_access.tex", r"registered test, a\s+heterogeneity in syntactic", "A2", "VII-D"),
    ("main_access.tex", r"second of two generation runs", "A1", "Conclusion matches the abstract (two runs, not identical)"),
    ("main_access.tex", r"Three findings emerge|0 to 3 of the 130 stored outputs under the structural gate\. The", "A3/V2/V4",
     "Conclusion first finding: decomposition, structural gate, denominators"),
    ("main_access.tex", r"identical on the\s+gate-passing and gate-pass bases", "A2", "Appendix A: run agreement on the new bases"),
    ("main_access.tex", r"Registered cross-model tests|192 of 192", "A2", "Appendix A: registered cross-model tests"),
    ("tables/sensitivity_e1_baseline.tex", r".", "A1/V4", "Table 14 (was 13): reference translation columns; 130 stored outputs"),
    ("tables/precision_audit_v2.tex", r".", "A4", "Table 10: rater B primary, adjudicated = rater A as sensitivity"),
    ("tables/comparator_rules.tex", r".", "A3", "new Table 13: mean firings per program by check"),
    ("tables/quantization_summary.tex", r".", "A2", "Table 17 (was 16): three bases, exact uncorrected p, tasks per test"),
    ("tables/crossmodel_registered.tex", r".", "A2", "new Table 19: every registered cross-model test"),
]


def sources(rev: str | None) -> dict[str, list[str]]:
    def read(rel: str) -> str | None:
        if rev is None:
            p = REPO / "paper" / rel
            return p.read_text() if p.is_file() else None
        r = subprocess.run(["git", "show", f"{rev}:paper/{rel}"], cwd=REPO, capture_output=True, text=True)
        return r.stdout if r.returncode == 0 else None

    main = read("main_access.tex")
    files = {"main_access.tex": main}
    for name in INPUT.findall(main):
        name = name if name.endswith(".tex") else name + ".tex"
        files[name] = read(name)
    return {k: (v.splitlines() if v is not None else []) for k, v in files.items()}


def tokens(lines: list[str]) -> Counter:
    c = Counter()
    for line in lines:
        code = re.split(r"(?<!\\)%", line, maxsplit=1)[0]
        c.update(TOKEN.findall(code))
    return c


def section_at(lines: list[str], idx: int) -> str:
    for i in range(min(idx, len(lines) - 1), -1, -1):
        m = SECTION.search(lines[i])
        if m:
            return m.group(2)
    return "Abstract"


def annotate(fname: str, text: str) -> tuple[str, str]:
    for f, pat, code, reason in ANNOTATIONS:
        if f == fname and re.search(pat, text):
            return code, reason
    return "UNASSIGNED", ""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=REPO / "docs/review/round3/numeric_tokens.csv")
    args = ap.parse_args()
    old, new = sources(BASE), sources(None)
    rows = []
    for fname in list(new) + [f for f in old if f not in new]:
        a, b = old.get(fname, []), new.get(fname, [])
        sm = difflib.SequenceMatcher(a=a, b=b, autojunk=False)
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag == "equal":
                continue
            ta, tb = tokens(a[i1:i2]), tokens(b[j1:j2])
            removed, added = ta - tb, tb - ta
            if not removed and not added:
                continue
            code, reason = annotate(fname, "\n".join(b[j1:j2]) + "\n" + "\n".join(a[i1:i2]))
            rows.append({"file": f"paper/{fname}", "new_lines": f"{j1 + 1}-{j2}" if j2 > j1 else f"(after {j1})",
                         "old_lines": f"{i1 + 1}-{i2}" if i2 > i1 else f"(after {i1})",
                         "section": section_at(b, j1) if fname == "main_access.tex" else fname,
                         "old_tokens": " ".join(sorted(removed.elements(), key=lambda s: (len(s), s))),
                         "new_tokens": " ".join(sorted(added.elements(), key=lambda s: (len(s), s))),
                         "item": code, "reason": reason})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    un = [r for r in rows if r["item"] == "UNASSIGNED"]
    print(f"wrote {args.out}: {len(rows)} blocks with numeric changes, {len(un)} unassigned")
    for r in un:
        print("UNASSIGNED", r["file"], r["new_lines"], r["old_tokens"][:60], "->", r["new_tokens"][:60])


if __name__ == "__main__":
    main()
