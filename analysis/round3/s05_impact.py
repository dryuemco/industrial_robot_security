#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Impact list: every line of the paper that carries a number or claim touched by A1-A4.

Searches paper/main_access.tex, every file it pulls in with \\input/\\include (recursively)
and the figure captions. Read-only. Each hit is tagged with the item it belongs to and the
place in the paper (abstract, section, table or figure label).

Outputs under out/impact/:
  hits.csv          file, line, place, items, patterns, text
  hits.md           the same grouped by item
  denominators.md   every use of 130 and 135 as a count
  figures.md        figure captions and the script that draws each figure
"""
from __future__ import annotations

import re
from pathlib import Path

import pandas as pd

from common import PAPER, REPO, out, rel

PAPER_DIR = PAPER.parent  # review/access-r2 source (common.R2_COMMIT)
ITEMS = {
    "A1": [r"89\.2", r"SM6-only", r"reference translation", r"[Tt]ranslator", r"preamble", r"set\\_tcp", r"set\\_payload",
           r"5\.1\.1[456]", r"comparator", r"lower (?:end|bound)", r"tool[- ]centre[- ]point", r"\bpayload\b"],
    "A2": [r"Cochran(?!-Armitage)", r"12\.60?\b", r"20\.00\b", r"8\.67", r"7\.82", r"0\.004\b", r"0\.026", r"heterogene",
           r"homogene", r"cross-model", r"registered for", r"per-task violation rate", r"discordant"],
    "A3": [r"10\.4\b", r"10\.37\b", r"\b5\.7\b", r"\b9\.8\b", r"twice", r"never fires", r"rule profile",
           r"input-validation", r"error-handling", r"52 and 82", r"firings per program", r"syntactic prox"],
    "A4": [r"rater[- ]B", r"adjudicat", r"kappa", r"\\kappa", r"inter-rater", r"precision", r"SM2\+SM6", r"\b90\\?%",
           r"program-level", r"high-precision"],
    "range": [r"98\.8", r"89\.2", r"1 to 9", r"\b9 of 13[05]", r"\b1 of 13[05]", r"substantive-safe"],
    "denominator": [r"(?<![\d.])13[05](?![\d.])"],
}


def inputs(path: Path, seen: list) -> list[Path]:
    seen.append(path)
    for m in re.finditer(r"\\(?:input|include)\{([^}]+)\}", path.read_text()):
        p = PAPER_DIR / (m.group(1) if m.group(1).endswith(".tex") else m.group(1) + ".tex")
        if p.is_file() and p not in seen:
            inputs(p, seen)
    return seen


def places(main: Path) -> tuple[dict, dict]:
    """Line -> place for main_access.tex, and input file -> place of its \\input line."""
    lines = main.read_text().splitlines()
    place, cur, env = {}, "front matter", []
    sec_names = {"section": "", "subsection": ""}
    in_abs = False
    for i, l in enumerate(lines, 1):
        if r"\begin{abstract}" in l:
            in_abs = True
        m = re.search(r"\\(section|subsection)\*?\{([^}]*)\}", l)
        if m:
            sec_names[m.group(1)] = m.group(2)
            if m.group(1) == "section":
                sec_names["subsection"] = ""
            cur = sec_names["section"] + (" / " + sec_names["subsection"] if sec_names["subsection"] else "")
        b = re.search(r"\\begin\{(table\*?|figure\*?)\}", l)
        if b:
            env.append([b.group(1), i, ""])
        if env:
            lab = re.search(r"\\label\{((?:tab|fig):[^}]+)\}", l)
            if lab:
                env[-1][2] = lab.group(1)
        where = "abstract" if in_abs else cur
        if env:
            where += f" [{env[-1][0]} from line {env[-1][1]}{', ' + env[-1][2] if env[-1][2] else ''}]"
        place[i] = where
        if env and re.search(r"\\end\{(table\*?|figure\*?)\}", l):
            env.pop()
        if r"\end{abstract}" in l:
            in_abs = False
    inp = {}
    for i, l in enumerate(lines, 1):
        m = re.search(r"\\input\{([^}]+)\}", l)
        if m:
            inp[m.group(1) if m.group(1).endswith(".tex") else m.group(1) + ".tex"] = f"{place[i]} (\\input at line {i})"
    return place, inp


def main() -> None:
    main_tex = PAPER_DIR / "main_access.tex"
    files = inputs(main_tex, [])
    place, inp = places(main_tex)
    rows = []
    for f in files:
        rel_in = str(f.relative_to(PAPER_DIR))
        for i, l in enumerate(f.read_text().splitlines(), 1):
            if l.lstrip().startswith("%"):
                continue
            items, pats = [], []
            for item, plist in ITEMS.items():
                for p in plist:
                    if re.search(p, l):
                        items.append(item)
                        pats.append(p)
            if not items:
                continue
            if f == main_tex:
                where = place[i]
            else:
                lab = re.search(r"\\label\{([^}]+)\}", f.read_text())
                where = inp.get(rel_in, "?") + (f" [{lab.group(1)}]" if lab else "")
            rows.append({"file": rel(f), "line": i, "place": where, "items": ",".join(sorted(set(items))),
                         "patterns": " | ".join(sorted(set(pats))), "text": l.strip()})
    df = pd.DataFrame(rows)
    df.to_csv(out("impact", "hits.csv"), index=False)

    md = ["# Impact grep", "", f"Files searched: {', '.join('`' + rel(f) + '`' for f in files)}", ""]
    for item in ITEMS:
        sub = df[df["items"].str.contains(item)]
        md += [f"## {item} ({len(sub)} lines)", "", "| file:line | place | text |", "|---|---|---|"]
        md += [f"| {r.file}:{r.line} | {r.place} | {r.text[:220].replace('|', '/')} |" for r in sub.itertuples()]
        md.append("")
    out("impact", "hits.md").write_text("\n".join(md))

    den = df[df["items"].str.contains("denominator")]
    dm = ["# Uses of 130 and 135 in the paper", "",
          "130 = baseline outputs with a stored program (5 of the 135 calls returned no program: "
          "4 errors, 1 parse failure; out/gate_equivalence.csv); 135 = baseline calls.", "",
          "| file:line | place | count used | text |", "|---|---|---|---|"]
    for r in den.itertuples():
        nums = sorted(set(re.findall(r"(?<![\d.])(13[05])(?![\d.])", r.text)))
        dm.append(f"| {r.file}:{r.line} | {r.place} | {', '.join(nums)} | {r.text[:240].replace('|', '/')} |")
    out("impact", "denominators.md").write_text("\n".join(dm) + "\n")

    # Figure captions and the scripts behind the figures.
    tex = main_tex.read_text()
    fm = ["# Figures: captions and source scripts", ""]
    for m in re.finditer(r"\\begin\{figure\*?\}(.*?)\\end\{figure\*?\}", tex, re.S):
        body = m.group(1)
        line = tex[:m.start()].count("\n") + 1
        img = re.findall(r"\\includegraphics(?:\[[^]]*\])?\{([^}]+)\}", body)
        lab = re.findall(r"\\label\{([^}]+)\}", body)
        cap = re.search(r"\\caption\{(.*)\}\s*\\label", body, re.S)
        scripts = []
        for im in img:
            stem = Path(im).stem
            scripts += [rel(p) for p in (REPO / "scripts/figures").glob("*.py") if stem in p.read_text()]
        fm += [f"## line {line}: {', '.join(lab)}", f"- image: {', '.join(img)}",
               f"- drawn by: {', '.join(sorted(set(scripts))) or 'not found by file name'}",
               f"- caption: {re.sub(r'\s+', ' ', cap.group(1)) if cap else ''}", ""]
    out("impact", "figures.md").write_text("\n".join(fm))
    print(df.groupby("items").size().to_string())
    print(f"{len(df)} lines; files: {[rel(f) for f in files]}")


if __name__ == "__main__":
    main()
