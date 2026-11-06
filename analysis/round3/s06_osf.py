#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""A2.4: is the cross-model test in the OSF preregistration?

Reads the registration ve5m2 and every schema response (original registration, the
three amendments, the record note) from api.osf.io, read-only, without login or token,
and caches the raw JSON under out/osf/. Later runs use the cache unless --refresh is
given. Searches every registered text field, the local mirror docs/OSF_PREREGISTRATION.md
and the three paper statements.

Outputs: out/osf/*.json, out/a2/osf_search.csv, out/a2/osf_search.md
"""
from __future__ import annotations

import argparse
import json
import re
import urllib.request

import pandas as pd

from common import OUT, REPO, md_table, out, rel

API = "https://api.osf.io/v2"
REG = "ve5m2"
PATTERNS = [r"cochran", r"heterogene", r"homogene", r"cross[- ]model", r"across (?:the )?(?:three |3 )?models",
            r"between(?:-| )models?", r"models? differ", r"compar\w* (?:the |across )?(?:three |3 )?models",
            r"model (?:as a )?factor", r"model effect", r"per[- ]model"]
PAPER_STATEMENTS = [r"cross-model\s+homogeneity test is reported as an additional analysis only",
                    r"Cross-model heterogeneity, Cochran",
                    r"registered for at either uniform quantization",
                    r"additional non-confirmatory analysis outside the preregistered confirmatory"]


def get(url: str) -> dict:
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.api+json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read().decode())


def fetch(refresh: bool) -> dict[str, dict]:
    cache = OUT / "osf"
    cache.mkdir(parents=True, exist_ok=True)
    docs = {}
    if not refresh and (cache / "schema_responses.json").is_file():
        for f in sorted(cache.glob("*.json")):
            docs[f.stem] = json.loads(f.read_text())
        return docs
    docs["registration"] = get(f"{API}/registrations/{REG}/")
    lst = get(f"{API}/registrations/{REG}/schema_responses/")
    docs["schema_responses"] = lst
    for r in lst["data"]:
        docs[f"schema_response_{r['id']}"] = get(f"{API}/schema_responses/{r['id']}/")
    for k, v in docs.items():
        (cache / f"{k}.json").write_text(json.dumps(v, indent=1, ensure_ascii=False))
    return docs


def texts(docs: dict) -> list[tuple[str, str, str]]:
    """(source, field, text) for every string in the registered content."""
    rows = []

    def walk(o, src, path):
        if isinstance(o, dict):
            for k, v in o.items():
                walk(v, src, f"{path}.{k}" if path else k)
        elif isinstance(o, list):
            for i, v in enumerate(o):
                walk(v, src, f"{path}[{i}]")
        elif isinstance(o, str) and len(o) > 3:
            rows.append((src, path, o))

    reg = docs["registration"]["data"]["attributes"]
    walk({k: reg.get(k) for k in ("title", "description", "registration_responses", "registered_meta")},
         "registration ve5m2 (current)", "")
    for k, v in docs.items():
        if k.startswith("schema_response_"):
            a = v["data"]["attributes"]
            walk({"revision_justification": a.get("revision_justification"),
                  "revision_responses": a.get("revision_responses")}, k, "")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--refresh", action="store_true", help="re-read api.osf.io instead of the cache")
    args = ap.parse_args()
    try:
        docs = fetch(args.refresh)
    except Exception as exc:  # network down: report MISSING, keep the rest of the run going
        out("a2", "osf_search.md").write_text(f"MISSING: api.osf.io not reachable and no cache ({exc})\n")
        print(f"MISSING: {exc}")
        return
    meta = []
    for k, v in docs.items():
        if k.startswith("schema_response_"):
            a = v["data"]["attributes"]
            meta.append({"id": v["data"]["id"], "state": a.get("reviews_state"),
                         "submitted": a.get("date_submitted"), "created": a.get("date_created"),
                         "revised_fields": ";".join(a.get("updated_response_keys") or []),
                         "justification_start": (a.get("revision_justification") or "")[:120]})
    meta = pd.DataFrame(meta).sort_values("created")

    hits = []
    rows = texts(docs)
    mirror = REPO / "docs/OSF_PREREGISTRATION.md"
    rows += [("local mirror " + rel(mirror), f"line {i}", l) for i, l in enumerate(mirror.read_text().splitlines(), 1)]
    for src, field, txt in rows:
        for pat in PATTERNS:
            for m in re.finditer(pat, txt, re.I):
                s, e = max(0, m.start() - 160), min(len(txt), m.end() + 160)
                hits.append({"source": src, "field": field, "pattern": pat,
                             "context": re.sub(r"\s+", " ", txt[s:e])})
    hits = pd.DataFrame(hits, columns=["source", "field", "pattern", "context"]).drop_duplicates()
    hits.to_csv(out("a2", "osf_search.csv"), index=False)

    tex = (REPO / "paper/main_access.tex").read_text().splitlines()
    joined = [(i + 1, l) for i, l in enumerate(tex)]
    stmts = []
    for pat in PAPER_STATEMENTS:
        # match across a line break: search a sliding window of three lines
        for i in range(len(joined)):
            win = " ".join(l for _, l in joined[i:i + 3])
            if re.search(pat, win) and not re.search(pat, " ".join(l for _, l in joined[i + 1:i + 3])):
                stmts.append((pat, joined[i][0], " ".join(l.strip() for _, l in joined[max(0, i - 1):i + 4])))
                break

    # Per version: does the registered content mention Cochran's Q; verbatim sentences from
    # the first version that does, and whether the latest version carries the same sentences.
    versions = []
    for k, v in sorted(((k, v) for k, v in docs.items() if k.startswith("schema_response_")),
                       key=lambda kv: kv[1]["data"]["attributes"]["date_created"]):
        rr = v["data"]["attributes"].get("revision_responses") or {}
        sents = []
        for key, txt in rr.items():
            if isinstance(txt, str):
                for m in re.finditer(r"[^.\n]*(?:Cochran|invalid pseudo-code|parser-invalid|Combined Violation Verdict)[^.\n]*[.\n]?", txt, re.I):
                    sents.append((key, m.group(0).strip()))
        versions.append((v["data"]["id"], v["data"]["attributes"]["date_created"], sents))
    first = next(((vid, d, s) for vid, d, s in versions if any("cochran" in t.lower() for _, t in s)), None)
    latest = versions[-1]

    md = ["# A2.4 Cross-model test in the OSF preregistration", "",
          f"Read-only from {API}/registrations/{REG}/ and {API}/schema_responses/<id>/; raw JSON cached in "
          f"`{rel(OUT / 'osf')}`.", "", "## Schema responses (registration versions)", "",
          md_table(meta), "", "## Verdict", ""]
    md += [f"- {vid} (created {d[:10]}): Cochran's Q mentioned: "
           f"{any('cochran' in t.lower() for _, t in s)}" for vid, d, s in versions]
    if first is None:
        md.append("\n**No match in the registration or any schema response: the cross-model test is not preregistered.**")
    else:
        md += ["", f"First version that registers it: `{first[0]}` (created {first[1][:10]}). "
               "Verbatim sentences (field key :: sentence) on Cochran's Q, invalid pseudo-code and the "
               "combined violation verdict:", ""]
        md += [f"- `{k}` :: \"{t}\"" for k, t in first[2]]
        same = {t for _, t in first[2]} <= {t for _, t in latest[2]}
        md += ["", f"Latest version `{latest[0]}` contains all of these sentences unchanged: {same}"]
        rr = docs[f"schema_response_{first[0]}"]["data"]["attributes"]["revision_responses"]
        md += ["", f"Registered cross-model paragraphs, verbatim from `{first[0]}`:", ""]
        for key in sorted(k for k, v in rr.items() if isinstance(v, str) and "Cross-Model" in v):
            v = rr[key]
            i = v.find("Cross-Model")
            j = v.find("---", v.find("\n", i) + 1)
            k2 = v.find("\n\n", v.find("\n", i) + 2)
            end = min(x for x in (j, k2, len(v)) if x > i)
            md.append(f"- `{key}`: \"{re.sub(r'\s+', ' ', v[i:end]).strip()}\"")
    md += ["", f"## Pattern search over every registered text field ({len(rows)} strings incl. the local mirror)", "",
           f"Patterns: {', '.join('`' + p + '`' for p in PATTERNS)}. Full list: `{rel(OUT / 'a2' / 'osf_search.csv')}` "
           f"({len(hits)} rows). Matches on Cochran / cross-model / heterogeneity / homogeneity:", ""]
    osf_hits = hits[~hits.source.str.startswith("local mirror")
                    & hits.pattern.isin([r"cochran", r"heterogene", r"homogene", r"cross[- ]model"])]
    md += [f"- {r.source} / `{r.field}` [{r.pattern}]: \"{r.context}\"" for r in osf_hits.itertuples()]
    loc = hits[hits.source.str.startswith("local mirror")]
    md += ["", "Matches in the local mirror:" if not loc.empty else "No match in the local mirror either."]
    md += [f"- {r.field} [{r.pattern}]: \"{r.context}\"" for r in loc.itertuples()]
    md += ["", "## Paper statements on the status of the test (paper/main_access.tex)", ""]
    md += [f"- line {ln}: \"{ctx}\"" for _, ln, ctx in stmts]
    out("a2", "osf_search.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
