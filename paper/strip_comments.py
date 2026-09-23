#!/usr/bin/env python3
"""Remove LaTeX comments from the source files of the upload package.

Used when building source.zip: lines that are only a comment are dropped, and
a trailing comment is cut back to its bare "%" so that a "%" that suppresses
an end-of-line space keeps doing so. Escaped "\\%" and the contents of
verbatim and lstlisting environments are left untouched.

Usage: python3 paper/strip_comments.py FILE.tex [FILE.tex ...]   (edits in place)
"""
from __future__ import annotations

import re
import sys

RAW_BEGIN = re.compile(r"\\begin\{(verbatim\*?|lstlisting)\}")
RAW_END = re.compile(r"\\end\{(verbatim\*?|lstlisting)\}")


def comment_start(line: str) -> int:
    """Index of the first unescaped %, or -1."""
    i = 0
    while i < len(line):
        if line[i] == "\\":
            i += 2
            continue
        if line[i] == "%":
            return i
        i += 1
    return -1


def strip(text: str) -> str:
    out, raw = [], False
    for line in text.splitlines(keepends=True):
        if raw:
            out.append(line)
            raw = not RAW_END.search(line)
            continue
        k = comment_start(line)
        if k >= 0:
            code = line[:k]
            if not code.strip():
                continue  # comment-only line: drop it together with its newline
            line = code + "%\n"
        out.append(line)
        m = RAW_BEGIN.search(line)
        raw = bool(m) and not RAW_END.search(line[m.end():])
    return "".join(out)


def main() -> None:
    for path in sys.argv[1:]:
        with open(path, encoding="utf-8") as f:
            text = f.read()
        with open(path, "w", encoding="utf-8") as f:
            f.write(strip(text))


if __name__ == "__main__":
    main()
