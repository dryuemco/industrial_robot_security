#!/usr/bin/env python3
"""Mark new and revised floats in a latexdiff output file.

latexdiff treats table and figure environments as pictures (PICTUREENV in
build_diff.sh), so a table that is entirely new since the submitted version is
compiled without any highlighting. IEEE Access asks for every change to be
highlighted, so this script inserts a highlighted line at the top of every
float whose label did not exist at tag paper-submitted-r0 ("New in this
revision"), whose graphics file changed since that tag ("Figure regenerated in
this revision"), or whose environment text (caption, body, notes) differs from
the submitted version ("Revised in this revision"). The three rules are applied
in that order of precedence. Floats that latexdiff emits inside a deleted block
are skipped.

Usage: mark_new_floats.py <diff.tex>   (rewrites the file in place)
"""
import pathlib
import re
import subprocess
import sys

TAG = "paper-submitted-r0"
FLOAT_RE = re.compile(r"\\begin\{(table\*?|figure\*?)\}(\[[^\]]*\])?")
LABEL_RE = re.compile(r"\\label\{((?:tab|fig|lst):[^}]*)\}")
GRAPHICS_RE = re.compile(r"\\includegraphics(?:\[[^\]]*\])?\{([^}]*)\}")
DEL_BEGIN_RE = re.compile(r"\\DIFdelbegin(?:FL)?\b")
DEL_END_RE = re.compile(r"\\DIFdelend(?:FL)?\b")
# latexdiff artefacts stripped before comparing a float with its submitted version.
DIFDELCMD_LINE_RE = re.compile(r"^%DIFDELCMD.*$", re.M)
DIF_BLOCK_RE = re.compile(r"\\DIF(?:add|del)(?:begin|end)(?:FL)?\b")
DIF_ADD_RE = re.compile(r"\\DIFadd(?:FL)?\{")
DIF_DEL_RE = re.compile(r"\\DIFdel(?:FL)?\{")
COMMENT_RE = re.compile(r"(?<!\\)%.*$", re.M)


def git(*args: str) -> str:
    return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout


def submitted_sources() -> str:
    src = git("show", f"{TAG}:paper/main_access.tex")
    for path in git("ls-tree", "-r", "--name-only", TAG, "paper/tables").split():
        src += "\n" + git("show", f"{TAG}:{path}")
    return src


def submitted_labels(src: str) -> set[str]:
    return set(LABEL_RE.findall(src))


def changed_graphics() -> set[str]:
    names = set()
    for path in git("diff", "--name-only", TAG, "--", ":/paper/figures").split():
        names.add(pathlib.Path(path).stem)
    return names


def float_end(text: str, env: str, start: int):
    """Position of the first uncommented \\end{env} after start, or None."""
    m = re.compile(r"^[^%\n]*\\end\{" + re.escape(env) + r"\}", re.M).search(text, start)
    return m.start() if m else None


def unwrap(text: str, cmd_re: re.Pattern, keep: bool) -> str:
    """Replace every \\cmd{...} matched by cmd_re with its content (keep) or nothing."""
    out, pos = [], 0
    while True:
        m = cmd_re.search(text, pos)
        if m is None:
            out.append(text[pos:])
            return "".join(out)
        out.append(text[pos:m.start()])
        depth, i = 1, m.end()
        while i < len(text) and depth:
            c = text[i]
            if c == "\\":
                i += 2
                continue
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
            i += 1
        if keep:
            out.append(unwrap(text[m.end():i - 1], cmd_re, keep))
        pos = i


def normalize(body: str, strip_diff: bool = False) -> str:
    if strip_diff:
        body = DIFDELCMD_LINE_RE.sub("", body)
        body = unwrap(body, DIF_ADD_RE, keep=True)
        body = unwrap(body, DIF_DEL_RE, keep=False)
        body = DIF_BLOCK_RE.sub("", body)
    body = COMMENT_RE.sub("", body)
    return " ".join(body.split())


def submitted_float_bodies(src: str) -> dict[str, str]:
    """Map every float label in the submitted sources to its normalized body."""
    bodies = {}
    for m in FLOAT_RE.finditer(src):
        line_start = src.rfind("\n", 0, m.start()) + 1
        if "%" in src[line_start:m.start()]:
            continue
        end = float_end(src, m.group(1), m.end())
        if end is None:
            continue
        body = src[m.end():end]
        for lab in LABEL_RE.findall(body):
            bodies[lab] = normalize(body)
    return bodies


def in_deleted_block(text: str, pos: int) -> bool:
    last_begin = max((m.end() for m in DEL_BEGIN_RE.finditer(text, 0, pos)), default=-1)
    last_end = max((m.end() for m in DEL_END_RE.finditer(text, 0, pos)), default=-1)
    return last_begin > last_end


def main(path: str) -> None:
    text = pathlib.Path(path).read_text()
    src = submitted_sources()
    old_labels = submitted_labels(src)
    old_bodies = submitted_float_bodies(src)
    changed = changed_graphics()
    out, cursor, marked = [], 0, []
    for m in FLOAT_RE.finditer(text):
        env = m.group(1)
        line_start = text.rfind("\n", 0, m.start()) + 1
        if "%" in text[line_start:m.start()]:
            continue  # latexdiff comments out deleted floats with %DIFDELCMD
        end = float_end(text, env, m.end())
        if end is None or in_deleted_block(text, m.start()):
            continue
        body = text[m.end():end]
        labels = LABEL_RE.findall(body)
        graphics = {pathlib.Path(g).stem for g in GRAPHICS_RE.findall(body)}
        if labels and all(lab not in old_labels for lab in labels):
            note = "New in this revision."
        elif graphics & changed:
            note = "Figure regenerated in this revision."
        elif labels and any(
            lab in old_bodies and old_bodies[lab] != normalize(body, strip_diff=True)
            for lab in labels
        ):
            note = "Revised in this revision."
        else:
            continue
        out.append(text[cursor:m.end()])
        out.append("\n\\noindent{\\footnotesize\\hl{\\textbf{" + note + "}}}\\par\\vspace{2pt}\n")
        cursor = m.end()
        marked.append((labels[0] if labels else env, note))
    out.append(text[cursor:])
    # The container writes the diff file as root; replace it by renaming a
    # sibling, which only needs write permission on the directory.
    tmp = pathlib.Path(path + ".marked")
    tmp.write_text("".join(out))
    tmp.replace(path)
    for label, note in marked:
        print(f"marked {label}: {note}")


if __name__ == "__main__":
    main(sys.argv[1])
