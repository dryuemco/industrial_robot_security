#!/usr/bin/env python3
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Shared paths, loaders and statistics for the round-3 analyses (Access-2026-55984).

Every input is read from a committed path. Nothing here writes outside
analysis/round3/out/. The rule checker, gates, rule sets and translator are
imported from the existing scripts so that the definitions are the paper's own.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import sys
from itertools import product
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
OUT = HERE / "out"

RUN2_E1 = REPO / "data/confirmatory/run2_2026-05/E1_full"
RUN1_E1 = REPO / "data/confirmatory/run1_2026-04/e1_confirmatory_session14"
RUN2_DIR = REPO / "data/confirmatory/run2_2026-05"
TASKS_DIR = REPO / "paper/data/tasks_as_run"
AUDIT_DIR = REPO / "docs/review/audit_v2"
PAPER = REPO / "paper/main_access.tex"
TABLES = REPO / "paper/tables"
X10 = {
    "Qwen2.5-Coder-32B": (REPO / "paper/data/X10_qwen_q4_0/e1_results.csv", "qwen2.5-coder:32b-instruct-q4_0", "Q4_0"),
    "DeepSeek-Coder-V2-16B": (REPO / "paper/data/X10_deepseek_q4_K_M/e1_results.csv", "deepseek-coder-v2:16b-lite-instruct-q4_K_M", "Q4_K_M"),
    "CodeLlama-34B": (REPO / "paper/data/X10_codellama_q4_K_M/e1_results.csv", "codellama:34b-instruct-q4_K_M", "Q4_K_M"),
}
# Confirmatory model id -> (display name, level it was served at)
SERVED = {
    "qwen2.5-coder:32b": ("Qwen2.5-Coder-32B", "Q4_K_M"),
    "deepseek-coder-v2:16b": ("DeepSeek-Coder-V2-16B", "Q4_0"),
    "codellama:34b": ("CodeLlama-34B", "Q4_0"),
}
MODELS = ["Qwen2.5-Coder-32B", "DeepSeek-Coder-V2-16B", "CodeLlama-34B"]
RULES = ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]

for p in ("enfield_watchdog_static", "enfield_tasks", "enfield_llm", "enfield_translators",
          "scripts", "scripts/review"):
    sys.path.insert(0, str(REPO / p))

import sensitivity_rescoring as sr  # noqa: E402  (gates, rule sets, firings)
import comparator_urscript as cu  # noqa: E402  (translator programs, per-program score)
from enfield_watchdog_static import StaticWatchdog  # noqa: E402
from sm5_corrected import check_sm5_corrected  # noqa: E402

FNAME = re.compile(r"^(T\d{3})_(.+?)_(baseline|safety)_rep(\d)(?:_retry(\d))?(\.invalid)?\.urscript$")
FILE_MODEL = {"qwen2.5-coder_32b": "qwen2.5-coder:32b", "deepseek-coder-v2_16b": "deepseek-coder-v2:16b",
              "codellama_34b": "codellama:34b"}

_WD = None


def wd() -> StaticWatchdog:
    global _WD
    if _WD is None:
        _WD = StaticWatchdog()
    return _WD


def out(*parts: str) -> Path:
    p = OUT.joinpath(*parts)
    p.parent.mkdir(parents=True, exist_ok=True)
    return p


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()


def rel(p: Path) -> str:
    return str(Path(p).resolve().relative_to(REPO))


# --------------------------------------------------------------------------- flags

FLAGS = OUT / "flags.json"


def set_flag(key: str, value) -> None:
    d = json.loads(FLAGS.read_text()) if FLAGS.is_file() else {}
    d[key] = value
    FLAGS.parent.mkdir(parents=True, exist_ok=True)
    FLAGS.write_text(json.dumps(d, indent=2, sort_keys=True))


def get_flag(key: str, default=None):
    return json.loads(FLAGS.read_text()).get(key, default) if FLAGS.is_file() else default


# --------------------------------------------------------------------------- loaders

def load_e1_csv(run_dir: Path) -> pd.DataFrame:
    d = pd.read_csv(run_dir / "e1_results.csv")
    d["model_disp"] = d.model.map(lambda m: SERVED[m][0])
    d["gate"] = (d.status == "success").astype(int)
    d["viol"] = (d.total_violations > 0).astype(int)
    return d


def load_x10(model_disp: str) -> pd.DataFrame:
    path, mid, _ = X10[model_disp]
    d = pd.read_csv(path)
    d = d[d.model == mid].copy()
    d["model_disp"] = model_disp
    d["gate"] = (d.status == "success").astype(int)
    d["viol"] = (d.total_violations > 0).astype(int)
    return d


def load_code(run_dir: Path, condition: str = "baseline") -> list[dict]:
    """Every stored program of one E1 condition, with the row keys of e1_results.csv."""
    progs = []
    for f in sorted((run_dir / "code").glob("*.urscript")):
        m = FNAME.match(f.name)
        if not m or m.group(3) != condition:
            continue
        task, mtok, cond, rep, retry, invalid = m.groups()
        progs.append({"file": f, "task_id": task, "model": FILE_MODEL[mtok],
                      "model_disp": SERVED[FILE_MODEL[mtok]][0], "rep": int(rep),
                      "invalid_suffix": bool(invalid), "code": f.read_text(errors="replace")})
    return progs


def task_caps() -> dict[str, float]:
    return sr.task_caps()


def load_tasks() -> dict[str, dict]:
    return {json.loads(f.read_text())["task"]["id"]: json.loads(f.read_text())
            for f in sorted(TASKS_DIR.glob("T*.json"))}


# --------------------------------------------------------------------------- scoring

def rule_counts(code: str, cap: float | None) -> dict[str, dict[str, int]]:
    """Per-rule firing counts under the preregistered and the corrected SM-5 rule sets."""
    rep = wd().analyze_code(code)
    pre = {r: 0 for r in RULES}
    for v in rep.violations:
        pre[v.detection_mechanism] = pre.get(v.detection_mechanism, 0) + 1
    corr = dict(pre)
    corr["SM-5"] = len(check_sm5_corrected(code, tcp_cap=cap))
    return {"pre": pre, "corr": corr}


def violations(code: str):
    return wd().analyze_code(code).violations


def ruleset_firings(code: str, cap: float | None, ruleset: str) -> int:
    keep, corrected = sr.RULESETS[ruleset]
    return sr.firings(code, cap, wd(), keep, corrected)


# --------------------------------------------------------------------------- statistics

def wilson(k: int, n: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, centre - half), min(1.0, centre + half)


def clopper_pearson(k: int, n: int) -> tuple[float, float]:
    """Exact 95% interval, as in scripts/review/precision_audit_analyze.py."""
    from scipy.stats import beta
    if n == 0:
        return float("nan"), float("nan")
    lo = 0.0 if k == 0 else beta.ppf(0.025, k, n - k + 1)
    hi = 1.0 if k == n else beta.ppf(0.975, k + 1, n - k)
    return float(lo), float(hi)


def cohen_kappa(a: pd.Series, b: pd.Series) -> tuple[float, float, int]:
    """Percent agreement, Cohen's kappa and n on TP/FP pairs (UNSURE excluded pairwise).

    Same definition as scripts/review/precision_audit_analyze.py.
    """
    m = a.isin(["TP", "FP"]) & b.isin(["TP", "FP"])
    a, b = a[m], b[m]
    n = len(a)
    if n == 0:
        return float("nan"), float("nan"), 0
    po = float((a == b).mean())
    pe = sum(float((a == c).mean()) * float((b == c).mean()) for c in ["TP", "FP"])
    k = (po - pe) / (1 - pe) if pe < 1 else 1.0
    return po, k, n


def cochran_q_stat(mat: np.ndarray) -> float:
    """Cochran's Q for an (N blocks x k) 0/1 matrix; nan when no block is discordant."""
    mat = np.asarray(mat, dtype=int)
    n, k = mat.shape
    C = mat.sum(axis=0)
    R = mat.sum(axis=1)
    T = R.sum()
    den = k * T - (R ** 2).sum()
    if den == 0:
        return float("nan")
    return float((k - 1) * (k * (C ** 2).sum() - T ** 2) / den)


def cochran_exact(mat: np.ndarray) -> dict:
    """Asymptotic and exact (within-block permutation) Cochran's Q.

    Under H0 the k entries of each block are exchangeable. Q depends on the
    data only through the column totals once the row totals are fixed, so the
    exact null distribution is obtained by convolving, block by block, the
    uniform distribution over the distinct arrangements of each discordant
    block. Exact enumeration; no Monte Carlo needed at this size.
    """
    from scipy.stats import chi2
    mat = np.asarray(mat, dtype=int)
    n, k = mat.shape
    R = mat.sum(axis=1)
    disc = [(0 < r < k) for r in R]
    n_disc = int(sum(disc))
    q_obs = cochran_q_stat(mat)
    res = {"n_blocks": n, "k": k, "n_discordant": n_disc, "Q": q_obs,
           "p_asymptotic": float("nan"), "p_exact": float("nan"), "n_arrangements": 0}
    if n_disc == 0 or math.isnan(q_obs):
        return res
    res["p_asymptotic"] = float(chi2.sf(q_obs, k - 1))
    base = mat[[not d for d in disc]].sum(axis=0)
    dist = {tuple(int(x) for x in base): 1.0}
    total = 1
    for r in R[np.array(disc)]:
        arrs = sorted({tuple(a) for a in _arrangements(k, int(r))})
        total *= len(arrs)
        new: dict[tuple, float] = {}
        for c, pr in dist.items():
            for a in arrs:
                key = tuple(ci + ai for ci, ai in zip(c, a))
                new[key] = new.get(key, 0.0) + pr / len(arrs)
        dist = new
    T = int(R.sum())
    den = k * T - int((R ** 2).sum())
    p = 0.0
    for c, pr in dist.items():
        q = (k - 1) * (k * sum(x * x for x in c) - T * T) / den
        if q >= q_obs - 1e-9:
            p += pr
    res["p_exact"] = float(min(1.0, p))
    res["n_arrangements"] = total
    return res


def _arrangements(k: int, r: int):
    for bits in product((0, 1), repeat=k):
        if sum(bits) == r:
            yield bits


def holm(pvals: list[float]) -> list[float]:
    """Holm step-down adjusted p values; nan entries are left out of the family."""
    idx = [i for i, p in enumerate(pvals) if not (p is None or math.isnan(p))]
    m = len(idx)
    adj = [float("nan")] * len(pvals)
    running = 0.0
    for rank, i in enumerate(sorted(idx, key=lambda j: pvals[j])):
        running = max(running, min(1.0, (m - rank) * pvals[i]))
        adj[i] = running
    return adj


def md_table(df: pd.DataFrame, floatfmt: str = "{:.3f}") -> str:
    cols = list(df.columns)
    lines = ["| " + " | ".join(str(c) for c in cols) + " |", "|" + "---|" * len(cols)]
    for _, r in df.iterrows():
        cells = []
        for c in cols:
            v = r[c]
            if isinstance(v, float):
                cells.append("" if math.isnan(v) else floatfmt.format(v))
            else:
                cells.append(str(v))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)
