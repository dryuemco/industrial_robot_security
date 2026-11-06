# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""Checks of the statistics helpers in common.py against independent implementations."""
from __future__ import annotations

import itertools
import math
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import cochran_exact, cochran_q_stat, cohen_kappa, holm, wilson  # noqa: E402


def brute_force_p(mat: np.ndarray) -> float:
    """Exact permutation p by enumerating every within-block arrangement."""
    n, k = mat.shape
    q_obs = cochran_q_stat(mat)
    options = [sorted(set(itertools.permutations(row))) for row in mat]
    hits = total = 0
    for combo in itertools.product(*options):
        q = cochran_q_stat(np.array(combo))
        total += 1
        hits += q >= q_obs - 1e-9
    return hits / total


@pytest.mark.parametrize("seed", range(8))
def test_exact_matches_brute_force(seed):
    rng = np.random.default_rng(seed)
    mat = rng.integers(0, 2, size=(7, 3))
    if cochran_exact(mat)["n_discordant"] == 0:
        mat[0] = [1, 0, 0]
    res = cochran_exact(mat)
    assert res["p_exact"] == pytest.approx(brute_force_p(mat), abs=1e-12)


def test_q_matches_statsmodels():
    from statsmodels.stats.contingency_tables import cochrans_q
    rng = np.random.default_rng(1)
    for _ in range(20):
        mat = rng.integers(0, 2, size=(15, 3))
        if cochran_exact(mat)["n_discordant"] == 0:
            continue
        r = cochrans_q(mat)
        res = cochran_exact(mat)
        assert res["Q"] == pytest.approx(r.statistic)
        assert res["p_asymptotic"] == pytest.approx(r.pvalue)


def test_no_discordant_block_is_undefined():
    mat = np.array([[1, 1, 1], [0, 0, 0], [1, 1, 1]])
    res = cochran_exact(mat)
    assert res["n_discordant"] == 0
    assert math.isnan(res["Q"]) and math.isnan(res["p_exact"])


def test_monte_carlo_agrees_with_exact():
    mat = np.array([[1, 0, 0]] * 4 + [[1, 1, 0]] * 3 + [[1, 1, 1]] * 4 + [[0, 1, 0]] * 2 + [[0, 0, 0]] * 2)
    q_obs = cochran_q_stat(mat)
    rnd = random.Random(42)
    hits = 0
    n = 20000
    for _ in range(n):
        perm = np.array([rnd.sample(list(r), len(r)) for r in mat])
        hits += cochran_q_stat(perm) >= q_obs - 1e-9
    assert hits / n == pytest.approx(cochran_exact(mat)["p_exact"], abs=0.01)


def test_wilson_matches_statsmodels():
    from statsmodels.stats.proportion import proportion_confint
    for k, n in [(18, 20), (0, 20), (20, 20), (9, 20), (77, 100)]:
        lo, hi = proportion_confint(k, n, method="wilson")
        assert wilson(k, n) == pytest.approx((lo, hi), abs=1e-9)


def test_holm_matches_statsmodels():
    from statsmodels.stats.multitest import multipletests
    p = [0.0018, 4.5e-5, 0.018, 1.0e-5]
    assert holm(p) == pytest.approx(list(multipletests(p, method="holm")[1]))
    assert math.isnan(holm([float("nan"), 0.11])[0]) and holm([float("nan"), 0.11])[1] == pytest.approx(0.11)


def test_kappa_perfect_and_chance():
    a = pd.Series(["TP", "FP", "TP", "FP"])
    assert cohen_kappa(a, a)[1] == pytest.approx(1.0)
    b = pd.Series(["TP", "TP", "FP", "FP"])
    assert cohen_kappa(a, b)[1] == pytest.approx(0.0)
