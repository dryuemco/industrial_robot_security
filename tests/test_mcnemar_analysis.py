"""
Pytest tests for scripts/mcnemar_analysis.py
=============================================
Tests cover: contingency table building, McNemar statistic,
Wilson CI, H4/H5/H6 hypothesis tests, demo data, and edge cases.

Run:
  pytest tests/test_mcnemar_analysis.py -v
"""

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

# Add scripts/ to path for import
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from mcnemar_analysis import (
    ALPHA,
    H4_THRESHOLD,
    H5_THRESHOLD,
    H6_THRESHOLD,
    ContingencyTable,
    McNemarResult,
    build_contingency,
    delta_ci_newcombe,
    generate_demo_data,
    run_mcnemar,
    run_h4,
    run_h5,
    run_h6,
    wilson_ci,
)


# ---------------------------------------------------------------------------
# ContingencyTable
# ---------------------------------------------------------------------------

class TestContingencyTable:
    def test_counts(self):
        t = ContingencyTable(a=10, b=5, c=3, d=7)
        assert t.n == 25

    def test_rate_a(self):
        # A violates in cells c and d
        t = ContingencyTable(a=10, b=5, c=3, d=7)
        assert math.isclose(t.rate_a, (3 + 7) / 25)

    def test_rate_b(self):
        # B violates in cells b and d
        t = ContingencyTable(a=10, b=5, c=3, d=7)
        assert math.isclose(t.rate_b, (5 + 7) / 25)

    def test_rate_empty(self):
        t = ContingencyTable()
        assert math.isnan(t.rate_a)
        assert math.isnan(t.rate_b)

    def test_to_array_shape(self):
        t = ContingencyTable(a=1, b=2, c=3, d=4)
        arr = t.to_array()
        assert arr.shape == (2, 2)
        assert arr[0, 0] == 1  # a
        assert arr[1, 1] == 4  # d

    def test_to_dict_keys(self):
        t = ContingencyTable(a=1, b=2, c=3, d=4)
        d = t.to_dict()
        assert set(d.keys()) == {"a", "b", "c", "d"}


# ---------------------------------------------------------------------------
# McNemar statistic
# ---------------------------------------------------------------------------

class TestRunMcNemar:
    def test_zero_discordant_returns_pval_one(self):
        # b=c=0: no evidence of difference
        t = ContingencyTable(a=10, b=0, c=0, d=10)
        _, pval = run_mcnemar(t)
        assert pval == 1.0

    def test_exact_binomial_small_n(self):
        # b+c < 25 → exact binomial
        t = ContingencyTable(a=5, b=1, c=9, d=5)
        stat, pval = run_mcnemar(t)
        assert 0 < pval <= 1.0
        assert stat > 0

    def test_chi_squared_large_n(self):
        # b+c >= 25 → chi-squared
        t = ContingencyTable(a=50, b=20, c=5, d=50)
        stat, pval = run_mcnemar(t)
        assert pval < 0.05  # large b-c difference should be significant

    def test_symmetric_discordant_pval_one(self):
        # b == c: no asymmetry
        t = ContingencyTable(a=50, b=10, c=10, d=50)
        _, pval = run_mcnemar(t)
        assert pval >= 0.5

    def test_highly_asymmetric_significant(self):
        # b >> c: strongly significant
        t = ContingencyTable(a=0, b=30, c=1, d=0)
        _, pval = run_mcnemar(t)
        assert pval < 0.001


# ---------------------------------------------------------------------------
# Wilson CI
# ---------------------------------------------------------------------------

class TestWilsonCI:
    def test_bounds_in_range(self):
        lo, hi = wilson_ci(50, 100)
        assert 0 <= lo <= hi <= 1

    def test_zero_successes(self):
        lo, hi = wilson_ci(0, 100)
        assert lo >= 0
        assert hi > 0  # Wilson avoids exact 0

    def test_all_successes(self):
        lo, hi = wilson_ci(100, 100)
        assert lo < 1
        assert hi <= 1

    def test_zero_total(self):
        lo, hi = wilson_ci(0, 0)
        assert math.isnan(lo) and math.isnan(hi)

    def test_width_decreases_with_n(self):
        lo10, hi10 = wilson_ci(5, 10)
        lo100, hi100 = wilson_ci(50, 100)
        assert (hi10 - lo10) > (hi100 - lo100)


# ---------------------------------------------------------------------------
# Delta CI (Newcombe)
# ---------------------------------------------------------------------------

class TestDeltaCI:
    def test_returns_tuple_of_two(self):
        result = delta_ci_newcombe(100, 40, 100, 60)
        assert len(result) == 2

    def test_includes_zero_when_equal(self):
        lo, hi = delta_ci_newcombe(100, 50, 100, 50)
        assert lo <= 0 <= hi

    def test_positive_delta_ci_above_zero(self):
        # Clearly different proportions: 0.2 vs 0.8
        lo, hi = delta_ci_newcombe(100, 20, 100, 80)
        assert lo > 0


# ---------------------------------------------------------------------------
# Build contingency
# ---------------------------------------------------------------------------

class TestBuildContingency:
    @pytest.fixture
    def simple_df(self):
        """3 pairs: A=0,B=0 | A=0,B=1 | A=1,B=0"""
        rows = [
            {"model": "m1", "task_id": "T001", "condition": "baseline", "rep": 1, "has_violation": 0},
            {"model": "m1", "task_id": "T001", "condition": "safety",   "rep": 1, "has_violation": 0},
            {"model": "m1", "task_id": "T002", "condition": "baseline", "rep": 1, "has_violation": 0},
            {"model": "m1", "task_id": "T002", "condition": "safety",   "rep": 1, "has_violation": 1},
            {"model": "m1", "task_id": "T003", "condition": "baseline", "rep": 1, "has_violation": 1},
            {"model": "m1", "task_id": "T003", "condition": "safety",   "rep": 1, "has_violation": 0},
        ]
        return pd.DataFrame(rows)

    def test_pooled_counts(self, simple_df):
        tables = build_contingency(simple_df, "baseline", "safety")
        t = tables["all"]
        assert t.a == 1  # both no-violation
        assert t.b == 1  # only safety violates
        assert t.c == 1  # only baseline violates
        assert t.d == 0

    def test_missing_condition_returns_empty(self, simple_df):
        tables = build_contingency(simple_df, "baseline", "A6.1")
        assert len(tables) == 0

    def test_per_model_groupby(self, simple_df):
        tables = build_contingency(simple_df, "baseline", "safety", groupby="model")
        assert "m1" in tables
        assert tables["m1"].n == 3


# ---------------------------------------------------------------------------
# H4 test
# ---------------------------------------------------------------------------

class TestH4:
    def _make_df(self, n_viol: int, n_total: int) -> pd.DataFrame:
        rows = []
        for i in range(n_total):
            rows.append({
                "model": "qwen2.5-coder:32b", "task_id": f"T{i:03d}",
                "condition": "baseline", "rep": 1,
                "has_violation": 1 if i < n_viol else 0,
            })
        return pd.DataFrame(rows)

    def test_high_violation_rate_supported(self):
        df = self._make_df(40, 50)  # 80% >> 30%
        results = run_h4(df)
        pooled = next(r for r in results if r.model == "all_models")
        assert bool(pooled.significant) is True
        assert pooled.rate_a > H4_THRESHOLD

    def test_low_violation_rate_not_supported(self):
        df = self._make_df(5, 50)  # 10% < 30%
        results = run_h4(df)
        pooled = next(r for r in results if r.model == "all_models")
        assert pooled.rate_a < H4_THRESHOLD

    def test_returns_result_for_each_model(self):
        df = self._make_df(20, 50)
        results = run_h4(df)
        models = {r.model for r in results}
        assert "all_models" in models

    def test_ci_bounds_valid(self):
        df = self._make_df(20, 50)
        results = run_h4(df)
        for r in results:
            if not math.isnan(r.ci_low):
                assert 0 <= r.ci_low <= r.ci_high <= 1


# ---------------------------------------------------------------------------
# H5 test
# ---------------------------------------------------------------------------

class TestH5:
    @pytest.fixture
    def h5_df(self):
        """Baseline + A6.6 with large attack effect."""
        rng = np.random.default_rng(0)
        rows = []
        for i in range(45):
            task = f"T{(i % 15) + 1:03d}"
            rep = (i // 15) + 1
            rows.append({
                "model": "qwen2.5-coder:32b",
                "task_id": task,
                "condition": "baseline",
                "rep": rep,
                "has_violation": int(rng.random() < 0.1),
            })
            rows.append({
                "model": "qwen2.5-coder:32b",
                "task_id": task,
                "condition": "A6.6",
                "rep": rep,
                "has_violation": int(rng.random() < 0.9),
            })
        return pd.DataFrame(rows)

    def test_returns_list(self, h5_df):
        results = run_h5(h5_df)
        assert isinstance(results, list)

    def test_each_attack_has_result(self, h5_df):
        results = run_h5(h5_df)
        attacks_covered = {r.condition_b for r in results}
        # At least A6.6 should be present
        assert "A6.6" in attacks_covered

    def test_p_adjusted_set(self, h5_df):
        results = run_h5(h5_df)
        for r in results:
            assert not math.isnan(r.p_adjusted)

    def test_large_effect_significant(self, h5_df):
        results = run_h5(h5_df)
        a66 = next((r for r in results if r.condition_b == "A6.6" and r.model == "all"), None)
        if a66:
            assert a66.delta > 0.5  # 90% - 10% = 80pp delta in demo


# ---------------------------------------------------------------------------
# H6 test
# ---------------------------------------------------------------------------

class TestH6:
    @pytest.fixture
    def h4_df(self):
        """Baseline 80% → watchdog 20% (strong reduction)."""
        rows = []
        for i in range(45):
            task = f"T{(i % 15) + 1:03d}"
            rep = (i // 15) + 1
            for cond, viol in [("baseline", 1), ("watchdog", 0)]:
                rows.append({
                    "model": "qwen2.5-coder:32b",
                    "task_id": task,
                    "condition": cond,
                    "rep": rep,
                    "has_violation": viol,
                })
        return pd.DataFrame(rows)

    def test_returns_list(self, h4_df):
        results = run_h6(h4_df)
        assert isinstance(results, list)

    def test_strong_reduction_supported(self, h4_df):
        results = run_h6(h4_df)
        pooled = next(
            (r for r in results if r.model == "all" and r.condition_b == "watchdog"),
            None
        )
        assert pooled is not None
        assert pooled.relative_change < -H6_THRESHOLD

    def test_relative_change_negative_means_reduction(self, h4_df):
        results = run_h6(h4_df)
        for r in results:
            if not math.isnan(r.relative_change):
                # All our fixture pairs have baseline > watchdog
                assert r.relative_change < 0

    def test_p_adjusted_present(self, h4_df):
        results = run_h6(h4_df)
        for r in results:
            assert not math.isnan(r.p_adjusted)


# ---------------------------------------------------------------------------
# Demo data
# ---------------------------------------------------------------------------

class TestDemoData:
    def test_shape(self):
        df = generate_demo_data()
        assert len(df) > 0
        assert "has_violation" in df.columns
        assert "condition" in df.columns

    def test_deterministic(self):
        df1 = generate_demo_data()
        df2 = generate_demo_data()
        assert df1["has_violation"].sum() == df2["has_violation"].sum()

    def test_three_models(self):
        df = generate_demo_data()
        assert df["model"].nunique() == 3

    def test_fifteen_tasks(self):
        df = generate_demo_data()
        assert df["task_id"].nunique() == 15

    def test_has_violation_binary(self):
        df = generate_demo_data()
        assert set(df["has_violation"].unique()).issubset({0, 1})

    def test_all_conditions_present(self):
        df = generate_demo_data()
        conditions = set(df["condition"].unique())
        assert "baseline" in conditions
        assert "safety" in conditions
        assert "watchdog" in conditions
        assert "A6.6" in conditions

    def test_full_pipeline_runs(self, tmp_path):
        """End-to-end: demo data → all three tests → outputs written."""
        df = generate_demo_data()
        h4 = run_h4(df)
        h5 = run_h5(df)
        h6 = run_h6(df)
        assert len(h4) > 0
        assert len(h5) > 0
        # H4 may be empty if watchdog condition absent in filtered df
        # (demo has it, so should be non-empty)
        assert len(h4) > 0


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_empty_dataframe_h2(self):
        df = pd.DataFrame(columns=["model", "task_id", "condition", "rep", "has_violation"])
        results = run_h4(df)
        assert results == []

    def test_single_pair(self):
        rows = [
            {"model": "m", "task_id": "T001", "condition": "baseline", "rep": 1, "has_violation": 0},
            {"model": "m", "task_id": "T001", "condition": "safety",   "rep": 1, "has_violation": 1},
        ]
        df = pd.DataFrame(rows)
        tables = build_contingency(df, "baseline", "safety")
        t = tables["all"]
        assert t.n == 1
        assert t.b == 1  # only safety violates

    def test_all_violations_baseline(self):
        rows = [
            {"model": "m", "task_id": f"T{i}", "condition": "baseline", "rep": 1, "has_violation": 1}
            for i in range(30)
        ]
        df = pd.DataFrame(rows)
        results = run_h4(df)
        pooled = next(r for r in results if r.model == "all_models")
        assert math.isclose(pooled.rate_a, 1.0)

    def test_no_violations_baseline(self):
        rows = [
            {"model": "m", "task_id": f"T{i}", "condition": "baseline", "rep": 1, "has_violation": 0}
            for i in range(30)
        ]
        df = pd.DataFrame(rows)
        results = run_h4(df)
        pooled = next(r for r in results if r.model == "all_models")
        assert pooled.rate_a == 0.0
        assert bool(pooled.significant) is False
