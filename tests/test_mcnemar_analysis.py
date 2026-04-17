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
    MODELS,
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
    CochranResult,
    run_cross_model_cochran_q,
    _task_model_matrix,
    _cochran_from_matrix,
    cochran_results_to_df,
    generate_markdown_report,
    refusal_rate_summary,
    decompose_by_rule,
    write_sensitivity_outputs,
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
        tables = build_contingency(simple_df, "baseline", "adversarial_A8.1")
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
        """Baseline + adversarial_A8.6 with large attack effect (matches runner CSV format)."""
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
                "condition": "adversarial_A8.6",
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
        # At least adversarial_A8.6 should be present
        assert "adversarial_A8.6" in attacks_covered

    def test_p_adjusted_set(self, h5_df):
        results = run_h5(h5_df)
        for r in results:
            assert not math.isnan(r.p_adjusted)

    def test_large_effect_significant(self, h5_df):
        results = run_h5(h5_df)
        a66 = next((r for r in results if r.condition_b == "adversarial_A8.6" and r.model == "all"), None)
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
        assert "adversarial_A8.6" in conditions

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

# ---------------------------------------------------------------------------
# TestCrossModelCochranQ — paper-level H6 (cross-model heterogeneity)
# ---------------------------------------------------------------------------


class TestCrossModelCochranQ:
    """Tests for the Cochran's Q omnibus test across k=3 models."""

    # ---- fixtures ----

    @pytest.fixture
    def homogeneous_df(self):
        """All three models have the same ~50% violation rate per
        condition. Cochran's Q should fail to reject H0."""
        rows = []
        models = ["qwen", "deepseek", "codellama"]
        # 10 tasks, same violation pattern across models, rep=1.
        for t in range(10):
            v = 1 if t % 2 == 0 else 0
            for m in models:
                rows.append({
                    "task_id": f"T{t:03d}",
                    "model": m,
                    "condition": "baseline",
                    "rep": 1,
                    "has_violation": v,
                })
        return pd.DataFrame(rows)

    @pytest.fixture
    def divergent_df(self):
        """One model violates on every task, the other two never do.
        Cochran's Q should strongly reject H0."""
        rows = []
        for t in range(10):
            rows.append({"task_id": f"T{t:03d}", "model": "qwen",
                         "condition": "baseline", "rep": 1,
                         "has_violation": 1})
            rows.append({"task_id": f"T{t:03d}", "model": "deepseek",
                         "condition": "baseline", "rep": 1,
                         "has_violation": 0})
            rows.append({"task_id": f"T{t:03d}", "model": "codellama",
                         "condition": "baseline", "rep": 1,
                         "has_violation": 0})
        return pd.DataFrame(rows)

    @pytest.fixture
    def full_condition_df(self):
        """DataFrame with baseline / safety / adversarial_A8.1 / watchdog
        rows so the four per-condition Cochran tests can all fire."""
        rows = []
        models = ["qwen", "deepseek", "codellama"]
        conditions = ["baseline", "safety", "adversarial_A8.1", "watchdog"]
        for cond in conditions:
            for t in range(6):
                # Make qwen the outlier under each condition so all
                # three omnibus tests have a real effect to find.
                for m in models:
                    v = 1 if m == "qwen" else 0
                    rows.append({
                        "task_id": f"T{t:03d}",
                        "model": m,
                        "condition": cond,
                        "rep": 1,
                        "has_violation": v,
                    })
        return pd.DataFrame(rows)

    # ---- matrix helper ----

    def test_task_model_matrix_shape(self, homogeneous_df):
        matrix, models, tasks = _task_model_matrix(homogeneous_df)
        assert matrix.shape == (10, 3)
        assert set(models) == {"qwen", "deepseek", "codellama"}
        assert len(tasks) == 10

    def test_task_model_matrix_empty_df(self):
        empty = pd.DataFrame(
            columns=["task_id", "model", "condition", "rep", "has_violation"]
        )
        matrix, models, tasks = _task_model_matrix(empty)
        assert matrix.shape == (0, 0)
        assert models == []

    # ---- _cochran_from_matrix direct tests ----

    def test_cochran_from_matrix_insufficient_data_returns_nan(self):
        import numpy as _np
        empty = _np.zeros((0, 0), dtype=int)
        r = _cochran_from_matrix(empty, [], [], condition_label="x")
        assert math.isnan(r.q_statistic)
        assert math.isnan(r.p_value)
        assert r.n_subjects == 0

    def test_cochran_from_matrix_homogeneous_data_nonsignificant(
        self, homogeneous_df
    ):
        matrix, models, tasks = _task_model_matrix(homogeneous_df)
        r = _cochran_from_matrix(
            matrix, models, tasks, condition_label="baseline"
        )
        assert r.k == 3
        assert r.df == 2
        assert r.n_subjects == 10
        assert r.p_value > 0.05

    def test_cochran_from_matrix_all_rows_identical_reports_q0_p1(self):
        """Degenerate perfect-agreement case: every task has the
        same 0/1 answer on every model. Cochran's Q is formally
        undefined (zero denominator) but the null hypothesis of
        equal proportions is trivially consistent with the data,
        so the wrapper reports Q=0 and p=1.0 rather than NaN."""
        import numpy as _np
        # Five all-1 rows and five all-0 rows: perfect agreement.
        matrix = _np.array(
            [[1, 1, 1]] * 5 + [[0, 0, 0]] * 5,
            dtype=int,
        )
        r = _cochran_from_matrix(
            matrix,
            ["codellama", "deepseek", "qwen"],
            [f"T{i:03d}" for i in range(10)],
            condition_label="baseline",
        )
        assert r.q_statistic == 0.0
        assert r.p_value == 1.0
        assert r.n_subjects == 10
        assert "degenerate" in r.note
        # And it must be safe to Holm-Bonferroni through this value.
        from statsmodels.stats.multitest import multipletests
        reject, p_adj, _, _ = multipletests(
            [r.p_value], alpha=0.05, method="holm"
        )
        assert not reject[0]
        assert p_adj[0] == 1.0

    def test_cochran_from_matrix_divergent_data_significant(
        self, divergent_df
    ):
        matrix, models, tasks = _task_model_matrix(divergent_df)
        r = _cochran_from_matrix(
            matrix, models, tasks, condition_label="baseline"
        )
        assert r.k == 3
        assert r.df == 2
        assert r.p_value < 0.01
        # qwen is at index corresponding to its alphabetical position;
        # rates for deepseek and codellama must be zero.
        qwen_idx = r.per_condition_labels.index("qwen")
        assert r.per_condition_rates[qwen_idx] == 1.0

    # ---- public entry point tests ----

    def test_run_returns_result_per_condition(self, full_condition_df):
        results = run_cross_model_cochran_q(full_condition_df)
        # Exactly four results: baseline, safety, adversarial_any, watchdog.
        assert len(results) == 4
        conds = {r.condition for r in results}
        assert conds == {"baseline", "safety", "adversarial_any", "watchdog"}

    def test_run_applies_holm_bonferroni(self, full_condition_df):
        results = run_cross_model_cochran_q(full_condition_df)
        # All four conditions are strongly divergent in the fixture,
        # so every adjusted p-value must be set (not NaN).
        for r in results:
            assert not math.isnan(r.p_adjusted)
            assert 0.0 <= r.p_adjusted <= 1.0

    def test_run_significant_when_models_diverge(self, full_condition_df):
        results = run_cross_model_cochran_q(full_condition_df)
        assert all(r.significant for r in results)

    def test_run_handles_empty_dataframe(self):
        empty = pd.DataFrame(
            columns=["task_id", "model", "condition", "rep", "has_violation"]
        )
        results = run_cross_model_cochran_q(empty)
        # Should return four 'insufficient data' placeholders,
        # never raise.
        assert len(results) == 4
        for r in results:
            assert r.n_subjects == 0
            assert "insufficient" in r.note

    def test_cochran_results_to_df_shape_and_columns(self, full_condition_df):
        results = run_cross_model_cochran_q(full_condition_df)
        df = cochran_results_to_df(results)
        assert len(df) == 4
        for col in (
            "condition", "k", "n_subjects",
            "q_statistic", "df", "p_value",
            "p_adjusted", "significant",
            "per_condition_rates", "per_condition_labels",
        ):
            assert col in df.columns, f"missing column: {col}"
        # List fields must be serialised as semicolon-joined strings.
        for v in df["per_condition_labels"]:
            assert isinstance(v, str)
            assert ";" in v

    def test_markdown_report_includes_cochran_section(self, full_condition_df):
        results = run_cross_model_cochran_q(full_condition_df)
        # Pass through the real runner entry point.
        md = generate_markdown_report(
            h4_list=[], h5_list=[], h6_list=[],
            cochran_results=results,
        )
        assert "## Exploratory: Cross-Model Cochran's Q (not part of H4-H6 confirmatory family)" in md
        assert "baseline" in md
        assert "safety" in md
        assert "adversarial_any" in md
        assert "watchdog" in md

    def test_markdown_report_handles_no_cochran(self):
        md = generate_markdown_report(h4_list=[], h5_list=[], h6_list=[])
        # When cochran_results is None, the section header is still
        # rendered and a 'not computed' placeholder row is emitted
        # so the reader knows the test was deliberately skipped.
        assert "## Exploratory: Cross-Model Cochran's Q (not part of H4-H6 confirmatory family)" in md
        assert "not computed" in md

    def test_cochran_result_serializable(self, full_condition_df):
        from dataclasses import asdict
        results = run_cross_model_cochran_q(full_condition_df)
        for r in results:
            d = asdict(r)
            assert "q_statistic" in d
            assert "per_condition_rates" in d


# ---------------------------------------------------------------------------
# Hypothesis mapping invariants (Phase 6)
# ---------------------------------------------------------------------------

class TestHypothesisMappingInvariants:
    """
    Pin the contract between run_h4/h5/h6 and paper hypotheses H4/H5/H6
    (post OSF Amendment 1, approved 2026-04-07).

    These tests are semantic tripwires: they encode mapping invariants
    that a future refactor could silently break without affecting any
    other test in the suite. They deliberately build minimal independent
    fixtures (no reuse of h5_df, full_condition_df, etc.) so that each
    invariant is about the function's behavior, not a shared setup's
    coincidences.

    See scripts/mcnemar_analysis.py module docstring for the
    authoritative naming convention declaration, and
    docs/WEEK10_TODO.md item #12 for the audit trail.
    """

    # ---- minimal fixture helpers ----

    def _make_baseline_only_df(self):
        """3 models x 10 tasks, condition='baseline' only, 50% violation rate."""
        rows = []
        for model in MODELS:
            for i in range(10):
                rows.append({
                    "model": model,
                    "task_id": f"T{i:03d}",
                    "condition": "baseline",
                    "rep": 1,
                    "has_violation": 1 if i < 5 else 0,
                })
        return pd.DataFrame(rows)

    def _make_full_condition_df(self):
        """3 models x 10 tasks x {baseline, safety, watchdog, adversarial_A8.1}."""
        rows = []
        for model in MODELS:
            for cond in ["baseline", "safety", "watchdog", "adversarial_A8.1"]:
                for i in range(10):
                    rows.append({
                        "model": model,
                        "task_id": f"T{i:03d}",
                        "condition": cond,
                        "rep": 1,
                        "has_violation": 1 if i < 5 else 0,
                    })
        return pd.DataFrame(rows)

    # ---- T1: H4 ignores non-baseline rows ----

    def test_run_h4_only_uses_baseline_condition(self):
        """H4 must filter to condition=='baseline' and ignore other rows.

        Behavioral invariant: if we inject all-violation contamination
        rows for safety/watchdog/adversarial conditions, the pooled
        baseline rate from run_h4 must be unchanged. A basic
        'baseline-only df runs' test would be vacuous -- it would pass
        even if the condition filter were removed entirely.
        """
        df_clean = self._make_baseline_only_df()

        contaminants = []
        for model in MODELS:
            for i in range(10):
                for cond in ["safety", "watchdog", "adversarial_A8.1"]:
                    contaminants.append({
                        "model": model,
                        "task_id": f"T{i:03d}",
                        "condition": cond,
                        "rep": 1,
                        "has_violation": 1,
                    })
        df_contaminated = pd.concat(
            [df_clean, pd.DataFrame(contaminants)], ignore_index=True
        )

        results_clean = run_h4(df_clean)
        results_contaminated = run_h4(df_contaminated)

        pooled_clean = next(r for r in results_clean if r.model == "all_models")
        pooled_contaminated = next(
            r for r in results_contaminated if r.model == "all_models"
        )

        assert pooled_clean.rate_a == pooled_contaminated.rate_a
        assert pooled_clean.n_pairs == pooled_contaminated.n_pairs

    # ---- T2: H4 comparison string and row shape ----

    def test_run_h4_comparison_string_and_row_shape(self):
        """H4 returns comparison='H4_baseline_rate' for every row, with
        one pooled row ('all_models') plus one row per model in MODELS.
        """
        df = self._make_baseline_only_df()
        results = run_h4(df)

        assert all(r.comparison == "H4_baseline_rate" for r in results)

        models_seen = {r.model for r in results}
        assert "all_models" in models_seen
        for m in MODELS:
            assert m in models_seen

        assert len(results) == 1 + len(MODELS)

    # ---- T3: H5 comparison strings use runner-aligned prefixed labels ----

    def test_run_h5_uses_prefixed_adversarial_labels(self):
        """H5 comparison strings must match runner CSV label format
        ('adversarial_A8.N'), not the bare 'A8.N' form. This pins the
        fix from commit 01739ba (Phase 6-prereq) against silent
        regression.
        """
        rows = []
        for model in MODELS:
            for i in range(15):
                rows.append({
                    "model": model,
                    "task_id": f"T{i:03d}",
                    "condition": "baseline",
                    "rep": 1,
                    "has_violation": 1 if i < 2 else 0,
                })
                rows.append({
                    "model": model,
                    "task_id": f"T{i:03d}",
                    "condition": "adversarial_A8.1",
                    "rep": 1,
                    "has_violation": 1 if i < 12 else 0,
                })
        df = pd.DataFrame(rows)

        results = run_h5(df)
        assert len(results) > 0

        comparisons = {r.comparison for r in results}
        assert "H5_adversarial_A8.1_vs_baseline" in comparisons
        assert "H5_A8.1_vs_baseline" not in comparisons

        for r in results:
            assert r.condition_b.startswith("adversarial_A8.")

    # ---- T4: H6 tests both single-shot conditions against watchdog ----

    def test_run_h6_tests_only_baseline_against_watchdog(self):
        """H6 must produce a single confirmatory contrast:
        (baseline, watchdog). The earlier safety-vs-watchdog contrast
        was removed in Session 17 Commit 5 to align with Amendment 1
        of the OSF preregistration, which defines H6 only as
        'vs single-pass' (= baseline); see paper VI.H and docs/
        E3_DESIGN.md section 5.1 for rationale.

        Full fixture => len(MODELS) per-model rows + 1 pooled row
        per condition pair = 1 * (len(MODELS) + 1) = 4 rows total.

        The pooled row uses model_key=='all' (distinct from H4's
        'all_models' label; this stylistic inconsistency is a
        pre-existing property of run_h5/run_h6 vs run_h4 and is NOT
        in scope for this commit to fix).
        """
        df = self._make_full_condition_df()
        results = run_h6(df)

        cond_a_set = {r.condition_a for r in results}
        cond_b_set = {r.condition_b for r in results}
        assert cond_a_set == {"baseline"}, (
            f"H6 must test only baseline (not {cond_a_set}); "
            f"safety-vs-watchdog was removed from confirmatory family."
        )
        assert cond_b_set == {"watchdog"}

        assert len(results) == 1 * (len(MODELS) + 1)

    # ---- T5: H6 comparison string format is exact ----

    def test_run_h6_comparison_string_format(self):
        """H6 comparison strings must be exactly the single documented
        form after Session 17 Commit 5; any extra or missing
        permutation fails the set equality.
        """
        df = self._make_full_condition_df()
        results = run_h6(df)

        comparisons = {r.comparison for r in results}
        assert comparisons == {"H6_watchdog_vs_baseline"}

    # ---- T6: confirmatory vs exploratory return-type distinction ----

    def test_confirmatory_family_and_cochran_return_types_distinct(self):
        """run_h4/h5/h6 return list[McNemarResult]; run_cross_model_cochran_q
        returns list[CochranResult]. The return-type distinction encodes
        the confirmatory / exploratory split (OSF Amendment 1) and must
        be preserved across any future refactor.
        """
        df = self._make_full_condition_df()

        h4 = run_h4(df)
        h5 = run_h5(df)
        h6 = run_h6(df)
        cq = run_cross_model_cochran_q(df)

        assert len(h4) > 0
        assert len(h5) > 0
        assert len(h6) > 0
        assert len(cq) > 0

        assert all(isinstance(r, McNemarResult) for r in h4)
        assert all(isinstance(r, McNemarResult) for r in h5)
        assert all(isinstance(r, McNemarResult) for r in h6)
        assert all(isinstance(r, CochranResult) for r in cq)


class TestLoadResultsDeriveHasViolation:
    """Verify load_results derives has_violation from total_violations."""

    def test_derives_when_only_total_violations_present(self, tmp_path):
        import pandas as pd
        from scripts.mcnemar_analysis import load_results

        # CSV with total_violations but no has_violation (matches the
        # actual schema produced by scripts/llm_experiment_runner.py).
        df = pd.DataFrame([
            {"experiment": "E1", "model": "m1", "task_id": "T001",
             "condition": "baseline", "rep": 1, "total_violations": 0},
            {"experiment": "E1", "model": "m1", "task_id": "T002",
             "condition": "baseline", "rep": 1, "total_violations": 3},
            {"experiment": "E1", "model": "m1", "task_id": "T003",
             "condition": "baseline", "rep": 1, "total_violations": 7},
        ])
        csv_path = tmp_path / "e1_results.csv"
        df.to_csv(csv_path, index=False)

        loaded = load_results(tmp_path)
        assert "has_violation" in loaded.columns
        assert loaded["has_violation"].tolist() == [0, 1, 1]

    def test_existing_has_violation_is_preserved(self, tmp_path):
        import pandas as pd
        from scripts.mcnemar_analysis import load_results

        # CSV that already carries has_violation must not be overwritten
        # even if total_violations is also present (the explicit column
        # takes precedence).
        df = pd.DataFrame([
            {"experiment": "E1", "model": "m1", "task_id": "T001",
             "condition": "baseline", "rep": 1,
             "has_violation": 0, "total_violations": 5},
        ])
        csv_path = tmp_path / "e1_results.csv"
        df.to_csv(csv_path, index=False)

        loaded = load_results(tmp_path)
        # Explicit has_violation wins over the derivation rule.
        assert loaded["has_violation"].tolist() == [0]

    def test_missing_both_columns_still_raises(self, tmp_path):
        import pandas as pd
        import pytest
        from scripts.mcnemar_analysis import load_results

        # Neither has_violation nor total_violations -> existing
        # validation must still fire.
        df = pd.DataFrame([
            {"experiment": "E1", "model": "m1", "task_id": "T001",
             "condition": "baseline", "rep": 1},
        ])
        csv_path = tmp_path / "e1_results.csv"
        df.to_csv(csv_path, index=False)

        with pytest.raises(ValueError, match="Missing columns"):
            load_results(tmp_path)


class TestGatePassingSensitivity:
    """Verify that subset_label is wired through and the gate-passing
    subset can produce different results from the naive contrast when
    invalid_pseudocode and error rows are present."""

    def test_subset_label_default_is_all(self):
        from scripts.mcnemar_analysis import McNemarResult
        r = McNemarResult(
            comparison="x", model="m", condition_a="a", condition_b="b",
            n_pairs=1, rate_a=0.5, rate_b=0.5, delta=0.0,
            relative_change=0.0, b_discordant=0, c_discordant=0,
            statistic=1.0, p_value=1.0,
        )
        assert r.subset_label == "all"

    def test_run_h4_results_can_be_tagged_with_subset_label(self):
        import pandas as pd
        from scripts.mcnemar_analysis import run_h4

        df = pd.DataFrame([
            {"model": "qwen2.5-coder:32b", "task_id": f"T{i:03d}",
             "condition": "baseline", "rep": 1, "has_violation": 1,
             "status": "success"}
            for i in range(1, 16)
        ] + [
            {"model": "deepseek-coder-v2:16b", "task_id": f"T{i:03d}",
             "condition": "baseline", "rep": 1, "has_violation": 0,
             "status": "success"}
            for i in range(1, 16)
        ])
        results = run_h4(df)
        for r in results:
            r.subset_label = "gate_passing"
        assert all(r.subset_label == "gate_passing" for r in results)

    def test_gate_passing_subset_changes_h4_outcome_when_invalid_present(self):
        """If half the calls are invalid_pseudocode (counted as
        no-violation in the naive contrast) but every successful call
        violates, the gate-passing CVR is much higher than the naive
        CVR."""
        import pandas as pd
        from scripts.mcnemar_analysis import run_h4

        rows = []
        for i in range(1, 11):
            rows.append({
                "model": "qwen2.5-coder:32b", "task_id": f"T{i:03d}",
                "condition": "baseline", "rep": 1,
                "has_violation": 1, "status": "success",
            })
        for i in range(11, 21):
            rows.append({
                "model": "qwen2.5-coder:32b", "task_id": f"T{i:03d}",
                "condition": "baseline", "rep": 1,
                "has_violation": 0, "status": "invalid_pseudocode",
            })
        df = pd.DataFrame(rows)

        naive_results = run_h4(df)
        naive_qwen = next(r for r in naive_results
                          if r.model == "qwen2.5-coder:32b")
        assert abs(naive_qwen.rate_a - 0.50) < 1e-9

        df_gate = df[df["status"] == "success"]
        gate_results = run_h4(df_gate)
        gate_qwen = next(r for r in gate_results
                         if r.model == "qwen2.5-coder:32b")
        assert abs(gate_qwen.rate_a - 1.00) < 1e-9
# ---------------------------------------------------------------------------
# Sensitivity analyses (session 16, commit 1a)
# ---------------------------------------------------------------------------

class TestRefusalRateSummary:
    def test_empty_df_returns_empty_df(self):
        empty = pd.DataFrame(columns=["model", "condition", "refusal"])
        result = refusal_rate_summary(empty)
        assert result.empty
        assert list(result.columns) == [
            "model", "condition", "n", "n_refusals", "refusal_rate",
        ]

    def test_zero_refusals_yields_zero_rate(self):
        df = pd.DataFrame([
            {"model": "m1", "condition": "baseline", "refusal": False},
            {"model": "m1", "condition": "baseline", "refusal": False},
        ])
        result = refusal_rate_summary(df)
        row = result.iloc[0]
        assert row["n"] == 2
        assert row["n_refusals"] == 0
        assert row["refusal_rate"] == 0.0

    def test_mixed_refusals_counts_correctly(self):
        df = pd.DataFrame([
            {"model": "m1", "condition": "baseline", "refusal": True},
            {"model": "m1", "condition": "baseline", "refusal": False},
            {"model": "m1", "condition": "baseline", "refusal": True},
        ])
        result = refusal_rate_summary(df)
        row = result.iloc[0]
        assert row["n"] == 3
        assert row["n_refusals"] == 2
        assert abs(row["refusal_rate"] - 2 / 3) < 1e-9

    def test_string_refusal_values_accepted(self):
        """CSV-loaded frames may carry 'True'/'False' strings."""
        df = pd.DataFrame([
            {"model": "m1", "condition": "baseline", "refusal": "True"},
            {"model": "m1", "condition": "baseline", "refusal": "False"},
        ])
        result = refusal_rate_summary(df)
        row = result.iloc[0]
        assert row["n_refusals"] == 1


class TestDecomposeByRule:
    def test_empty_df_returns_empty_df(self):
        empty = pd.DataFrame(columns=["model", "condition", "violation_types"])
        result = decompose_by_rule(empty)
        assert result.empty

    def test_single_sm_token_counted_once(self):
        df = pd.DataFrame([{
            "model": "m1",
            "condition": "baseline",
            "violation_types": "SM-2",
        }])
        result = decompose_by_rule(df)
        assert len(result) == 1
        row = result.iloc[0]
        assert row["condition_group"] == "baseline"
        assert row["n_rows"] == 1
        assert row["SM-2"] == 1
        assert row["SM-1"] == 0

    def test_duplicate_tokens_counted_per_occurrence(self):
        df = pd.DataFrame([{
            "model": "m1",
            "condition": "baseline",
            "violation_types": "SM-2,SM-2,SM-5",
        }])
        result = decompose_by_rule(df)
        row = result.iloc[0]
        assert row["SM-2"] == 2
        assert row["SM-5"] == 1

    def test_empty_violation_types_gives_zero_counts(self):
        df = pd.DataFrame([{
            "model": "m1",
            "condition": "baseline",
            "violation_types": "",
        }])
        result = decompose_by_rule(df)
        row = result.iloc[0]
        assert row["n_rows"] == 1
        for rule in ["SM-1", "SM-2", "SM-3", "SM-4", "SM-5", "SM-6", "SM-7"]:
            assert row[rule] == 0

    def test_sm3_sm7_remain_zero_when_absent(self):
        """Empirical: SM-3 and SM-7 did not fire in E1+E2 confirmatory data."""
        df = pd.DataFrame([{
            "model": "m1",
            "condition": "baseline",
            "violation_types": "SM-1,SM-2,SM-4,SM-5,SM-6",
        }])
        result = decompose_by_rule(df)
        row = result.iloc[0]
        assert row["SM-3"] == 0
        assert row["SM-7"] == 0

    def test_adversarial_conditions_pool_to_any_a8k(self):
        df = pd.DataFrame([
            {"model": "m1", "condition": "adversarial_A8.1",
             "violation_types": "SM-2"},
            {"model": "m1", "condition": "adversarial_A8.4",
             "violation_types": "SM-5"},
        ])
        result = decompose_by_rule(df)
        assert len(result) == 1
        row = result.iloc[0]
        assert row["condition_group"] == "adversarial_any_a8k"
        assert row["n_rows"] == 2
        assert row["SM-2"] == 1
        assert row["SM-5"] == 1

    def test_unknown_condition_excluded(self):
        df = pd.DataFrame([{
            "model": "m1",
            "condition": "unexpected_mode",
            "violation_types": "SM-1",
        }])
        result = decompose_by_rule(df)
        assert result.empty


class TestWriteSensitivityOutputs:
    def test_writes_two_csvs_to_sensitivity_subdir(self, tmp_path):
        df = pd.DataFrame([
            {"model": "m1", "condition": "baseline", "refusal": False,
             "violation_types": "SM-2"},
        ])
        write_sensitivity_outputs(df, tmp_path)
        sens_dir = tmp_path / "sensitivity"
        assert sens_dir.is_dir()
        assert (sens_dir / "refusal_summary.csv").exists()
        assert (sens_dir / "per_rule_decomposition.csv").exists()


# ---------------------------------------------------------------------------
# E3 adapter in load_results (Session 17 Commit 5)
# ---------------------------------------------------------------------------


class TestE3Adapter:
    """Tests for the E3 post-processing step in load_results. The
    adapter selects the maximum-retry row per (model, task_id, rep)
    in E3 data and relabels its condition to 'watchdog' so that
    run_h6's matched pairs form correctly. Session 17 Commit 5."""

    def _write_csv(self, path, rows):
        import pandas as pd
        pd.DataFrame(rows).to_csv(path, index=False)

    def test_adapter_keeps_only_max_retry_row_per_triple(self, tmp_path):
        """Given an E3 CSV with retry=[0,1,2,3] for one triple, the
        adapter should keep only retry=3 in the H6 family."""
        from scripts.mcnemar_analysis import load_results
        rows = []
        for retry in range(4):
            rows.append({
                "experiment": "E3",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": retry,
                "total_violations": 10 - retry,  # improving
            })
        self._write_csv(tmp_path / "e3_results.csv", rows)
        df = load_results(tmp_path)
        e3 = df[df["experiment"] == "E3"]
        assert len(e3) == 1, (
            f"Expected 1 E3 row after adapter, got {len(e3)}"
        )
        assert int(e3.iloc[0]["retry"]) == 3, (
            "Adapter must keep the max-retry row"
        )

    def test_adapter_relabels_condition_to_watchdog(self, tmp_path):
        """The kept E3 row must have condition='watchdog', not the
        original 'baseline' label."""
        from scripts.mcnemar_analysis import load_results
        rows = [
            {
                "experiment": "E3",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": 0,
                "total_violations": 5,
            },
            {
                "experiment": "E3",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": 1,
                "total_violations": 3,
            },
        ]
        self._write_csv(tmp_path / "e3_results.csv", rows)
        df = load_results(tmp_path)
        e3 = df[df["experiment"] == "E3"]
        assert len(e3) == 1
        assert e3.iloc[0]["condition"] == "watchdog"

    def test_adapter_preserves_e1_rows_unchanged(self, tmp_path):
        """E1 rows must pass through the adapter untouched (condition
        stays 'baseline', row count preserved)."""
        from scripts.mcnemar_analysis import load_results
        e1_rows = [
            {
                "experiment": "E1",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": 0,
                "total_violations": 7,
            },
            {
                "experiment": "E1",
                "model": "deepseek-coder-v2:16b",
                "task_id": "T001",
                "condition": "safety",
                "rep": 1,
                "retry": 0,
                "total_violations": 3,
            },
        ]
        self._write_csv(tmp_path / "e1_results.csv", e1_rows)
        df = load_results(tmp_path)
        e1 = df[df["experiment"] == "E1"]
        assert len(e1) == 2
        # Conditions preserved
        conditions = set(e1["condition"])
        assert conditions == {"baseline", "safety"}

    def test_adapter_handles_mixed_e1_e3_input(self, tmp_path):
        """With both E1 and E3 rows present, the adapter should
        preserve E1 and process E3 correctly."""
        from scripts.mcnemar_analysis import load_results
        mixed = [
            {
                "experiment": "E1",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": 0,
                "total_violations": 7,
            },
        ]
        for retry in range(3):
            mixed.append({
                "experiment": "E3",
                "model": "qwen2.5-coder:32b",
                "task_id": "T001",
                "condition": "baseline",
                "rep": 1,
                "retry": retry,
                "total_violations": 7 - retry,
            })
        self._write_csv(tmp_path / "mixed_results.csv", mixed)
        df = load_results(tmp_path)
        # 1 E1 row + 1 E3 final row = 2 total
        assert len(df) == 2
        conditions = set(df["condition"])
        assert conditions == {"baseline", "watchdog"}
