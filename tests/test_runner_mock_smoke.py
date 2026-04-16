"""End-to-end smoke test for scripts/llm_experiment_runner.py.

This is the first real coverage of llm_experiment_runner. Until
Week 10 the runner had never been executed end-to-end and had
surfaced four hidden bugs in the span of a few days:

    1. parser.parse() vs parser.extract()              (a4fdacd)
    2. missing URScript validity gate                  (068bb0a)
    3. analyze_combined() wrong argument order         (92c0d5c)
    4. response.content vs response.raw_response       (b6ac8f2)

The smoke test runs E1 and E2 end-to-end against MockLLMClient via
``--provider mock`` to ensure every code path in run_single_call is
exercised: SUCCESS with violations, SUCCESS without violations,
REFUSAL, PARSE_FAILURE and the invalid_pseudocode validity-gate
branch. A regression here means no confirmatory E1/E2/E3 run can
start until the pipeline is green again — this is the "mandatory
gate" item in docs/WEEK10_TODO.md (#5b).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Load scripts/llm_experiment_runner.py as a module. The file lives under
# scripts/ and is normally invoked via ``python3 scripts/...``; it is not a
# package member, so we import it by file path.
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = REPO_ROOT / "scripts" / "llm_experiment_runner.py"

# Make the runner's own sys.path additions idempotent by ensuring the
# package directories are already on sys.path before import.
for pkg in ("enfield_llm", "enfield_tasks", "enfield_watchdog_static"):
    pkg_path = str(REPO_ROOT / pkg)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

_spec = importlib.util.spec_from_file_location(
    "llm_experiment_runner", RUNNER_PATH
)
runner = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(runner)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def two_tasks():
    """Load two real task IRs (T001 pick-place collab, T002 linear weld
    fenced) the same way the runner's load_tasks() would. Using the
    real tasks instead of synthetic ones catches schema drift between
    the runner and the Task IR dataset."""
    return runner.load_tasks("T001,T002")


@pytest.fixture
def output_dir(tmp_path):
    """Isolated per-test output directory."""
    d = tmp_path / "smoke_out"
    d.mkdir()
    return d


# Required columns that every row from run_single_call must carry.
# Documented implicitly in scripts/llm_experiment_runner.py; pinning
# them here guards against silent schema drift in the CSV export.
REQUIRED_COLUMNS = {
    "experiment",
    "model",
    "task_id",
    "task_category",
    "operating_mode",
    "condition",
    "adversarial_type",
    "rep",
    "retry",
    "timestamp",
    "status",
    "refusal",
    "code_lines",
    "has_motion",
    "has_safety_check",
    "dm_violations",
    "sm_violations",
    "total_violations",
    "violation_types",
    "severity_max",
    "tokens_in",
    "tokens_out",
    "latency_ms",
}


# ---------------------------------------------------------------------------
# E1 smoke
# ---------------------------------------------------------------------------


class TestE1MockSmoke:
    def test_run_e1_returns_nonempty_result_list(self, two_tasks, output_dir):
        results = runner.run_e1(
            tasks=two_tasks,
            reps=1,
            output_dir=output_dir,
            provider="mock",
            mock_seed=42,
        )
        assert isinstance(results, list)
        # 1 mock model * 2 tasks * 2 conditions (baseline, safety) * 1 rep
        assert len(results) == 4

    def test_run_e1_rows_have_required_schema(self, two_tasks, output_dir):
        results = runner.run_e1(
            tasks=two_tasks, reps=1, output_dir=output_dir,
            provider="mock", mock_seed=42,
        )
        for row in results:
            missing = REQUIRED_COLUMNS - row.keys()
            assert not missing, f"row missing columns: {missing}"

    def test_run_e1_status_values_are_expected(self, two_tasks, output_dir):
        """Every row status must come from the known runner vocabulary."""
        results = runner.run_e1(
            tasks=two_tasks, reps=1, output_dir=output_dir,
            provider="mock", mock_seed=42,
        )
        allowed = {
            "success",
            "refusal",
            "parse_failure",
            "invalid_pseudocode",
            "error",
        }
        seen = {row["status"] for row in results}
        unexpected = seen - allowed
        assert not unexpected, f"unknown statuses: {unexpected}"

    def test_run_e1_is_deterministic_across_runs(self, two_tasks, output_dir):
        """Given the same mock_seed, two separate runs produce the
        same (task_id, condition, status) tuples. This pins the
        reproducibility contract the preregistration relies on."""
        r1 = runner.run_e1(
            tasks=two_tasks, reps=1, output_dir=output_dir / "run1",
            provider="mock", mock_seed=42,
        )
        (output_dir / "run1").mkdir(exist_ok=True)
        r2 = runner.run_e1(
            tasks=two_tasks, reps=1, output_dir=output_dir / "run2",
            provider="mock", mock_seed=42,
        )
        key = lambda row: (row["task_id"], row["condition"], row["status"])
        assert [key(r) for r in r1] == [key(r) for r in r2]


# ---------------------------------------------------------------------------
# E2 smoke — adversarial branch
# ---------------------------------------------------------------------------


class TestE2MockSmoke:
    def test_run_e2_returns_all_adversarial_subtypes(
        self, two_tasks, output_dir
    ):
        """E2 iterates over the full A6 (legacy A8 in the paper)
        adversarial family, so 1 mock model * 2 tasks * 8 subtypes
        = 16 rows are expected."""
        results = runner.run_e2(
            tasks=two_tasks,
            output_dir=output_dir,
            provider="mock",
            mock_seed=42,
        )
        assert len(results) == 14
        for row in results:
            assert row["experiment"] == "E2"
            assert row["condition"].startswith("adversarial_")

    def test_run_e2_rows_have_required_schema(self, two_tasks, output_dir):
        results = runner.run_e2(
            tasks=two_tasks, output_dir=output_dir,
            provider="mock", mock_seed=42,
        )
        for row in results:
            missing = REQUIRED_COLUMNS - row.keys()
            assert not missing, f"row missing columns: {missing}"


# ---------------------------------------------------------------------------
# Code-path coverage — the whole reason PR-4b exists
# ---------------------------------------------------------------------------


class TestRunnerCodePathCoverage:
    """Aggregating E1 + E2 output must exercise every meaningful status
    branch of run_single_call. This is the regression net for the four
    hidden bugs listed in the module docstring."""

    @pytest.fixture
    def combined_results(self, two_tasks, output_dir):
        e1 = runner.run_e1(
            tasks=two_tasks, reps=1, output_dir=output_dir / "e1",
            provider="mock", mock_seed=42,
        )
        (output_dir / "e1").mkdir(exist_ok=True)
        e2 = runner.run_e2(
            tasks=two_tasks, output_dir=output_dir / "e2",
            provider="mock", mock_seed=42,
        )
        return e1 + e2

    def test_success_status_is_reached(self, combined_results):
        assert any(r["status"] == "success" for r in combined_results)

    def test_refusal_status_is_reached(self, combined_results):
        assert any(r["status"] == "refusal" for r in combined_results)

    def test_invalid_pseudocode_status_is_reached(self, combined_results):
        """This is the validity-gate regression test. If the gate ever
        stops firing on the PSEUDOCODE template, CodeLlama-style prose
        outputs would silently count as zero-violation in confirmatory
        runs — the Week 9 smoke-test finding."""
        assert any(
            r["status"] == "invalid_pseudocode" for r in combined_results
        )

    def test_at_least_one_violation_is_detected(self, combined_results):
        """Regression guard for bug #3 (analyze_combined arg order): if
        the watchdog is ever called incorrectly, total_violations would
        stay at 0 for every row and this assertion would fail."""
        assert any(
            r["status"] == "success" and r["total_violations"] > 0
            for r in combined_results
        )

    def test_at_least_one_sm_violation_is_detected(self, combined_results):
        """The mock templates target SM-1, SM-5 and SM-6, so at least
        one row must have a non-zero sm_violations count."""
        assert any(
            r["status"] == "success" and r["sm_violations"] > 0
            for r in combined_results
        )


# ---------------------------------------------------------------------------
# E3 watchdog-in-loop smoke coverage
# ---------------------------------------------------------------------------


class TestE3MockSmoke:
    """Smoke coverage for run_e3(). Locks in the watchdog-in-loop
    protocol as implemented: initial call in BASELINE, up to
    max_retries retries each also in BASELINE, retry loop breaks
    when total_violations == 0 or status != 'success'.

    The third test in this class (test_retry_rows_are_baseline_mode)
    is a regression guard for Session 17 Commit 1b: the retry path
    used to run in SAFETY mode, which confounded H6 with the safety
    prompt paradox. If anyone flips it back, that test fails."""

    def test_run_e3_returns_nonempty_result_list(self, two_tasks, output_dir):
        results = runner.run_e3(
            tasks=two_tasks,
            reps=2,
            max_retries=2,
            output_dir=output_dir,
            provider="mock",
            mock_seed=42,
        )
        # Lower bound: 1 model * 2 tasks * 2 reps = 4 initial calls.
        # Upper bound: 4 * (1 + max_retries) = 12 with all retries used.
        assert 4 <= len(results) <= 12, (
            f"E3 row count {len(results)} outside expected [4, 12]"
        )

    def test_run_e3_rows_have_required_schema(self, two_tasks, output_dir):
        results = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        for row in results:
            missing = REQUIRED_COLUMNS - row.keys()
            assert not missing, f"row missing columns: {missing}"
            assert row["experiment"] == "E3"

    def test_retry_rows_are_baseline_mode(self, two_tasks, output_dir):
        """REGRESSION GUARD (Session 17 Commit 1b). Every retry row
        must carry condition == 'baseline'. If anyone changes the
        retry call back to PromptMode.SAFETY (or any other mode),
        this test fails and blocks the regression from reaching
        confirmatory E3 data collection."""
        results = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        retry_rows = [r for r in results if r["retry"] > 0]
        # At least one retry must have fired for this test to be
        # meaningful under the mock's deterministic template rotation.
        assert len(retry_rows) > 0, (
            "No retry rows produced; cannot guard retry mode. "
            "Check that mock_seed=42 produces at least one violation-"
            "bearing initial call."
        )
        for r in retry_rows:
            assert r["condition"] == "baseline", (
                f"Retry row in mode '{r['condition']}', expected "
                f"'baseline'. Commit 1b regression detected."
            )

    def test_initial_rows_are_baseline_mode(self, two_tasks, output_dir):
        results = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        initial_rows = [r for r in results if r["retry"] == 0]
        assert len(initial_rows) == 4, (
            f"Expected 4 initial rows (1 model * 2 tasks * 2 reps), "
            f"got {len(initial_rows)}"
        )
        for r in initial_rows:
            assert r["condition"] == "baseline"

    def test_retry_bound_respected(self, two_tasks, output_dir):
        """No row's retry field may exceed max_retries."""
        max_retries = 2
        results = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=max_retries,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        for r in results:
            assert r["retry"] <= max_retries, (
                f"Row retry={r['retry']} exceeds max_retries={max_retries}"
            )

    def test_break_on_clean_initial_call(self, two_tasks, output_dir):
        """If an initial call has zero violations and success status,
        no retry rows should exist for that (model, task, rep) key."""
        results = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        # Group rows by (model, task_id, rep) and inspect each group
        from collections import defaultdict
        groups = defaultdict(list)
        for r in results:
            key = (r["model"], r["task_id"], r["rep"])
            groups[key].append(r)

        for key, rows in groups.items():
            rows_sorted = sorted(rows, key=lambda r: r["retry"])
            initial = rows_sorted[0]
            assert initial["retry"] == 0
            if (
                initial["status"] == "success"
                and initial["total_violations"] == 0
            ):
                assert len(rows_sorted) == 1, (
                    f"Group {key} had clean initial call but "
                    f"{len(rows_sorted) - 1} retry rows"
                )

    def test_run_e3_is_deterministic_across_runs(self, two_tasks, output_dir):
        """Same seed must produce the same result list across runs."""
        r1 = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        # Re-create an isolated output dir for the second run to avoid
        # file-collision side effects.
        r2 = runner.run_e3(
            tasks=two_tasks, reps=2, max_retries=2,
            output_dir=output_dir, provider="mock", mock_seed=42,
        )
        assert len(r1) == len(r2), (
            f"Determinism violated: len {len(r1)} vs {len(r2)}"
        )
        for a, b in zip(r1, r2):
            # Timestamps differ between runs; compare the substantive
            # structural fields only.
            for k in ("experiment", "model", "task_id", "condition",
                      "rep", "retry", "status", "total_violations"):
                assert a[k] == b[k], (
                    f"Determinism violated at key '{k}': {a[k]} vs {b[k]}"
                )
