# Task Complexity Score (TCS) Specification

**Status:** Frozen as of commit `3a0756d` (PR-B.1).
**Scope:** Exploratory analysis instrument for OSF Amendment 3, H8.
**Outcome-blind invariant:** Computed from baseline Task IR only. Enforced by AST-level test `tests/test_task_complexity.py::test_no_outcome_imports`.

## 1. Definition

For each baseline task `t` in the 15-task suite:

```
TCS_struct(t) =  w_motion · motion_command_count(t)
               + w_types  · distinct_command_types(t)
               + w_frames · distinct_motion_frames(t)
               + w_env    · environment_frame_count(t)
               + w_zones  · safety_zone_count(t)

TCS_safety(t) =  w_mode   · mode_strictness(t)
               + w_humans · human_proximity_flag(t)
               + w_iso    · applicable_iso_clause_count(t)
               + w_halfs  · halfspace_count(t)              (A2 surface)
               + w_cones  · orientation_cone_count(t)       (A3 surface)
               + w_nodes  · required_safety_node_count(t)   (A5 surface)
               + w_tool   · tool_activation_present(t)      (A7 surface)
               + w_wos    · work_object_spec_present(t)     (A6 surface)
               + w_estop  · estop_required_flag(t)          (A5 binary)
               + w_patts  · blocked_pattern_count(t)        (A8 surface)

TCS_total(t)  =  0.5 · z(TCS_struct(t)) + 0.5 · z(TCS_safety(t))
```

where `z(·)` is z-score normalization across the 15-task suite using sample (n−1) standard deviation.

Tertile cut points are derived from the empirical distribution of `{TCS_total(t)}`:

```
lo_cut = TCS_total sorted at index floor(n/3) - 1   = -0.618 (confirmatory run, n=15)
hi_cut = TCS_total sorted at index floor(2n/3)     = +0.346 (confirmatory run, n=15)
T_low  = { t : TCS_total(t) <= lo_cut }
T_high = { t : TCS_total(t) >= hi_cut }
T_med  = remainder
```

## 2. Frozen Weight Specification

Stored in `configs/tcs_weights.yaml` v2.0. Reproduced here for reviewer convenience:

| Group       | Feature                          | Weight |
|-------------|----------------------------------|--------|
| Structural  | motion_command_count             | 1.0    |
| Structural  | distinct_command_types           | 1.0    |
| Structural  | distinct_motion_frames           | 1.0    |
| Structural  | environment_frame_count          | 1.0    |
| Structural  | safety_zone_count                | 1.0    |
| Safety      | mode_strictness                  | **3.0** |
| Safety      | human_proximity_flag             | **2.0** |
| Safety      | estop_required_flag              | **2.0** |
| Safety      | iso_clause_count                 | 1.0    |
| Safety      | halfspace_count                  | 1.0    |
| Safety      | orientation_cone_count           | 1.0    |
| Safety      | required_safety_node_count       | 1.0    |
| Safety      | tool_activation_present          | 1.0    |
| Safety      | work_object_spec_present         | 1.0    |
| Safety      | blocked_pattern_count            | 1.0    |
| Blend       | struct                           | 0.5    |
| Blend       | safety                           | 0.5    |

The SHA-256 hash of `configs/tcs_weights.yaml` is recorded in every output manifest written by `scripts/compute_task_complexity.py`.

## 3. Weight Selection Rationale

Weights were chosen **before any outcome data was inspected** for complexity-stratified results. Three principles guided the choice:

1. **Uniformity by default.** Each count contributes one unit per item, so a task with more waypoints, frames, zones, halfspaces, cones, required nodes, or blocked patterns scores proportionally higher.
2. **Mode and human-proximity get scalar boosts.** ISO 10218:2025 explicitly elevates collaborative-mode requirements; we encode this by giving `mode_strictness` 3× weight and `human_proximity_flag` 2×.
3. **E-Stop is binary but high-risk.** `estop_required_flag` gets 2× rather than 1× because absence-of-E-Stop maps directly to A5 (Safety Logic Bypass) violations of ISO 10218:2025 Clause 5.4.2.

## 4. Sensitivity Analysis Plan

To verify that findings are not artifacts of the 0.5/0.5 blend choice, `scripts/complexity_correlation_analysis.py` will be re-run with two alternate blends:

- 0.7 / 0.3 (safety-heavy): verifies that headline negative-correlation findings persist when safety surface drives TCS.
- 0.3 / 0.7 (structural-heavy): verifies the same when structural complexity dominates.

Findings in Section VII.C of the paper should qualitatively persist across these blends. Sensitivity results will be reported in `results/complexity_correlation/sensitivity_blends/` as additional manifest entries when re-run.

## 5. Tertile Distribution (Confirmatory Run)

From `results/task_complexity_manifest.json`:

```
tertile_lo_cut: -0.618379
tertile_hi_cut: +0.346165
tertile_distribution: { T_low: 5, T_med: 5, T_high: 5 }
```

Tertile assignment table:

| Tertile | Tasks                              |
|---------|------------------------------------|
| T_high  | T003, T004, T007, T010, T012       |
| T_med   | T001, T002, T011, T013, T014       |
| T_low   | T005, T006, T008, T009, T015       |

## 6. Reproducibility

To reproduce:

```bash
python scripts/compute_task_complexity.py \
  --tasks-dir enfield_tasks/ir/tasks/ \
  --weights-config configs/tcs_weights.yaml \
  --output results/task_complexity_scores.csv \
  --manifest results/task_complexity_manifest.json

python scripts/complexity_correlation_analysis.py \
  --tcs-csv results/task_complexity_scores.csv \
  --e1-csv results/e1_confirmatory_session14/e1_results.csv \
  --e2-csv results/e2_confirmatory/e2_results.csv \
  --e3-csv results/e3_confirmatory/e3_results.csv \
  --output-dir results/complexity_correlation/ \
  --outcome-metrics cvr count severity \
  --bootstrap 10000 --seed 42
```

Integrity check (the SHA-256 values below should match the committed manifest):

```bash
sha256sum configs/tcs_weights.yaml
sha256sum scripts/compute_task_complexity.py
sha256sum scripts/complexity_correlation_analysis.py
```

## 7. Limitations of TCS as a Complexity Construct

`TCS_safety` is monotonic in the number of declared safety constraints, which conflates **specification richness** (clearer scaffold for the model) with **constraint difficulty** (more rules to obey simultaneously). The negative correlation observed in Section VII.C (Finding C.2) is consistent with the former interpretation. A future ablation should separate the two by holding the IR fixed and varying only prompt-side specification density.

`TCS_struct` weights all command types equally, treating a `move_linear` and a `set_tool` as identical contributions. A more sophisticated extension would weight commands by their kinematic complexity (e.g., circular vs. linear motion) or by the number of safety constraints they activate. We did not pursue this in the current version because (a) it would require additional weight choices that could not be pre-specified before outcome inspection, and (b) the current uniform weighting already produces an attack-surface-aligned TCS_safety that proved informative under the count metric.
