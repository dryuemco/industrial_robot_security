# OSF Pre-registration — Amendment 3 (DRAFT)

**Project DOI:** 10.17605/OSF.IO/VE5M2
**Amendment 1:** Approved 2026-04-07.
**Amendment 2:** Pending host-supervisor acknowledgment. **Amendment 3 must NOT be filed until Amendment 2 is acknowledged.**
**Amendment 3:** This document.

## Summary of Changes

Amendment 3 introduces two analysis-plan changes and one operational disclosure:

- **H7 (confirmatory methodological update):** Locks the execution-side simulator selection to URSim e-Series 5.12.8 + ROS2 Humble + `ur_robot_driver` 2.12.0, replacing the Gazebo placeholder named in the v1 prereg. This is a methodological clarification, not a hypothesis change.
- **H8 (exploratory):** Adds an exploratory subgroup analysis examining whether per-task Task Complexity Score (TCS) is associated with per-task violation outcomes from E1, E2, and E3.
- **Operational Disclosures (non-hypothesis):** Clarifies that the Ollama host IP recorded in Amendment 1 is operational metadata rather than a protocol parameter, and discloses the model-digest drift between the Session-12 exploratory pilot and the confirmatory runs.

**No change affects H1–H6.** The primary McNemar and Cochran's Q analyses, the watchdog-in-loop H6 test, and all confirmatory conclusions registered in Amendment 1 remain unchanged.

---

## H7. Simulator Selection (Confirmatory Methodological Update)

**Statement.** All execution-based replication and downstream validation studies for ENFIELD-generated URScript will use the URSim e-Series Docker image (version 5.12.8) together with ROS2 Humble and `ur_robot_driver` 2.12.0, in place of the Gazebo + Ignition stack that was named provisionally in the v1 pre-registration.

**Rationale.** URSim provides Universal Robots' vendor-supplied PolyScope simulator, which exposes the same External Control URCap API used in real UR5e deployment. This gives stronger sim-to-real fidelity than Gazebo for the URScript code generated in our experiments. The smoke test confirming a `RUNNING/NORMAL` URSim state with live `/joint_states` and `/tcp_pose_broadcaster/pose` topics is documented in §V.G of the manuscript (Execution Setup) and in `docs/SIMULATOR_DECISION.md`.

**Impact on H1–H6.** None. H1–H6 are static-analysis hypotheses evaluated on generated URScript text and on watchdog verdicts; they do not depend on the simulator. The simulator selection only affects future execution-based studies.

**Disclosure.** The verbal go-ahead for this change was obtained from the host supervisor; this amendment formalizes the decision. The supervisor has not been shown H8-specific outcomes prior to acknowledgment of Amendment 2.

---

## H8. Complexity-Stratified Subgroup Analysis (Exploratory)

### H8 Statement (Exploratory; Non-directional)

For each `(experiment, model, condition)` stratum in E1, E2, and E3, we will report the Spearman rank correlation ρ between per-task Task Complexity Score (TCS_total) and per-task outcome, using three outcome metrics: CVR (binary), violation count (continuous), and `severity_max` (continuous). We will additionally report tertile-stratified mean outcomes with percentile bootstrap 95% CIs.

**This is explicitly an exploratory analysis. No directional hypothesis is preregistered. No multiple-comparisons correction will be applied.**

### Why We Are Filing This Now

The host supervisor raised the question of whether per-task complexity modulates the violation patterns reported under H4–H6 in our primary analysis. The question is reviewer-anticipated and would otherwise have been answered post-hoc and unblinded. By filing it here as exploratory, we commit to reporting the analysis irrespective of outcome and disclose all design decisions (metric choice, aggregation policy, weight specification) prior to inspecting the H8-specific results.

We confirm that, at the time of filing, only the primary H4/H5/H6 results have been inspected. The TCS feature extractor, weight specification, and analysis pipeline are committed in the repository (commits `b979881`, `e5bc8e2`, `57f620b`, `3a0756d`) and pass an AST-level outcome-blindness test (`tests/test_task_complexity.py::test_no_outcome_imports`). The weights-config SHA-256 hash and analysis-script SHA-256 hash are recorded in the output manifest of every run.

### TCS Construction (Full Specification)

The formal definition is given in `docs/TASK_COMPLEXITY_SPEC.md`. Briefly:

> TCS_total(t) = 0.5 · z(TCS_struct(t)) + 0.5 · z(TCS_safety(t))

where `TCS_struct` sums five structural counts (motion commands, command types, motion frames, environment frames, safety zones) from the baseline Task IR, and `TCS_safety` sums ten safety/attack-surface descriptors mapped to A1–A8 (mode strictness with 3× weight, human-proximity flag with 2× weight, applicable ISO clause count, halfspace count, orientation cone count, required safety-node count, tool-activation-constraint presence, work-object-specification presence, E-Stop required flag with 2× weight, blocked-pattern count). z-normalization uses sample (n−1) standard deviation across the 15-task suite.

Tertiles `T_low`, `T_med`, `T_high` are assigned from the empirical TCS_total distribution; in the confirmatory run the cut points are `lo = −0.618` and `hi = +0.346`, producing a balanced 5/5/5 split.

### Analysis Policies (Locked)

**Outcome aggregation.**

1. For E3 only: collapse retries by keeping the row with `MAX(retry)` per `(model, task_id, rep, condition, adversarial_type)`. This represents the watchdog-in-loop **final** state.
2. Across reps: take the MEAN of the chosen outcome metric within each `(experiment, model, task_id, condition, adversarial_type)` cell. With 3 reps, CVR yields values in `{0, 1/3, 2/3, 1}`.
3. CVR = 1 iff `total_violations > 0`; count = `total_violations`; severity = `severity_max` as recorded in the experiment CSV.

**Inference.**

1. Spearman ρ per `(experiment, model, condition, adversarial_type, outcome_metric)` stratum, with `n` = number of tasks with non-missing data in that stratum.
2. Strata where the outcome vector is constant within the stratum will be reported with `y_is_constant = 1` and ρ undefined (a documented edge case, not a missing value). This is the *expected* behavior for CVR under a ceiling effect.
3. Tertile-stratified mean outcome with percentile bootstrap 95% CI: 10000 resamples, seed = 42 (deterministic).
4. **No multiple-comparisons correction.**

**Reporting commitment.**

All ρ, p, and n values are reported in the per-stratum CSV regardless of magnitude or significance. Significant findings will be flagged as "consistent with" or "inconsistent with" plausible interpretations; we will not claim confirmation. The manuscript Section VII.C will report all three findings (ceiling effect, negative density correlation, adversarial flattening) and acknowledge that the negative direction in Finding C.2 was not anticipated.

### Pre-filing Inspection Commitment

Once filed, no further modifications will be made to: (a) the TCS feature set, (b) the weights config, (c) the aggregation policies, or (d) the seed value. Any deviation will be reported as a separate amendment.

### Reproducibility Artifacts

Scripts (commit hashes to be inserted at filing time):

- `scripts/compute_task_complexity.py` (commit `e5bc8e2`)
- `scripts/complexity_correlation_analysis.py` (commit `3a0756d`)
- `configs/tcs_weights.yaml` (commit `e5bc8e2`)
- `tests/test_task_complexity.py` (commit `e5bc8e2`)
- `tests/test_complexity_correlation.py` (commit `3a0756d`)

Outputs to be deposited at Month-6 OSF release:

- `results/task_complexity_scores.csv` (per-task TCS)
- `results/task_complexity_manifest.json` (TCS run manifest with SHA-256 hashes)
- `results/complexity_correlation/per_scenario_joined.csv` (long-form joined data)
- `results/complexity_correlation/spearman_summary.csv` (one row per stratum per metric)
- `results/complexity_correlation/tertile_cvr_summary.csv` (tertile-stratified means + CIs)
- `results/complexity_correlation/manifest.json` (analysis run manifest)

---

## Operational Disclosures (Non-Hypothesis)

This section documents two operational drifts that have occurred since the v1 pre-registration and Amendment 1, neither of which affects the H1–H8 protocol or the confirmatory analysis. They are filed here for transparency and to clarify the historical record of operational metadata.

### Ollama Host IP — Clarification

The Amendment 1 table records the Ollama host IP as `192.168.1.5:11434`, replacing the v2.1 value of `192.168.1.4:11434`. The actual IP allocated to PC2 has subsequently drifted back to `192.168.1.4` following a routine DHCP refresh (observed 2026-05-05), inverting the direction of change recorded in Amendment 1.

We clarify here that the Ollama host IP is **operational metadata**, not a protocol parameter. All inference calls use the `OLLAMA_HOST` environment variable and are insensitive to the specific IP assigned to PC2 at any given moment. The values listed in the Amendment 1 table reflect the IP active on the dates of those amendments and should be read as a historical log, not as constraints fixed by the protocol. No retroactive edit of Amendment 1 will be performed; the inverted-direction record is preserved as part of the project's transparent change history.

A replicator using a different Ollama host (different IP, different machine, or even a local-machine Ollama instance) is operating within protocol. The run manifest for every confirmatory run records the IP in effect at the time of that run, but only as a debugging-aid field.

### Model Digest Drift — Disclosure

Ollama tracks pulled models by an internal manifest digest. The digest active during the Session-12 exploratory E1 pilot (2026-04-15) for `qwen2.5-coder:32b` Q4_K_M is no longer recoverable from the local Ollama store; the current model digest, `b92d6a0b...` (last modified 2026-05-05), is what has been used for all subsequent runs including the full confirmatory E1, E2, and E3 and the Lane-3 URSim execution pilot reported in paper §VI.L.

**Impact on the confirmatory analysis.** None. The full E1, E2, and E3 confirmatory runs were generated under the current pinned digest. The Session-12 artefacts in `results/e1_pilot_session12/` are an exploratory pilot only; they were never part of the H4–H6 confirmatory family or of any registered analysis.

**Impact on the Lane-3 URSim pilot.** The pilot (§VI.L) was generated freshly under the current pinned digest rather than reusing Session-12 baselines. The decision is recorded in `docs/TODO.md` (S25/S26 Faz B, "decision: B-fresh"). Bit-reproduction of Session-12 outputs is therefore not possible; bit-reproduction of the confirmatory outputs and of the Lane-3 pilot outputs is possible against the current digest.

**Replication commitment.** The current digest `b92d6a0b...` will be deposited in `docs/replication/MODEL_DIGESTS.txt` at the Month-6 OSF release, together with the corresponding digests for `deepseek-coder-v2:16b` and `codellama:34b`. The replication package will instruct replicators to `ollama pull` the named tags and verify the locally pulled digests against the recorded values; should a future Ollama tag-to-digest mapping drift, the recorded digests in `MODEL_DIGESTS.txt` are the authoritative reference, not the tag string.

**Future drift policy.** Any further model-digest drift observed before the Month-6 release will be disclosed in a subsequent OSF amendment rather than retroactively edited into Amendment 3 or earlier records.

---

## Notes for Supervisor Review

This document is a draft. Before filing on OSF:

1. Amendment 2 must be acknowledged by the host supervisor (current blocker).
2. The commit hashes for the scripts cited above will be re-verified with `git rev-parse HEAD` at filing time and inserted as the canonical references.
3. The "host supervisor verbal approval" for the URSim selection (H7) will be linked to the corresponding meeting note in the project log.
4. The "host supervisor raised the question" sentence under H8 will be reviewed by the supervisor to ensure accurate attribution.
