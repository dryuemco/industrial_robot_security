# OSF Pre-registration: Formal Adversarial Testing of LLM-Generated Code for Industrial Robots

**Registration Template:** OSF Standard Pre-Data Collection Registration
**Version:** 3.0 (submission-ready)
**Date:** 2026-03-16
**Status:** Ready for OSF submission

**Principal Investigator:** Yunus Emre Cogurcu, PhD (Cukurova University)
**Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)
**Funding:** Horizon Europe Grant Agreement No 101120657 (ENFIELD Exchange Scheme)
**Repository:** https://github.com/dryuemco/industrial_robot_security

---

## 1. Study Information

### 1.1 Title

Formal Adversarial Testing of LLM-Generated Code for Industrial Robots: An AST-Based Static Watchdog for ISO 10218:2025 Safety Violation Detection

### 1.2 Research Questions

**RQ1 (Primary):** Can a rule-based static watchdog, operating on a vendor-neutral Task Intermediate Representation (IR), significantly reduce safety violations in adversarially manipulated industrial robot task specifications, as measured against ISO 10218:2025 safety invariants?

**RQ2 (Secondary):** Which attack types (A1-A8) are most effectively detected by IR-level static analysis, and which require runtime or vendor-specific detection mechanisms?

**RQ3 (Exploratory):** What is the relationship between attack layer (parametric vs structural vs meta) and detection difficulty for static analysis?

### 1.3 Hypotheses

**H1 (Primary):** The static watchdog achieves a relative reduction of at least 40% in safety violation (SV) count compared to the unprotected baseline (no watchdog), measured across matched task-attack pairs using McNemar's test (alpha = 0.05).

**H0 (Null):** There is no significant difference in safety violation rates between scenarios analyzed with the watchdog enabled versus the baseline (no analysis).

**H2 (Secondary):** The watchdog achieves at least 80% detection rate across all 8 attack types (A1-A8) when evaluated on the full stratified test suite.

**H3 (Secondary):** The false positive rate on safe baseline tasks is at most 5% (i.e., at least 95% of clean baselines pass without violations).

### 1.4 Pilot Data Disclosure

A pilot study was conducted on the initial 5 baseline tasks (T001-T005) and 40 adversarial variants (5 tasks x 8 attack types) to validate the experimental infrastructure and refine detection thresholds. Pilot results are disclosed here in full:

- Baselines: 5/5 safe (FP rate: 0.0%)
- Variants: 36/40 flagged (overall detection rate: 90.0%)
- Per-attack: A1=100%, A2=100%, A3=80%, A4=100%, A5=100%, A6=60%, A7=80%, A8=100%

These results informed threshold calibration for DM-5 (A6 frame detection) and DM-6 (A7 tool validation). Following the pilot, the task suite was expanded to 15 baseline tasks (T001-T015) and detection rules were refined. The confirmatory analysis will use the full 15-task suite (120 matched pairs). Pilot data (T001-T005) will be reported separately as calibration data.

---

## 2. Design Plan

### 2.1 Study Type

Controlled experiment with simulation-based evaluation. All experiments run in software (ROS2 Humble + Gazebo simulation). No physical robot deployment. All robot code samples are synthetic (hand-crafted baseline tasks with programmatically generated adversarial variants).

### 2.2 Study Design

Within-subject (matched pairs). Each task-attack combination is tested both with and without the static watchdog. The watchdog is deterministic (no stochastic components), so each scenario produces exactly one outcome per condition.

**Independent Variables:**
- Attack Type: A1-A8 (8 levels)
- Watchdog State: ON / OFF (2 levels)
- Task Category: pick_place, welding, palletizing, inspection, custom (5 levels for stratification)
- Operating Mode: collaborative, fenced, hybrid (3 levels)

**Dependent Variables:**
- Primary: Safety Violation count (SV) per scenario --- binary (safe/unsafe) for McNemar
- Secondary: Attack-specific detection rate (proportion detected per attack type)
- Secondary: False positive rate on baselines

### 2.3 Randomization

Not applicable. The study uses a complete factorial design: every task is paired with every attack type. Task generation follows a stratified sampling strategy to ensure coverage across 5 categories (pick_place, welding, palletizing, inspection, custom) and 3 operating modes (collaborative, fenced, hybrid). Adversarial variants are generated deterministically (seed=42).

---

## 3. Sampling Plan

### 3.1 Data Collection Procedures

Tasks are defined in a vendor-neutral JSON Intermediate Representation (Task IR) validated against a formal JSON Schema (Draft 2020-12). Each task specifies motion sequences, safety constraints (speed limits, safeguarded space, forbidden orientation cones, tool payload bounds), and required safety logic patterns.

Adversarial variants are generated programmatically by applying one of eight mutation operators (A1-A8) to each baseline task. Each mutation targets a specific ISO 10218:2025 safety invariant.

### 3.2 Sample Size

**Achieved:** 15 baseline tasks spanning 5 categories and 3 operating modes. With 8 attack types, this yields 120 matched pairs.

**Power Analysis:** McNemar's test for paired dichotomous data requires n >= 25 discordant pairs for 80% power at alpha = 0.05 with a minimum detectable effect size of 40% relative SV reduction. With 120 total pairs and an expected discordant pair rate of at least 50%, we expect approximately 60 discordant pairs, providing adequate statistical power.

### 3.3 Sample Size Rationale

The 15-task suite provides a complete 5x3 stratified design (5 categories x 3 operating modes = 15 cells, 14 filled). This exceeds the minimum 25 discordant pairs required for McNemar's test and ensures representation across all major industrial robot application domains. The pilot study (5 tasks) demonstrated infrastructure viability; the full suite triples the task count and adds two new categories (custom, expanded inspection).

### 3.4 Stopping Rule

Data collection is complete when all 120 planned task-attack combinations (15 tasks x 8 attacks) have been analyzed. No sequential testing or early stopping is planned.

---

## 4. Variables

### 4.1 Measured Variables

**Primary Outcome (per scenario):**
- safe: boolean --- did the watchdog flag zero violations?
- violation_count: integer --- total number of violations detected

**Secondary Outcomes (per scenario):**
- a1_detected through a8_detected: boolean flags for each attack type
- max_severity: float --- highest severity score across all violations
- detected_attacks: comma-separated list of detected attack IDs

**Aggregate Metrics:**
- Overall detection rate: proportion of adversarial variants flagged
- False positive rate: proportion of safe baselines incorrectly flagged
- Per-attack detection rate: detection proportion per attack type
- Detection matrix: 8 x 15 binary matrix (attack x task)

### 4.2 Indices

No composite indices are planned. Each attack type is analyzed independently.

### 4.3 Attack Taxonomy (A1-A8)

| ID | Attack | Layer | ISO Clause | Detection Mechanism |
|----|--------|-------|-----------|-------------------|
| A1 | Speed Injection | Parametric | 5.6 | DM-1: Value range check |
| A2 | Zone Penetration | Parametric | 5.12.3 | DM-2: Halfspace boundary test |
| A3 | Orientation Anomaly | Parametric | 5.3 | DM-3: Orientation cone test |
| A4 | Payload Misconfiguration | Parametric | 5.3/5.4 | DM-1: Bounds check |
| A5 | E-Stop / Logic Bypass | Structural | 5.4/5.5 | DM-4: AST pattern + presence |
| A6 | Frame Confusion | Structural | 5.12.3/5.3 | DM-5: Frame registry + sanity |
| A7 | Tool Misuse | Structural | 5.1.14/5.1.15 | DM-6: Tool state machine |
| A8 | Prompt Injection | Meta | 5.3 + EU AI Act | DM-7: Prompt security check |

Formal definitions in: attack_definitions_A1_A4.md and attack_definitions_A5_A8.md (repository docs/).

---

## 5. Analysis Plan

### 5.1 Statistical Models

**Primary Test:** McNemar's test for paired nominal data. Each matched pair consists of the same task-attack scenario evaluated with and without the watchdog. The outcome is binary: safe (no violations) or unsafe (one or more violations). Without the watchdog, all adversarial variants are classified as unsafe (the attack succeeds by design). With the watchdog, some are caught (classified as unsafe by the watchdog = attack detected).

The McNemar test evaluates whether the watchdog significantly reduces the proportion of undetected attacks (scenarios that remain classified as "safe" despite containing adversarial modifications).

**Multiple Comparisons:** Holm-Bonferroni correction for family-wise error rate (alpha = 0.05) when testing detection rates across the 8 attack-type subgroups.

**Confidence Intervals:** 95% exact Clopper-Pearson confidence intervals for all proportions (detection rates, false positive rates).

### 5.2 Transformations

None. All outcomes are binary or count data.

### 5.3 Inference Criteria

- Significance threshold: alpha = 0.05 (two-tailed for McNemar)
- Family-wise error rate: alpha = 0.05 (Holm-Bonferroni across 8 attack subgroups)
- Minimum effect size for H1: 40% relative reduction in undetected attacks
- H2 criterion: per-attack detection rate >= 80% for each A1-A8
- H3 criterion: false positive rate <= 5%

### 5.4 Data Exclusion

No data exclusion criteria. All generated scenarios are included in analysis. The pilot dataset (5 tasks, 40 variants) is reported separately as calibration data and excluded from the confirmatory analysis.

### 5.5 Exploratory Analyses

- Subgroup analysis by task category (pick_place, welding, palletizing, inspection, custom)
- Subgroup analysis by operating mode (collaborative, fenced, hybrid)
- Subgroup analysis by attack layer (parametric A1-A4 vs structural A5-A7 vs meta A8)
- Correlation between attack severity metric and detection success
- Analysis of multi-attack detection in A8 variants (which induced A1-A7 are caught)

---

## 6. Other

### 6.1 Existing Data

At the time of pre-registration, the following infrastructure and data exist:

**Infrastructure:**
- Task IR schema (JSON Schema Draft 2020-12)
- Attack variant generator producing A1-A8 mutations (deterministic, seed=42)
- Static watchdog with 8 detection rules (DM-1 through DM-7)
- IR to URScript translator for UR5e
- Experiment runner producing CSV/JSON reports
- CI/CD pipeline with 475 passing tests across 8 jobs

**Datasets:**
- 15 baseline tasks (T001-T015) spanning 5 categories x 3 operating modes (14/15 cells filled)
- 120 adversarial variants (15 tasks x 8 attack types)
- Pilot analysis on initial 5 tasks (T001-T005): 90% detection, 0% FP
- Full-suite analysis on 15 tasks (T001-T015): 95% detection, 0% FP

The confirmatory statistical analysis (McNemar test, Holm-Bonferroni correction, confidence intervals) has NOT yet been performed. The 95% figure above is the raw detection count (114/120 flagged) without formal hypothesis testing.

### 6.2 Tools and Software

- ROS2 Humble on Ubuntu 22.04
- Python 3.10+, jsonschema >= 4.20, pytest
- Docker multi-stage build with CycloneDX SBOM
- GitHub Actions CI/CD (8 jobs)
- Statistical analysis: scipy.stats (McNemar), statsmodels (Holm-Bonferroni)

### 6.3 Scope and Limitations

- All experiments are simulation-only. No physical robot deployment.
- ISO 10218:2025 compliance assessment is advisory, not formal certification.
- The static watchdog operates on the Task IR level. Vendor-specific code analysis (URScript, RAPID, KRL) is planned but not included in this registration.
- Results may not generalize to production robot systems without sim-to-real validation.
- LLM-generated code is simulated through programmatic attack mutations, not through actual LLM prompting (LLM integration is future work scope).

### 6.4 Open Science Commitment

- Repository: Apache-2.0 license, public GitHub release (v1.0) at Month 6
- Data: All experiment outputs (CSV, JSON) deposited on OSF
- SBOM: CycloneDX-format software bill of materials generated in CI
- Replication: Docker container with single-command experiment reproduction
- Responsible disclosure: 3-tier access model (public / verified researcher / vendor)

### 6.5 Ethical Considerations

This project involves only simulation environments and synthetic data. No human subjects, no physical robots, no real-world deployment. Adversarial content is generated exclusively for testing the watchdog's detection capabilities. The responsible disclosure framework ensures security-sensitive attack specifications are shared only with verified researchers and affected vendors.

---

## Appendix A: Detection Rule Specifications

| Rule | Input | Reference | Algorithm | Output |
|------|-------|-----------|-----------|--------|
| DM-1 | speed_mm_s per command | max_tcp_speed_mm_s | Direct comparison | Severity ratio |
| DM-2 | waypoint positions | halfspace constraints | n*p > d test | Penetration depth (mm) |
| DM-3 | quaternion orientation | forbidden cones | quat -> tool_z -> cone test | Angular penetration (deg) |
| DM-4 | estop_check commands, required_nodes | safety_logic_requirements | Presence + emptiness check | Missing node count |
| DM-5 | wos uframe_translation | safeguarded space | Component range check | Deviation (mm) |
| DM-6 | tool.type, allowed_modes | activation_constraints | Identity + mode match | Mismatch flag |
| DM-7 | prompt_security fields | threshold constants | Perplexity + pattern check | Bypass indicator |

## Appendix B: Pilot Detection Matrix (5-task calibration dataset)

| Attack | T001 | T002 | T003 | T004 | T005 | Rate |
|--------|------|------|------|------|------|------|
| A1 | 1 | 1 | 1 | 1 | 1 | 100% |
| A2 | 1 | 1 | 1 | 1 | 1 | 100% |
| A3 | 1 | 1 | 0 | 1 | 1 | 80% |
| A4 | 1 | 1 | 1 | 1 | 1 | 100% |
| A5 | 1 | 1 | 1 | 1 | 1 | 100% |
| A6 | 0 | 1 | 1 | 1 | 0 | 60% |
| A7 | 1 | 1 | 1 | 1 | 0 | 80% |
| A8 | 1 | 1 | 1 | 1 | 1 | 100% |

Note: A3/T003 miss = T003 has no forbidden orientation cones. A6 misses = small frame shifts within threshold. A7/T005 miss = camera tool has no activation_constraints. These are expected based on task design, not watchdog failures.

## Appendix C: Full-Suite Detection Summary (15-task confirmatory dataset)

| Attack | Detected | Total | Rate |
|--------|----------|-------|------|
| A1 | 15 | 15 | 100% |
| A2 | 15 | 15 | 100% |
| A3 | 14 | 15 | 93% |
| A4 | 15 | 15 | 100% |
| A5 | 15 | 15 | 100% |
| A6 | 12 | 15 | 80% |
| A7 | 13 | 15 | 87% |
| A8 | 15 | 15 | 100% |
| **Overall** | **114** | **120** | **95%** |

Baselines: 15/15 safe (FP rate: 0.0%)

Note: This is the raw detection count prior to formal confirmatory statistical analysis. McNemar test, Holm-Bonferroni correction, and confidence intervals will be computed as part of the registered analysis plan (Section 5).
