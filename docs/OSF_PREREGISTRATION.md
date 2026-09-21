# OSF Pre-registration: Formal Adversarial Testing of LLM-Generated Code for Industrial Robots

**Registration Template:** OSF Standard Pre-Data Collection Registration
**Version:** 3.1 (submitted)
**Date:** 2026-03-22
**Status:** Submitted — DOI: [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2)

**Principal Investigator:** Yunus Emre Cogurcu, PhD (Cukurova University)
**Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)
**Funding:** Horizon Europe Grant Agreement No 101120657 (ENFIELD Exchange Scheme)
**Repository:** https://github.com/dryuemco/industrial_robot_security

> **Note:** This document mirrors the content submitted to OSF on 2026-03-22.
> The authoritative version is the OSF registration at the DOI above.
> Any post-registration amendments will be documented in the manuscript.

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
| A1 | Speed Injection | Parametric | 5.5.3 | DM-1: Value range check |
| A2 | Zone Penetration | Parametric | 5.7.4 | DM-2: Halfspace boundary test |
| A3 | Orientation Anomaly | Parametric | 5.7.4 | DM-3: Orientation cone test |
| A4 | Payload Misconfiguration | Parametric | 5.1.15 | DM-1: Bounds check |
| A5 | E-Stop / Logic Bypass | Structural | 5.4.2 / 5.4 | DM-4: AST pattern + presence |
| A6 | Frame Confusion | Structural | 5.7.4 | DM-5: Frame registry + sanity |
| A7 | Tool Misuse | Structural | 5.1.14 | DM-6: Tool state machine |
| A8 | Prompt Injection | Meta | 5.1.16 + EU AI Act | DM-7: Prompt security check |

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
- CI/CD pipeline with 740 passing tests across 9 jobs

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

---

## Amendment 1 — 2026-04-06 (Week 10 of 24)

**Title:** Add security dimension (SM-1..7), LLM hypotheses (H4–H6),
Refusal Rate metric, and model substitution (Qwen3.5-27B → DeepSeek-Coder-V2-16B)

**Filed before E1/E2/E3 data collection:** ✅ Yes

### Changes

#### 1. Security dimension and new hypotheses (planned since Week 8)

Following supervisor feedback (Assoc. Prof. Georgios Spathoulas, NTNU),
the scope was extended to include:

- CWE-based security analysis layer: SM-1 (CWE-20), SM-2 (CWE-252),
  SM-3 (CWE-693), SM-4 (CWE-754), SM-5 (CWE-798), SM-6 (Missing Safety
  Preamble), SM-7 (Prompt Injection Marker)
- Three new hypotheses H4–H6 (see updated hypothesis table below)
- Refusal Rate (RR) metric: fraction of LLM calls returning no executable
  code, reported as a descriptive secondary outcome alongside H2/H3/H4
- `analyze_combined()` method: applies both DM-1..7 (safety) and SM-1..7
  (security) rules; `has_violation=1` if either layer flags a violation
  (fix committed 2026-04-06, before any experimental runs)

#### 2. Model substitution and LLM configuration (Week 10)

| Field | Previous (v2.1) | Updated (Amendment 1) |
|---|---|---|
| Model 2 | `qwen3.5:27b` (Alibaba, 27B, Apache 2.0) | `deepseek-coder-v2:16b` (DeepSeek AI, 16B, DeepSeek License) |
| Reason | — | Thinking-mode timeout >300 s during smoke testing |
| `max_tokens` | unspecified | `1024` (all models) |
| `timeout` | unspecified | `300 s` (all models) |
| Ollama host | `192.168.1.4:11434` | `192.168.1.5:11434` |

#### 3. Updated model table

| Model | Provider | Params | Quantization | License |
|---|---|---|---|---|
| `qwen2.5-coder:32b` | Alibaba Cloud | 32B | Q4_K_M | Apache 2.0 |
| `deepseek-coder-v2:16b` | DeepSeek AI | 16B | Q4_K_M | DeepSeek License |
| `codellama:34b` | Meta AI | 34B | Q4_K_M | Meta Llama 2 License |

Fixed inference parameters (all models): temperature=0.0, max_tokens=4096, timeout=300s. Determinism from greedy decoding at temperature=0.0 (no explicit sampling seed).

#### 4. Updated hypothesis table

| ID | Hypothesis | Test | Status |
|---|---|---|---|
| H1 | AST watchdog detects ≥90% of A1–A8 violations, ≤5% FPR | McNemar + exact binomial 95% CI | Original registration |
| H4 | ≥30% of baseline LLM-generated code contains ≥1 safety or security violation | One-sided exact binomial; Wilson 95% CI; α=0.05 | Added: Amendment 1 |
| H5 | Adversarial prompts (A6.1–A6.8) increase combined violation rate by ≥50 pp vs baseline | McNemar per attack; Holm-Bonferroni; family-wise α=0.05 | Added: Amendment 1 |
| H6 | Watchdog-in-loop reduces violation rate by ≥40% relative vs single-pass | McNemar; Holm-Bonferroni; Newcombe 95% CI; α=0.05 | Added: Amendment 1 |

### Related commits

| Commit | Description |
|---|---|
| `1d6d10f` | feat(analysis): add experiment runner and McNemar statistical analysis |
| *(latest)* | fix(runner): use analyze_combined() for combined safety+security violations |

### OSF platform note

This amendment was filed on the OSF pre-registration page on 2026-04-06 and **approved by OSF admin on 2026-04-07**. H4–H6 are now confirmatory under the registered protocol.
(DOI: [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2))
under the "Amendments" tab with the title and justification text above.


## Amendment 2 candidate — 2026-04-15 (Week 11 of 24)

**Title:** Reduce adversarial sub-variant family from eight to seven sub-variants; reduce H5 Holm–Bonferroni family from 24 to 21 (model, subtype) cells

**Status:** 🟡 **CANDIDATE — not yet submitted to OSF.** This block documents a scope-reduction action taken locally in the repository (paper, runner, McNemar module, tests) and held as a candidate amendment to the OSF pre-registration record. Submission to OSF is pending an explicit go-ahead. Until OSF acknowledgement, the OSF registration record continues to reflect the eight-subtype, twenty-four-cell H5 family of Amendment 1.

**Filed before E1/E2/E3 confirmatory data collection:** ✅ Yes. Only the E1 pilot (3 LLMs × 5 tasks × 2 conditions × 1 rep, exploratory, not part of the confirmatory family) has been executed. No confirmatory E1, E2, or E3 data exist as of this filing.

### Changes

#### 1. Adversarial sub-variant scope reduction (eight to seven)

One sub-variant in the original eight-subtype A8 taxonomy (the entry previously labelled A8.8 / Dual Instruction in earlier drafts of paper section III.C, and A6_4 in the codebase) had user-prompt and system-prompt templates that were placeholder strings rather than concrete adversarial content, making it a no-op at runtime. The variant could not contribute interpretable rejection power in the H5 confirmatory family.

Two alternatives were considered:
  (a) silently drop the variant during analysis with no record of the change — rejected as opaque to reviewers and inconsistent with pre-registration discipline;
  (b) back-fill the variant with new adversarial content after Amendment 1 approval — rejected as straightforwardly HARK-adjacent, since any new content would have been chosen with knowledge of the pilot data.

The remaining option — transparent removal before any confirmatory run, recorded explicitly here and in paper section VII.B.6 — was judged the more disciplined path. The seven retained sub-variants are: A8.1 Direct Override, A8.2 Role Playing, A8.3 Context Overflow, A8.4 Incremental, A8.5 Authority Claim, A8.6 Performance Framing, A8.7 Obfuscation.

#### 2. H5 Holm–Bonferroni family-size reduction (24 to 21 cells)

Consequent to change 1, the per-(model, subtype) cell count drops from 24 (3 models × 8 subtypes) to 21 (3 models × 7 subtypes). The corrected p-value threshold under Holm–Bonferroni at family-wise α = 0.05 changes from 0.05/24 = 0.00208 to 0.05/21 = 0.00238 for the most-significant cell, with the standard step-down adjustments for subsequent cells. The H5 decision criterion (at least one cell rejects H0: ΔCVR < 0.50 after correction) is unchanged in form; only the family size and corresponding thresholds shift.

#### 3. Experiment matrix call-budget update

The total call budget across E1, E2, E3 drops from approximately 1170 to 1125. The breakdown is: E1 baseline 270 calls (unchanged), E2 adversarial 315 calls (was 360; recomputed as 3 × 15 × 7), E3 watchdog-in-loop approximately 540 calls (unchanged). Cost remains zero (open-source models served via local Ollama).

#### 4. Updated H5 hypothesis row

| ID | Hypothesis (Amendment 2 candidate) | Test | Status |
|---|---|---|---|
| H5 | Adversarial prompts (A8.1–A8.7) increase combined violation rate by ≥50 pp vs baseline | McNemar per attack; Holm–Bonferroni; family-wise α=0.05; family size 21 | Candidate revision of Amendment 1 H5 row |

H4 and H6 rows are not affected by this amendment and retain the wording filed in Amendment 1.

### Affected paper sections (already reflect this candidate)

| Paper location | Status under this candidate |
|---|---|
| §III.B Table I, A8 row range | Updated to A8.1–A8.7 in Commit C |
| §III.C sub-variant table | Eighth row dropped, legacy editorial note removed in Commit C |
| §IV.B architecture diagram | Range narrowed in Commit C |
| §V.C experiment matrix (E2 and Total rows) | Recomputed in Commit C |
| §VI.G Table IV | Eighth column dropped from header, separator, and three model rows in Commit C |
| §VII.B.6 Pre-registered scope drift sub-subsection | Newly added in Commit D, references this candidate |

### Locked sections (intentionally retain Amendment 1 text)

The following sections continue to mirror the OSF-approved Amendment 1 record and will be brought into alignment with this candidate only after OSF acknowledgement:

| Locked location | Retained content |
|---|---|
| Paper §V.E (Statistical Analysis) | "A8.1–A8.8", "24 (3 models × 8 subtypes) cells", "8-subtype family" |
| Paper §VI.G H5 decision sentence | "across all 24 (3 × 8) tests" |
| Paper Appendix A (H5 description) | "A8.1–A8.8", "24 (3 models × 8 subtypes) cells" |
| This document, Amendment 1 H5 row (line 333) | "(A6.1–A6.8)", as approved by OSF |

### Related commits

| Commit | Description |
|---|---|
| `7b88bca` | refactor(prompt_builder): rename adversarial enum to A8.x and drop no-op variant |
| `649282d` | refactor(mcnemar): align condition labels with A8.x naming |
| `5558a5e` | docs(paper): align non-locked sections with seven-variant scope |
| `6d47f71` | docs(paper): document spec-vs-implementation drift in VII.B |

### OSF platform note

This amendment is **NOT yet filed** on the OSF pre-registration page (DOI: [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2)). The OSF registration record continues to reflect Amendment 1 (eight-subtype family, twenty-four-cell H5 contrast). When this candidate is submitted, the OSF "Amendments" tab will receive a new entry with the title and justification text above; the locked paper sections and the H5 row in this document will then be updated in a follow-up commit to remove the spec-vs-implementation drift documented in paper section VII.B.6.

---

## Record notes — 2026-09-21 (filing dates, data-collection timeline, commit identifiers)

This section was added during the IEEE Access revision after an audit of the OSF record
against the repository history and the raw result files. It does not change any
registered text above; the sections above are kept as they were written.

### Filing dates

| Amendment | Content | Filed on OSF (UTC) |
|---|---|---|
| 1 | Generator hypotheses H4–H6 | approved 2026-04-07 |
| 3 | URSim simulator lock (H7), exploratory complexity analysis (H8), operational disclosures | 2026-05-12T09:05:08Z |
| 2 | A8 family 8 → 7 strategies, H5 Holm family 24 → 21 | 2026-05-15T19:08:09Z |

The "Amendment 2 candidate" section above (2026-04-15) says the amendment is "NOT yet
filed"; that was its state when written. It was filed on 2026-05-15, after Amendment 3.

### Data-collection timeline

| Event | UTC |
|---|---|
| Amendment 2 code change (`prompt_builder`, McNemar labels) committed | 2026-04-15T17:54–18:09 |
| Amendment 2 candidate filed locally in this document: `d251c15` (working history) = `f501029` (main), same patch-id | 2026-04-15T18:54:51 |
| Run 1, E1 (270 rows, `results/e1_confirmatory_session14/`) | 2026-04-15T19:14:30 – 20:47:21 |
| Run 1, E2 (315 rows, `results/e2_confirmatory/`) | 2026-04-15T21:34:09 – 23:14:22 |
| Run 1, E3 (339 rows, `results/e3_confirmatory/`) | 2026-04-16T20:59:00 – 22:40:06 |
| H4/H5/H6 computed on run 1 | 2026-04-15 to 2026-04-17 |
| Qwen2.5-Coder-32B model modification date on the inference host | 2026-05-05 |
| First H8 computation, on run-1 data | 2026-05-11T14:22–14:33 |
| Model digests pinned (`docs/replication/MODEL_DIGESTS.txt`) | 2026-05-11 |
| Amendment 3 filed | 2026-05-12T09:05:08 |
| Run 2, E1 (270 rows, `results/E1_full/`) | 2026-05-15T18:13:00 – 20:29:45 |
| Amendment 2 filed | 2026-05-15T19:08:09 |
| Run 2, E2 (315 rows, `results/E2_full/`) | 2026-05-15T20:30:57 – 23:16:38 |
| Run 2, E3 (339 rows, `results/E3_full/`) | 2026-05-15T23:17:30 – 2026-05-16T02:34:56 |

The raw data of both runs ship in the public repository under `data/confirmatory/`
(`run1_2026-04/`, `run2_2026-05/`); the `results/` paths above are where the runs were
written. The paper reports run 2. 61 of the 270 run-2 E1 rows (Qwen2.5-Coder-32B, tasks T001–T011)
precede the Amendment 2 filing time; Amendment 2 concerns the adversarial family, and no
run-2 E2 or E3 row precedes it. The runs are compared in
`docs/confirmatory_results/` (`scripts/review/compare_confirmatory_runs.py`): every
preregistered test is identical in the two runs; count-level values differ slightly and
only through Qwen2.5-Coder-32B.

### Commit identifiers cited on OSF

The OSF amendments cite commit hashes of the project's working history. The public
`main` branch does not contain the manuscript-draft and session-note commits of that
history, so commits that exist in both have a different hash on `main`, and
manuscript-only commits have no counterpart there. The cited objects still resolve on
GitHub by hash.

| Cited on OSF | Date (UTC) | Subject | Same change on `main` |
|---|---|---|---|
| `7b88bca` | 2026-04-15T17:54:24 | refactor(prompt_builder): rename adversarial enum to A8.x and drop no-op variant | `92bcf2e` |
| `649282d` | 2026-04-15T18:09:23 | refactor(mcnemar): align condition labels with A8.x naming | `e7d29a0` |
| `d251c15` (cited in the session notes; Amendment 2 candidate) | 2026-04-15T18:54:51 | docs(prereg): file Amendment 2 candidate locally | `f501029`; d251c15 (working history) = f501029 (main), same patch-id `a1dcac98bd7abb9859992880a7806bb8d756e3d1` |
| `5558a5e` | 2026-04-15T18:31:14 | docs(paper): align non-locked sections with seven-variant scope | none (manuscript draft) |
| `6d47f71` | 2026-04-15T18:37:23 | docs(paper): document spec-vs-implementation drift in VII.B | none (manuscript draft) |
| `5147bca` | 2026-05-12T08:33:00 | docs(s29): append 2026-05-11 DHCP drift to Amendment 3 IP clarification (Obs 1) | none (amendment draft) |
| `b979881`, `e5bc8e2`, `57f620b`, `3a0756d` (Amendment 3, H8 pipeline) | 2026-05-11 | TCS extraction, schema alignment, H8 analysis, continuous metrics | `6dd1919`, `9a03c3d`, `e27561d`, `0a4222b` |

### Record note for the OSF project page (final text, 2026-09-21)

An earlier version of this section (commit `9afd1c2`) quoted two sentences from the
repository draft of Amendment 3 (`docs/OSF_AMENDMENT_3_DRAFT.md` at `5147bca`). The
amendments as filed on OSF are worded differently, so that version is superseded. The
quotations below are from the amendments as filed; they were read from the OSF record by
the author and cannot be checked against this repository, which holds only the drafts.
The note is posted on the OSF project page (wiki), not as a new amendment; there it ends
with a link to this file at the commit that introduced this section.

> Record note, 21 September 2026.
>
> The confirmatory experiments E1 to E3 were generated twice with the same generation
> code, prompts, tasks, models and decoding settings: on 15 and 16 April 2026 (run 1) and
> on 15 and 16 May 2026 (run 2). The article reports run 2. Run 1 was analysed when it
> completed, so the outcomes of H4 to H6 were known before run 2 was generated.
>
> The validity status and binary verdict of every generation agree across the two runs
> (924 of 924 rows), and all H4 to H6 test results are identical. Count-level values
> differ; for example, the mean number of violations at the final E3 retry is 11.18 in
> run 1 and 13.21 in run 2. All programs that differ between the runs were generated by
> Qwen2.5-Coder-32B; the programs of DeepSeek-Coder-V2-16B and CodeLlama-34B are
> byte-identical. The Qwen2.5-Coder-32B model on the inference host carries a modification
> date of 5 May 2026, between the two runs. Model digests were pinned on 11 May 2026.
>
> The following statements in the amendments require qualification.
>
> 1. Amendment 3 (submitted 12 May 2026, 09:05 UTC) states: "The digest active during the
>    confirmatory E1/E2/E3 runs is pinned and will be deposited in
>    docs/replication/MODEL_DIGESTS.txt at the Month-6 OSF release." At filing, only run 1
>    existed, and no digest had been recorded when it was generated. The model files of
>    DeepSeek-Coder-V2-16B and CodeLlama-34B predate run 1 (3 April and 25 March 2026) and
>    their programs are byte-identical in the two runs; the Qwen2.5-Coder-32B file was
>    modified on 5 May 2026, after run 1, so its run-1 digest is unknown. The statement
>    holds for run 2.
>
> 2. Amendment 3 states: "Amendments 1 and 2 were filed before any experimental data
>    collection." Amendment 1 (approved 7 April 2026) preceded the confirmatory data. At
>    the date of Amendment 3, Amendment 2 had not been filed, and run 1 had already been
>    generated (15 and 16 April 2026).
>
> 3. Amendment 3 registers the complexity-stratified analysis (H8) as exploratory and
>    states: "No new observations are collected for H8; the analysis is applied to the
>    existing E1/E2/E3 confirmatory results." A first version of this analysis had been
>    computed on run-1 data on 11 May 2026, the day before filing; the amendment does not
>    state this. The H8 values reported in the article are computed on run 2, which was
>    generated after filing, and the article reports H8 as exploratory, not as a
>    preregistered test.
>
> 4. Amendment 2 (submitted 15 May 2026, 19:08 UTC) states: "No confirmatory E1, E2, or E3
>    data have been inspected at the time of this filing; only the E1 pilot (3 LLMs x 5
>    tasks x 2 conditions x 1 rep, exploratory, not part of the confirmatory family) has
>    been executed." At filing, run 1 had been completed and analysed, and run 2 was in
>    progress: 61 of its 270 E1 rows preceded the submission, and none of its E2 or E3 rows
>    did. The change registered by Amendment 2 (seven instead of eight prompt-injection
>    strategies) was implemented and committed on 15 April 2026, before the first
>    generation of run 1.
>
> 5. The timing note of Amendment 2 states that "approximately 50 of 270 E1 baseline cells
>    have been executed" at submission. The exact number of run-2 E1 rows with a timestamp
>    before the submission time is 61.
>
> The commit hashes cited in the amendments refer to the project's working history; they
> resolve on GitHub by hash but are not reachable from any branch. Their counterparts on
> the public main branch, the full UTC timeline, and the raw data of both runs are in the
> public repository: [link to this file, section "Record notes — 2026-09-21"] and
> data/confirmatory/.

Where each statement of the note can be checked in this repository: the run comparison
in `docs/confirmatory_results/run_comparison_identity.csv` and
`run_comparison_headline.csv` (`scripts/review/compare_confirmatory_runs.py`); the row
timestamps in `data/confirmatory/*/e?_results.csv`; the model file dates in
`docs/replication/MODEL_DIGESTS.txt` (host-local time, UTC+3); the first H8 computation in
the messages of commits `e27561d` and `0a4222b`; the Amendment 2 code change in `92bcf2e`
and `e7d29a0`. The generation-side code (`enfield_llm/enfield_llm/prompt_builder.py`, the
client, the task files) is identical at the repository states of the two runs; the runner
differs only in a result-field fix, a log banner and summary fields.

Not part of the OSF note: Amendment 3 uses the labels H7 and H8 for the simulator-based
execution check and the complexity analysis. The manuscript as first submitted used the
same labels for two post-hoc hypotheses (saturation explains the adversarial null; refusal
is near zero); the revised manuscript drops those labels (Appendix A).
