# Open Science Release Checklist

**Project:** ENFIELD — Formal Adversarial Testing of LLM-Generated Code for Industrial Robots
**Version:** 5.0
**Date:** 2026-03-22

This document tracks compliance with ENFIELD Open Science requirements and EU Horizon Europe open access obligations.

---

## Month 2 Milestone (March 2026): OSF Pre-registration ✅

**DOI:** [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2)
**Template:** OSF Standard Pre-Data Collection Registration
**Submitted:** 2026-03-22
**Approved by:** Assoc. Prof. Georgios Spathoulas (NTNU)

| Item | Status | Notes |
|------|--------|-------|
| OSF account created | Done | osf.io |
| Pre-registration template selected | Done | OSF Standard Pre-Data Collection |
| Hypotheses specified (H1, H0, H2, H3) | Done | See `docs/OSF_PREREGISTRATION.md` |
| Statistical analysis plan (McNemar, Holm-Bonferroni) | Done | alpha=0.05, beta=0.20, n>=25 pairs |
| Primary outcome defined (SV reduction >=40%) | Done | |
| Secondary outcomes defined (per-attack rate, FP rate) | Done | |
| Pilot data disclosed (5 tasks, 40 variants) | Done | 90% detection, 0% FP |
| Full suite data generated (15 tasks, 120 variants) | Done | 95% detection, 0% FP |
| Pre-registration submitted | Done | 2026-03-22 |
| DOI obtained | Done | 10.17605/OSF.IO/VE5M2 |
| CITATION.cff updated with DOI | Done | 2026-03-22 |
| README.md badge updated with DOI link | Done | 2026-03-22 |

### Post-Approval Amendment (Week 8)

Georgios approved the pre-registration and provided feedback requiring two extensions:

1. **Security dimension:** Add CWE-based security analysis (SM-1..7) alongside existing safety rules (DM-1..7)
2. **LLM integration:** Add actual LLM code generation (Qwen2.5-Coder-32B (Apache 2.0), DeepSeek-Coder-V2-16B (DeepSeek License), CodeLlama-34B (Llama License)) with adversarial prompt testing (A6.1–A6.8)

New hypotheses (H4–H6) and the Refusal Rate (RR) metric were registered via OSF Amendment 1 (filed 2026-04-06, **approved by OSF admin 2026-04-07**), formalising changes made between Week 8 and Week 10. See `docs/OSF_PREREGISTRATION.md` Amendment 1 section.

| New Hypothesis | Description |
|----------------|-------------|
| H4 | ≥30% of baseline LLM-generated code contains ≥1 safety or security violation |
| H5 | Adversarial prompts (A8.1–A8.8) increase combined violation rate by ≥50 pp vs baseline |
| H6 | Watchdog-in-the-loop reduces violation rate by ≥40% relative to single-pass generation |

---

## Month 6 Milestone (July 2026): Public Release

### Code Release (GitHub)

| Item | Status | Notes |
|------|--------|-------|
| All ROS2 packages build cleanly | Done | `colcon build` green |
| All tests pass (740 total) | Done | 169+149+81+114+95+132 across 6 suites |
| CI pipeline green (9 jobs) | Done | build + 6 test suites + docker + sbom |
| LICENSE file present (Apache-2.0) | Done | |
| CITATION.cff complete with DOIs | Done | OSF DOI added 2026-03-22, v0.2.0 |
| README installation instructions verified | Done | Docker + native |
| No proprietary dependencies | Done | All OSI-licensed |
| LLM client package (enfield_llm) | Done | Qwen2.5-Coder-32B (Apache 2.0), DeepSeek-Coder-V2-16B (DeepSeek License), CodeLlama-34B (Llama License) — Ollama backends |
| Security rules (SM-1..7) | Pending | CWE-based detection in watchdog |
| LLM experiment runner | Pending | Orchestrate E1/E2/E3 experiments |
| Git tag `v1.0` created | Pending | |
| GitHub Release created with changelog | Pending | |

### Supply Chain Transparency (SBOM Scope, Model Pins, Image Pins)

The replication package rests on four complementary transparency mechanisms. This subsection documents the scope of each so reviewers and replicators understand which supply-chain artefact answers which question, and where the gaps are.

**1. CycloneDX SBOM (`sbom-and-scan` CI job).** The CI workflow generates a CycloneDX-format Software Bill of Materials at every push to `main`. The current SBOM scope is **Python runtime dependencies only**, generated from `requirements.txt` via `CycloneDX/gh-python-generate-sbom`. The artefact is uploaded as the `sbom` workflow artefact and is the authoritative inventory for the Python layer of the ENFIELD package set (`enfield_tasks`, `enfield_attacks`, `enfield_translators`, `enfield_watchdog_static`, `enfield_llm`, `experiment_runner`, `enfield_urscript_runtime`). The CycloneDX schema captures package name, version, license, and PURL; this is sufficient for reviewer-level supply-chain audit and for grant-funder defensibility.

**2. Trivy vulnerability scan (`sbom-and-scan` CI job).** The same CI job also runs a Trivy scan against the built `enfield:scan` Docker image (the multi-stage Dockerfile `runtime` target). The Trivy scan covers the full container image — base OS layer, apt packages, Python packages, and any other content inside the image — and emits CVEs at CRITICAL and HIGH severity to `trivy-report.json`. Trivy's coverage is therefore broader than the CycloneDX SBOM; the two artefacts answer different questions (Trivy: "what known vulnerabilities exist in the image?"; CycloneDX: "what packages are in the Python dependency graph?").

**3. Model weight pins (`docs/replication/MODEL_DIGESTS.txt`).** Model weights are out of scope for the CycloneDX SBOM and out of scope for Trivy: the LLM models live on PC2 (Windows + Ollama) and are pulled at experiment time, not baked into the Docker image. The authoritative record of the three Ollama model manifests used by ENFIELD is `docs/replication/MODEL_DIGESTS.txt`, which pins each model by name, quantization, sha256 manifest digest, and `modified_at` timestamp. The file is committed at every Ollama-state observation point (current snapshot: 2026-05-11, S28). The OSF Amendment 3 Operational Disclosures section documents the digest-drift and quantization-drift policy.

**4. URSim image digest pin (`docker/ursim/Dockerfile`).** The URSim simulation host is built from a digest-pinned base image: `FROM universalrobots/ursim_e-series:5.12@sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51` (line 24). The digest is also recorded as image metadata via the `enfield.ursim.base.digest` label, and the URCap layer added on top is itself sha256-pinned via the `enfield.urcap.sha256` label. The URSim image is currently built and validated only on the project's PC1 development host; there is no `docker-build-ursim` job in CI yet.

**Scope gaps acknowledged.**

- The CycloneDX SBOM does **not** include the URSim image layers, the Ollama model weights, or apt-level system dependencies of the main `enfield:scan` image. A full software-bill-of-materials covering all of these would require (a) a `docker-build-ursim` CI job emitting a Syft or Grype SBOM against the URSim image, (b) a model-weight SBOM convention (none currently exists for `.gguf` quantized models, which is a community standardisation gap), and (c) apt-level SBOM generation (e.g. via Anchore or `dpkg-query` post-processing). Items (a) and (c) are listed in the post-NTNU backlog; item (b) is not a project-specific gap.
- Trivy scans the `enfield:scan` image only, not the URSim image. The URSim image is an upstream Universal Robots distribution; vulnerability scanning of that image is treated as upstream-vendor scope under the responsible-disclosure policy in `docs/responsible_disclosure.md` §3.

**Replicator guidance.** A reviewer or replicator who wishes to verify the ENFIELD supply chain end-to-end should: (i) consult the CycloneDX SBOM uploaded by the most recent CI run for Python-layer dependencies; (ii) run `trivy image enfield:scan` against a locally-built image to verify vulnerability status; (iii) verify Ollama model state against `docs/replication/MODEL_DIGESTS.txt` using `curl http://<ollama-host>:11434/api/tags`; (iv) verify the URSim base image against the digest recorded in `docker/ursim/Dockerfile` line 24.

### Replication Package (OSF)

| Item | Status | Notes |
|------|--------|-------|
| Code snapshot (zip of tagged release) | Pending | |
| SBOM artifact (CycloneDX JSON) | Done | Generated by CI `sbom-and-scan` job (Python-deps scope; see Supply Chain Transparency subsection above for full mechanism inventory) |
| Dockerfile + docker-compose.yaml | Done | Multi-stage build |
| Task IR files (15 baselines) | Done | `enfield_tasks/ir/tasks/` T001-T015 |
| Attack variant files (120 variants) | Done | `enfield_attacks/generated/variants/` |
| Static watchdog (safety: DM-1..7) | Done | `enfield_watchdog_static/` |
| Static watchdog (security: SM-1..7) | Pending | CWE-mapped rules |
| LLM client + prompt templates | Done | `enfield_llm/` |
| LLM experiment logs (JSONL) | Pending | Full request/response logs |
| Experiment runner + reports | Done | `scripts/run_experiment.py` |
| Statistical analysis scripts | Pending | McNemar + Holm-Bonferroni + Cochran's Q |
| Run instructions | Pending | `REPLICATE.md` needed |
| URSim runtime image | Done | `universalrobots/ursim_e-series:5.12` (digest `sha256:b7ad69f5...51`); validated 2026-04-27 |
| ROS 2 driver stack | Done | `ur_robot_driver` 2.12.0 + `ur_client_library` 2.7.0 on ROS 2 Humble; bring-up documented in README |

### Current Test Summary

| Package | Tests | Status |
|---------|-------|--------|
| enfield_tasks | 169 | Done |
| enfield_attacks | 149 | Done |
| enfield_translators | 81 | Done |
| enfield_watchdog_static | 114 | Done |
| enfield_llm | 95 | Done |
| experiment runner | 132 | Done |
| **Total** | **740** | Done |

> enfield_tasks breakdown: 79 static + 90 parametrized (TestAllTasksSchemaValid: 6 × 15 tasks).
> enfield_llm breakdown: 5 base client + 14 prompt builder + 12 code parser + 7 factory tests.
> All test suites use dynamic glob — future task additions require zero test changes.

### Experiment Results

**Phase A — Deterministic Testing (15-task, 120 variants):**

| Metric | Value |
|--------|-------|
| Baselines safe | 15/15 (FP: 0%) |
| Variants flagged | 114/120 (95%) |
| A1 Speed | 15/15 (100%) |
| A2 Zone | 15/15 (100%) |
| A3 Orientation | 14/15 (93%) |
| A4 Payload | 15/15 (100%) |
| A5 Logic | 15/15 (100%) |
| A6 Frame | 12/15 (80%) |
| A7 Tool | 13/15 (87%) |
| A8 Prompt | 15/15 (100%) |

**Phase B–D — LLM Code Generation Testing (planned):**

| Experiment | Description | Status |
|-----------|-------------|--------|
| E1: Baseline LLM | 3 LLMs × 15 tasks × 2 conditions × 3 reps | Pending |
| E2: Adversarial | 3 LLMs × 15 tasks × 8 attacks (A6.1–A6.8) | Pending |
| E3: Watchdog-in-loop | 3 LLMs × 15 tasks × 3 reps with retry | Pending |

---

## Responsible Disclosure (3-Tier Access)

| Tier | Content | Access | Timeline |
|------|---------|--------|----------|
| **Tier 1 (Public)** | Methods, aggregated results, detection mechanisms, Task IR schema | GitHub public | Month 6 |
| **Tier 2 (Verified Researchers)** | Full attack specifications, per-scenario results, LLM logs | OSF restricted | After verification |
| **Tier 3 (Vendors)** | Critical vulnerability details, adversarial prompt templates | Direct communication | 90 days before release |

---

## Checklist Summary

```
Month 2 (March 2026):
  [x] OSF pre-registration document finalized (v3.0)
  [x] Pilot data generated and disclosed (5 tasks, 40 variants)
  [x] Full task suite generated (15 tasks, 120 variants)
  [x] Supervisor approval obtained (Georgios Spathoulas)
  [x] OSF pre-registration submitted (2026-03-22)
  [x] DOI obtained: 10.17605/OSF.IO/VE5M2
  [x] CITATION.cff updated with DOI
  [x] README.md badge updated with DOI link
  [x] Georgios feedback: add Security dimension + LLM integration
  [x] enfield_llm package created (95 tests passing)

Month 6 (July 2026):
  [x] Task suite complete (15 tasks)
  [x] LLM client package — Qwen2.5-Coder-32B (Apache 2.0), DeepSeek-Coder-V2-16B (DeepSeek License), CodeLlama-34B (Llama License), Ollama backend
  [ ] Security rules (SM-1..7) implemented in watchdog
  [ ] LLM experiments E1/E2/E3 executed
  [x] OSF pre-registration Amendment 1 (H4–H6, RR metric, security layer, LLM infrastructure migration) — filed 2026-04-06, **approved 2026-04-07**
  [ ] Confirmatory statistical analysis (McNemar, Holm-Bonferroni, Cochran's Q)
  [ ] Git tag v1.0
  [ ] GitHub Release with changelog
  [ ] OSF replication package uploaded
  [ ] SBOM + scan reports archived
  [ ] Responsible disclosure timeline completed
  [ ] arXiv preprint submitted (target: end of June 2026)
  [ ] Paper submitted (IEEE RA-L, target: end of July 2026)
```
