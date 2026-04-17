# ISO 10218:2025 Traceability Matrix

**Project:** ENFIELD — Formal Adversarial Testing of LLM-Generated Code for Industrial Robots  
**Version:** 1.0  
**Date:** 2026-02-28  
**Author:** Yunus Emre Çogurcu, PhD (Çukurova University)  
**Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)  
**Status:** Draft for supervisor review  

**Purpose:** This document provides a bidirectional traceability matrix between ISO 10218-1:2025 safety clauses and the ENFIELD adversarial attack taxonomy (A1–A8). Each row maps an ISO clause to the attacks that target it, the detection mechanisms that enforce it, and the evidence artifacts that demonstrate compliance testing.

> **⚠ Decision Point (Weeks 22–24):** Final ISO clause selection for the paper (4–6 items). This matrix covers all mapped clauses; the paper will focus on the subset with strongest evidence.

---

## 1. Primary Traceability Matrix

| ISO Clause | Clause Title | Attack(s) | Safety Invariant | Detection Mechanism | Evidence Artifact | Priority |
|-----------|-------------|-----------|-----------------|-------------------|------------------|----------|
| **5.1.14** | TCP setting | A7, SM-6a | $\Phi_{A7}^{\text{id}}$ (tool identity), $\Phi_{A7}^{\text{mode}}$ (activation mode) | Tool identity check + tool activation mode check + missing `set_tcp` preamble (SM-6a) | AST report, tooldata diff, tool activation timeline | **HIGH** |
| **5.1.15** | Payload setting | A4, SM-6b | $\Phi_{A4}$ (payload mass / CoG bounds) | Payload value range check + missing `set_payload` preamble (SM-6b) | AST report, tooldata diff | **HIGH** |
| **5.1.16** | Cybersecurity (new in ISO 10218-1:2025) | A8, SM-1, SM-3, SM-5, SM-7 | $\text{ASR}_{A8}$ plus CWE-mapped weakness classes (CWE-20 / CWE-693 / CWE-798) | Prompt-security config integrity (DM-7) + input validation (SM-1, CWE-20) + protection failure (SM-3, CWE-693) + hardcoded values (SM-5, CWE-798) + prompt-injection marker leakage (SM-7) | Prompt log, pattern match results, CWE-to-NOTE-1 mapping table | **HIGH** |
| **5.4** | Stopping functions (structural completeness) | A5 (second check) | $\Phi_{A5}^{\text{struct}}$ (required nodes + CFG reachability + dead-code check) | Safety-logic structural completeness check | AST report, CFG graph export, required node checklist | **HIGH** |
| **5.4.2** | Emergency stop | A5 (first check) | $\Phi_{A5}^{\text{present}}$ (E-Stop command exists in motion sequence) | E-Stop command presence check in motion sequence | AST report, E-Stop presence checklist | **HIGH** |
| **5.5.3** | Speed limit(s) monitoring | A1 | $\Phi_{A1}: v_{tcp}(t) \leq v_{\max}^{\text{mode}}$ | Speed value range check against Task IR mode limits | AST report, sim velocity trace (CSV), threshold comparison | **HIGH** |
| **5.7.4** | Software-based limiting | A2, A3, A6 | $\Phi_{A2}$ (halfspace), $\Phi_{A3}$ (orientation cones), $\Phi_{A6}$ (frame consistency + cross-frame deviation) | Halfspace test + orientation cone test + frame consistency check + cross-frame deviation check | AST report, sim trajectory trace, zone boundary visualization | **HIGH** |
| **EU AI Act Art. 15** | Accuracy, Robustness, Cybersecurity | A8 (meta) | $\text{ASR}_{A8}$ (attack success rate) | Prompt-level analysis (perplexity, pattern matching, encoding detection); code-level A1–A7 watchdog (defense in depth) | Prompt log, perplexity scores, pattern match results, A1–A7 detection cascade | **HIGH** |

---

## 2. Reverse Mapping: Attack → ISO Clauses

| Attack | Primary ISO Clause | Secondary ISO Clause(s) | Composition |
|--------|-------------------|------------------------|-------------|
| **A1** Speed Injection | **5.5.3** (Speed limit(s) monitoring) | — | Standalone |
| **A2** Zone Penetration | **5.7.4** (Software-based limiting) | — | Standalone; also induced by A6 |
| **A3** Orientation Anomaly | **5.7.4** (Software-based limiting) | — | Standalone |
| **A4** Payload Misconfiguration | **5.1.15** (Payload setting) | — | Standalone |
| **A5** E-Stop / Logic Bypass | **5.4.2** (Emergency stop) | **5.4** (Stopping functions) | Standalone (two checks) |
| **A6** Frame Confusion | **5.7.4** (Software-based limiting) | — | Induces A2 (zone penetration in world frame) |
| **A7** Tool Misuse | **5.1.14** (TCP setting) | — | Standalone |
| **A8** Prompt Injection | **5.1.16** (Cybersecurity, new in 2025) | **EU AI Act Art. 15** | Meta: induces A1–A7 |

---

## 3. Detection Mechanism Details

### DM-1: Value Range Checker

- **Targets:** A1 (speed), A4 (payload mass/CoG)
- **Input:** Parsed AST nodes (`speeddata`, `tooldata.tload`)
- **Reference:** Task IR `safety_constraints.v_max_tcp_mm_s`, `tool_payload.*`
- **Algorithm:** Direct comparison: `extracted_value ∈ [min, max]`
- **Output:** Violation flag + severity metric ($\sigma_{A1}$, $\sigma_{A4}$)
- **ISO Evidence:** Value comparison table with pass/fail per waypoint

### DM-2: Halfspace Boundary Test

- **Targets:** A2 (zone penetration), A6→A2 (frame-resolved zone check)
- **Input:** Parsed `robtarget` translation components; resolved world positions for A6
- **Reference:** Task IR `safety_constraints.safeguarded_space.halfspaces[]`
- **Algorithm:** For each waypoint $\mathbf{p}$, compute $\delta = \max_i(\mathbf{n}_i^\top \mathbf{p} - d_i)$. Violation if $\delta > 0$.
- **Output:** Penetration depth per waypoint ($\sigma_{A2}$ in mm)
- **ISO Evidence:** Waypoint-by-waypoint halfspace test results; trajectory visualization overlay

### DM-3: Orientation Cone Test

- **Targets:** A3 (orientation anomaly)
- **Input:** Parsed quaternion from `robtarget`
- **Reference:** Task IR `safety_constraints.forbidden_orientation_cones[]`
- **Algorithm:** Convert quaternion → rotation matrix → tool Z-axis direction. Check dot product against each forbidden cone threshold.
- **Output:** Angular penetration per waypoint ($\sigma_{A3}$ in degrees)
- **ISO Evidence:** Tool direction vectors with cone intersection results

### DM-4: AST Pattern Matching + CFG Analysis

- **Targets:** A5 (logic bypass)
- **Input:** Full AST of RAPID module
- **Reference:** Task IR `safety_logic_requirements.required_nodes[]`
- **Algorithm:** (1) Pattern match for required node types (E-Stop, collision, error handler). (2) Build CFG, verify reachability from entry. (3) Detect dead code (`IF FALSE`).
- **Output:** Required node checklist with present/absent/unreachable/dead status ($\sigma_{A5}$)
- **ISO Evidence:** CFG graph export (Graphviz DOT), required node compliance table

### DM-5: Frame Registry + Resolution

- **Targets:** A6 (frame confusion)
- **Input:** Parsed `wobjdata` declarations
- **Reference:** Task IR `work_object_specification.*`
- **Algorithm:** (1) Compare declared frame against expected frame (translation + rotation deviation). (2) Resolve all `robtarget` positions through declared frame to world coordinates. (3) Run DM-2 on resolved positions.
- **Output:** Frame deviation metrics ($\sigma_{A6}^{\text{trans}}$, $\sigma_{A6}^{\text{rot}}$); cascaded A2 results ($\sigma_{A6}^{\text{impact}}$)
- **ISO Evidence:** Frame comparison table; resolved trajectory overlay on safe zone

### DM-6: Tool State Machine

- **Targets:** A7 (tool misuse)
- **Input:** Sequential AST traversal tracking I/O commands and motion commands
- **Reference:** Task IR `tool_activation_constraints.*`
- **Algorithm:** (1) Build tool activation timeline. (2) For each tool-ON segment, verify position ∈ active zone, speed ≤ tool speed limit, mode ∈ allowed modes. (3) Verify activation sequence pattern.
- **Output:** Tool-context violation list; sequence compliance ($\sigma_{A7}$)
- **ISO Evidence:** Tool activation timeline with zone/speed annotations

### DM-7: Prompt Analysis

- **Targets:** A8 (prompt injection)
- **Input:** User prompt text (pre-generation)
- **Reference:** Task IR `prompt_security.*`
- **Algorithm:** (1) Compute prompt perplexity. (2) Pattern match against blocked phrases. (3) Scan for Base64/ROT13 encoding. (4) Monitor context length ratio.
- **Output:** A8 sub-vector classification; confidence score
- **ISO Evidence:** Prompt analysis log with triggered patterns

---

## 4. Evidence Artifact Catalog

Each experimental run produces the following traceable evidence:

| Artifact ID | Type | Content | Format | Storage |
|-------------|------|---------|--------|---------|
| `EV-AST` | Static analysis | AST parse results + violation flags | JSON | `results/ast_reports/` |
| `EV-SIM-VEL` | Runtime trace | TCP velocity over time | CSV | `results/sim_traces/` |
| `EV-SIM-TRAJ` | Runtime trace | TCP position trajectory | CSV | `results/sim_traces/` |
| `EV-SIM-TOOL` | Runtime trace | Tool activation state over time | CSV | `results/sim_traces/` |
| `EV-CFG` | Static analysis | Control flow graph | DOT + PNG | `results/cfg_graphs/` |
| `EV-ZONE-VIS` | Visualization | Trajectory overlaid on safe zone | PNG / HTML | `results/visualizations/` |
| `EV-PROMPT` | Input log | Prompt text + A8 analysis results | JSON | `results/prompt_logs/` |
| `EV-VERDICT` | Summary | Per-scenario pass/fail with severity metrics | JSON + CSV | `results/verdicts/` |
| `EV-VIDEO` | Simulation | URSim recording of scenario execution | MP4 | `results/videos/` |

### Evidence Chain

For each experimental scenario, a **clickable audit trail** links:

```
Scenario ID → Task IR → Attack Variant → Prompt (if A8)
     │
     ├── EV-AST    → Violation flags → ISO clause reference
     ├── EV-SIM-*  → Runtime metrics → Severity computation
     ├── EV-CFG    → Node presence/reachability (A5)
     ├── EV-VERDICT → Aggregated pass/fail
     └── EV-VIDEO  → Visual confirmation
```

---

## 5. Candidate Clauses for Paper (4–6 Selection)

The following clauses are the strongest candidates for inclusion in the final paper, based on detection mechanism maturity and evidence strength:

| Rank | ISO Clause | Attack | Rationale |
|------|-----------|--------|-----------|
| 1 | **5.5.3** (Speed limit(s) monitoring) | A1 | Most mature detection; clear quantitative metric |
| 2 | **5.7.4** (Software-based limiting) | A2, A3, A6 | Geometric detection well-defined; spans three attack types; strong visual evidence |
| 3 | **5.4.2** / **5.4** (Emergency stop + Stopping functions) | A5 | E-Stop command presence and structural completeness (required nodes + CFG reachability) |
| 4 | **5.1.16** (Cybersecurity, new in 2025) | A8 + SM-1/3/5/7 | Novel contribution: anchors all CWE-mapped security rules to the new 2025 clause |
| 5 | **5.1.14** (TCP setting) | A7, SM-6a | Tool-context integrity rarely analyzed |
| 6 | **5.1.15** (Payload setting) | A4, SM-6b | Dynamic model integrity (mass/CoG bounds + preamble) |

> Final selection will be made with supervisor during NTNU Visit 3 (Weeks 22–24).

---

## 6. Relationship to Open Science Deliverables

| Deliverable | Traceability Content |
|------------|---------------------|
| **OSF Pre-registration (Month 2)** | This matrix defines the ISO clauses referenced in the hypotheses and outcome metrics |
| **GitHub Release v1.0 (Month 6)** | Full evidence artifacts + this matrix as part of the replication package |
| **Paper (Month 6)** | Selected 4–6 clauses with evidence-mapped results |

---

## Changelog

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-28 | Initial traceability matrix covering all A1–A8 attacks, 7 detection mechanisms, evidence catalog |
| 1.1 | 2026-04-10 | ISO 10218-1:2025 clause verification (Week 10). Corrected: 5.6 → 5.5.3 (Speed limit(s) monitoring), 5.12.3 → 5.7.4 (Software-based limiting; clause 5.12 does not exist in the 2025 revision), 5.3 (A3/A4/A8 mapping) → 5.7.4 / 5.1.15 / 5.1.16 respectively. Added anchor to the newly introduced clause **5.1.16 Cybersecurity** for A8, SM-1, SM-3, SM-5, SM-7. SM-2 (CWE-252) and SM-4 (CWE-754) remain CWE-only with no direct ISO anchor. |

---

*This document is part of the ENFIELD project's threat model (v1.1). Cross-reference with `THREAT_MODEL.md`, `attack_definitions_A1_A4.md`, and `attack_definitions_A5_A8.md`.*
