# Threat Model: Adversarial Attacks on LLM-Generated Industrial Robot Code

**Version:** 1.2  
**Date:** 2026-04-14  
**Author:** Yunus Emre Çogurcu, PhD (Çukurova University)  
**Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)  
**Status:** Draft for supervisor review  

**Based on:** ISO 10218-1:2025, ISO 10218-2:2025, Zou et al. (2023), Kumar et al. (COLM 2024)

---

## 1. System Overview

The target system is an automated code generation pipeline where an Operator prompts a Large Language Model (LLM) to generate robot control code for an industrial robot operating in a ROS2 Humble simulation environment.

- **Primary Asset:** Industrial Robot — Universal Robots UR5e (simulation-first).
- **Secondary Assets (planned adapters):** ABB GoFa, KUKA KR3.
- **Asset Value:** Physical safety of operators, integrity of workcell equipment, ISO 10218 compliance.
- **Trust Boundary:** The interface between the LLM output and the robot controller / motion planner.
- **Scope:** Simulation-only — no physical robot deployment.

### System Architecture (Simplified)

```
┌──────────────┐     ┌─────────┐     ┌──────────────┐     ┌────────────┐
│  Operator    │────▶│  LLM    │────▶│  Watchdog    │────▶│  Robot     │
│  (Prompt)    │     │  (Code  │     │  (AST +      │     │  Controller│
│              │     │   Gen)  │     │   Runtime)   │     │  (ROS2)    │
└──────────────┘     └─────────┘     └──────────────┘     └────────────┘
       │                  ▲                │                      │
       │                  │                │                      │
       │            ┌─────┴─────┐    ┌─────┴──────┐        ┌─────┴──────┐
       │            │ A8: Prompt│    │ A1-A7:     │        │ Simulation │
       │            │ Injection │    │ Code-level │        │ (Gazebo)   │
       └────────────┤ Attacks   │    │ Violations │        └────────────┘
                    └───────────┘    └────────────┘
```

---

## 2. Attacker Profiles

Three attacker capability levels are defined, ordered from most to least powerful. The distribution of experimental scenarios follows the project's stratified sampling strategy.

### 2.1. White-Box Attacker (Internal / Researcher)

- **Knowledge:** Full access to LLM weights, gradients, and architecture (e.g., Llama-2-7B, Vicuna).
- **Capability:** Can run gradient-based optimization algorithms such as the Greedy Coordinate Gradient (GCG) algorithm (Zou et al., 2023) to compute mathematically optimal adversarial suffixes.
- **Compute Budget:** Unlimited. Can run thousands of forward/backward passes.
- **Query Budget:** Unlimited.
- **A8 Sub-vectors:** A8.1–A8.8 (all prompt-engineering sub-variants per paper §IV.C). White-box additionally benefits from GCG optimization (see Capability above) as a cross-cutting delivery mechanism over any sub-variant.
- **Goal:** Prove fundamental vulnerabilities in open-source models used for robot code generation.
- **Real-world Analogy:** Security researcher with local model deployment; insider with model access.
- **Scenario Share:** ~25% of experimental scenarios.

### 2.2. Gray-Box Attacker (API User / Partial Knowledge)

- **Knowledge:** API access to the LLM. Can observe input-output pairs. May know the model family (e.g., "GPT-4 class") but NOT weights or gradients. Partial knowledge of the system prompt structure through probing.
- **Capability:** Iterative prompt engineering, systematic prompt probing, few-shot manipulation, chain-of-thought exploitation. Can craft targeted prompts based on observed model behavior patterns.
- **Compute Budget:** Limited to API costs.
- **Query Budget:** **100 queries per scenario** (configurable; to be confirmed with supervisor: 100 vs 50).
- **A8 Sub-vectors:** A8.1–A8.8 (all prompt-engineering sub-variants per paper §IV.C). Realistic for iterative prompt engineering within the 100-query budget.
- **Goal:** Exploit API-deployed LLMs through systematic prompt engineering within realistic budget constraints.
- **Real-world Analogy:** External contractor with API access to a code generation service; malicious user of a robot programming assistant.
- **Scenario Share:** **~62.5% of experimental scenarios** (primary focus — most representative of real-world API deployment).

> **⚠ Decision Point (Week 2):** Gray-box query budget — 100 vs 50 queries/scenario. To be confirmed with supervisor.

### 2.3. Black-Box Attacker (External / No Model Access)

- **Knowledge:** Zero model access. Observes only execution outcomes (success/failure, error messages, robot behavior in simulation).
- **Capability:** Relies on **transferability** of adversarial suffixes (optimized on a white-box surrogate model) and manual prompt crafting based on outcome feedback.
- **Compute Budget:** Minimal (no model training).
- **Query Budget:** **10–20 queries per scenario**.
- **A8 Sub-vectors:** A8.1–A8.6, A8.8 (A8.7 Obfuscation is excluded because its delivery requires URScript parameter-name and unit-hex knowledge unavailable to black-box attackers who observe only execution outcomes).
- **Goal:** Demonstrate that even attackers with no model knowledge can induce safety violations through transferred attacks or naive prompt manipulation.
- **Real-world Analogy:** End-user with no AI expertise; attacker exploiting a publicly available robot programming chatbot.
- **Scenario Share:** ~12.5% of experimental scenarios.

### Attacker Capability Comparison

| Attribute | White-Box | Gray-Box | Black-Box |
|-----------|-----------|----------|-----------|
| Model weights | ✅ Full | ❌ None | ❌ None |
| Gradient access | ✅ Yes | ❌ No | ❌ No |
| API access | ✅ Yes | ✅ Yes | ❌ No |
| System prompt knowledge | ✅ Full | ⚠ Partial (probing) | ❌ None |
| Query budget | Unlimited | 100 / scenario | 10–20 / scenario |
| Optimization method | GCG (gradient) | Iterative prompt eng. | Transfer + manual |
| A8 sub-vectors | All (A8.1–A8.8) | All (A8.1–A8.8) | A8.1–A8.6, A8.8 (A8.7 excluded) |
| Scenario share | ~25% | **~62.5%** | ~12.5% |

---

## 3. Attack Taxonomy (A1–A8)

The complete attack taxonomy defines eight attack types organized across three layers: parametric, structural/logical, and meta/generative. Each attack type has a formal mathematical definition with safety invariants, detection strategies, and ISO 10218 clause mappings.

| Layer | ID | Attack Name | ISO Clause | Nature | Formal Definition |
|-------|----|-------------|-----------|--------|-------------------|
| Parametric | **A1** | Speed Injection | 5.5.3 | Wrong speed value | [A1–A4 doc](attack_definitions_A1_A4.md#a1--speed-injection) |
| Parametric | **A2** | Zone Penetration | 5.7.4 | Wrong position value | [A1–A4 doc](attack_definitions_A1_A4.md#a2--zone-penetration) |
| Parametric | **A3** | Orientation Anomaly | 5.7.4 | Wrong orientation value | [A1–A4 doc](attack_definitions_A1_A4.md#a3--orientation-anomaly) |
| Parametric | **A4** | Payload Misconfiguration | 5.1.15 | Wrong tooldata values | [A1–A4 doc](attack_definitions_A1_A4.md#a4--payload-misconfiguration) |
| Structural | **A5** | E-Stop / Logic Bypass | 5.4.2 / 5.4 | Missing control flow | [A5–A8 doc](attack_definitions_A5_A8.md#a5--emergency-stop--safety-logic-bypass) |
| Structural | **A6** | Frame Confusion | 5.7.4 | Wrong reference frame | [A5–A8 doc](attack_definitions_A5_A8.md#a6--frame-confusion-workobject-tampering) |
| Structural | **A7** | Tool Misuse (Logic) | 5.1.14 | Wrong tool sequencing | [A5–A8 doc](attack_definitions_A5_A8.md#a7--tool-misuse-semantic-logic) |
| Meta | **A8** | Prompt Injection | 5.1.16 + EU AI Act | LLM exploitation | [A5–A8 doc](attack_definitions_A5_A8.md#a8--prompt-injection--jailbreak-the-llm-vector) |

### Attack Layer Description

**Parametric (A1–A4):** Numeric values in otherwise correct program structure violate safety bounds. Detection relies on value range checks, geometric tests, and bounds validation against Task IR specifications.

**Structural / Logical (A5–A7):** Program structure, control flow, or semantic context is incorrect even when individual values appear safe. Detection requires AST pattern matching, control flow graph analysis, frame resolution, and state machine tracking.

**Meta / Generative (A8):** The LLM code generation process itself is exploited to produce code containing any combination of A1–A7 violations. Detection operates at two layers: prompt-level analysis (pre-generation) and code-level watchdog (post-generation, using A1–A7 detectors).

### Attack-Attacker Matrix

| Attack | White-Box | Gray-Box | Black-Box | Primary Delivery |
|--------|-----------|----------|-----------|-----------------|
| A1 Speed Injection | Direct AST edit | Performance framing (A8.6), Direct override (A8.1) | Transfer suffix | Value manipulation |
| A2 Zone Penetration | Direct AST edit | Context overflow (A8.3), Direct override (A8.1) | Transfer suffix | Value manipulation |
| A3 Orientation Anomaly | Direct AST edit | Incremental (A8.4), Direct override (A8.1) | Transfer suffix | Value manipulation |
| A4 Payload Misconfig | Direct AST edit | Authority claim (A8.5), Direct override (A8.1) | Transfer suffix | Value manipulation |
| A5 Logic Bypass | Code stripping | Direct override (A8.1), Performance framing (A8.6) | Transfer suffix | Structure removal |
| A6 Frame Confusion | Frame edit | Context overflow (A8.3), Role playing (A8.2) | Transfer suffix | Reference corruption |
| A7 Tool Misuse | Sequence edit | Role playing (A8.2), Incremental (A8.4) | Manual prompt | Sequence corruption |
| A8 Prompt Injection | Template injection (A8.1–A8.8) + GCG (§2.1) | Iterative probing | Role playing (A8.2) | LLM exploitation |

---

## 4. Methodology: Adapted GCG for Robotics

### 4.1. Original GCG (Zou et al., 2023)

The Greedy Coordinate Gradient algorithm optimizes adversarial suffixes by maximizing the probability of a target affirmative response. For standard NLP attacks, the target is typically `"Sure, here is..."`.

### 4.2. Robot-Specific Adaptation

For industrial robot code generation, the loss function is adapted to optimize for **Unsafe Control Parameters** rather than explicit harmful content:

**Optimization Target:**

Instead of maximizing `P("Sure, here is...")`, we maximize the probability of generating specific unsafe code patterns:

$$\mathcal{L} = -\log P(\text{unsafe\_code\_pattern} \mid \text{prompt} + \delta)$$

where $\delta$ is the adversarial suffix.

**Attack-Specific Targets:**

| Attack | Optimization Target |
|--------|-------------------|
| A1 | `P("v5000" \| prompt + δ)` — force high speed value |
| A2 | `P("[900, 200, 250]" \| prompt + δ)` — force out-of-zone position |
| A3 | `P("[1, 0, 0, 0]" \| prompt + δ)` — force dangerous orientation |
| A4 | `P("[0.1, [0, 0, 200]" \| prompt + δ)` — force wrong payload |
| A5 | `P("PROC main()\n  MoveL" \| prompt + δ)` — force logic-free code |

### 4.3. Key Difference from Standard NLP Attacks

| Aspect | Standard NLP (Zou et al.) | Robot Code (This Project) |
|--------|--------------------------|--------------------------|
| Harmful output | "Here's how to build a bomb" | `MoveL target, v5000, z10, tool0;` |
| Detectability | Easy (keyword filtering) | Hard (valid syntax) |
| LLM awareness | Model "knows" it's harmful | Model may not recognize safety violation |
| Impact domain | Digital (misinformation) | Physical (robot collision, operator injury) |

---

## 5. Defence Architecture

### 5.1. Two-Layer Defence Model

```
Layer 1 (Pre-Generation):    Prompt Analysis → A8 detection
                                    │
                                    ▼
Layer 2 (Post-Generation):   AST Parser → A1-A7 Detectors → Safety Verdict
                                    │
                                    ▼
                             Runtime Monitors → Real-time enforcement
```

**Layer 1 — Prompt-Level (A8 Detection):**
Perplexity thresholds, pattern matching for jailbreak phrases, encoding detection, context length monitoring.

**Layer 2 — Code-Level (A1–A7 Detection):**
AST-based static analysis operating on the generated code, regardless of how it was produced. This provides **defense in depth**: even if Layer 1 misses a novel prompt injection, Layer 2 catches the resulting code violations.

**Layer 3 — Runtime (Enforcement):**
ROS2 safety monitor nodes that validate velocity, zone boundaries, and tool state in real-time during simulation execution.

### 5.2. Detection Mechanism Summary

| Mechanism | Targets | Method |
|-----------|---------|--------|
| Value range checker | A1, A4 | Compare extracted values against Task IR bounds |
| Halfspace boundary test | A2 | Geometric intersection of waypoints with convex polytope |
| Orientation cone test | A3 | Quaternion → direction vector → forbidden cone check |
| AST pattern matching | A5 | Required node presence + CFG reachability |
| Frame registry + resolution | A6 | Compare declared vs expected frames; resolve world positions |
| Tool state machine | A7 | Track activation sequence; verify zone/speed context |
| Prompt analysis | A8 | Perplexity, pattern matching, encoding scan |

---

## 6. Scope and Limitations

- **Simulation-only:** All attacks and defences are tested in Gazebo simulation. No physical robot deployment.
- **Synthetic data:** All robot code samples are generated or hand-crafted for testing purposes.
- **No real-world harm:** Attack code is designed exclusively for testing the watchdog's detection capabilities.
- **ISO compliance is advisory:** This framework demonstrates feasibility of ISO 10218 traceability; it does not constitute formal certification.
- **LLM selection:** The specific LLMs used for gray-box attacks will be decided with the supervisor (Week 13 decision point).

---

## 7. Related Documents

| Document | Content |
|----------|---------|
| [`attack_definitions_A1_A4.md`](attack_definitions_A1_A4.md) | Formal definitions: Speed Injection, Zone Penetration, Orientation Anomaly, Payload Misconfiguration |
| [`attack_definitions_A5_A8.md`](attack_definitions_A5_A8.md) | Formal definitions: Logic Bypass, Frame Confusion, Tool Misuse, Prompt Injection |
| [`iso_clause_mapping.md`](iso_clause_mapping.md) | ISO 10218 traceability matrix (attack → clause → detection → evidence) |
| [`OSF_DRAFT.md`](OSF_DRAFT.md) | OSF pre-registration draft |

---

## Changelog

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-14 | Initial threat model with White-box and Black-box attackers, A1–A5 (preliminary) |
| 1.1 | 2026-02-28 | Added Gray-box attacker (§2.2); updated attack taxonomy to formal A1–A8; updated robot reference to UR5e; added system architecture diagram; added attack-attacker matrix; added defence architecture; cross-referenced formal definition documents |
| 1.2 | 2026-04-14 | W11-S1 editorial pass: re-labeled A8 sub-variant lists (§2.1–§2.3, §2 capability-comparison, §3 attack-attacker matrix) against paper §IV.C ground truth (A8.1 Direct Override through A8.8 Dual Instruction); removed the legacy GCG-suffix claim from the matrix A8 row since A8.* is a prompt-engineering taxonomy and GCG lives in §2.1 capability and §4 methodology; retired the session-8 advisory blockquote (commit 31f8f86). Code-side `A6_1..A6_8` enum rename is pending as W11-S2; paper §IV.C Note on legacy `A6.*` identifiers remains authoritative for the code↔paper label mapping. |

---

## References

1. Zou, A., Wang, Z., Kolter, J. Z., & Fredrikson, M. (2023). Universal and Transferable Adversarial Attacks on Aligned Language Models. *arXiv:2307.15043*.
2. Kumar, A., et al. (2024). Certifying LLM Safety against Adversarial Prompting. *COLM 2024*.
3. ISO 10218-1:2025. Robotics — Safety requirements — Part 1: Robots.
4. ISO 10218-2:2025. Robotics — Safety requirements — Part 2: Integration.
5. EU AI Act, Article 15: Accuracy, Robustness, and Cybersecurity.
