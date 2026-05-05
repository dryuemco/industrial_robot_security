---
title: "ENFIELD S25 Pre-NTNU Status"
subtitle: "Demo Deck for Georgios"
author: "Yunus Emre Cogurcu (PhD, Çukurova Üniversitesi)"
date: "May 2026"
geometry: margin=1in
fontsize: 11pt
documentclass: article
mainfont: DejaVu Serif
monofont: DejaVu Sans Mono
colorlinks: true
linkcolor: blue
urlcolor: blue
---

# ENFIELD S25 Pre-NTNU Status

> **Document scope.** This deck consolidates S20–S24 deliverables ahead of NTNU
> Visit 1 (2026-06-01 to 2026-07-31). Visit 1 is now scoped exclusively to paper
> editing and submission preparation; all infrastructure, experiment, and
> simulation work is to be completed beforehand in S25 (pre-NTNU sprint).

---

## Section 1 — Project Status

### Q1: Where are we in the project timeline?

As of S25 (May 2026), the project sits at HEAD `7fd813d` (Sub-lane C skeleton, this deck). S24 closed at `85a0530` with 9 atomic commits across sub-lanes A (URScript runtime, telemetry-only), B (paper figure suite), and D (paper audit: detection latency, vendor language). The paper draft is text-complete with all 17 IEEE numeric citations resolved and 740 unit tests stable. S25 Lanes 1–5 (this deck plus URCap unblock, LLM-live URSim, Georgios sync, SBOM) must close before **NTNU Visit 1 on 2026-06-01**, which is now scoped exclusively to paper editing, arXiv preprint (mid-June 2026), and IEEE RA-L submission (mid-July 2026).

### Q2: What did S24 deliver?

Nine commits closed S24 across three sub-lanes plus a TODO-close commit:

- **Sub-lane A — URScript runtime (4 commits).** New ROS2 package `enfield_urscript_runtime` with a URScript publisher node, telemetry recorder (CSV log of `joint_pos`, `joint_vel`, `joint_torque`), T001 smoke launch, and 8 offline tests passing. Live motion execution blocked on URCap install (see Q4, Q9).
- **Sub-lane B — Paper figure suite (3 commits).** Three paper-ready figures: per-model violation rate (Fig. 1), architecture diagram (Fig. 2, Graphviz `splines=true`), retry trajectories (Fig. 3, four-panel small multiples).
- **Sub-lane D — Paper audit (1 commit).** Added §V.D detection-latency microbenchmark and §IV.C vendor-language selection rationale.
- **Sub-lane C — Demo deck.** Deferred from S24 to S25 Lane 1; this document is the result.

### Q3: System architecture overview

![ENFIELD end-to-end architecture: prompt → LLM (Ollama backends) → URScript translator → static watchdog → URSim runtime telemetry.](paper/figures/architecture.pdf){width=92%}

The framework is a three-layer pipeline (Fig. 2): LLM evaluation, static watchdog, and simulation runtime. The LLM layer queries three local models via Ollama on PC2 (`192.168.1.5:11434`): Qwen2.5-Coder-32B, DeepSeek-Coder-V2-16B, and CodeLlama-34B, all Q4-quantised. Each candidate URScript program flows through the watchdog (DM-1…DM-7 syntactic detectors plus SM-1…SM-7 CWE-mapped security rules). Violations either (i) feed back as retry prompts (E3 watchdog-in-loop) or (ii) gate forwarding to the runtime layer. The runtime layer is the URSim e-Series 5.12 simulator with a ROS2 telemetry recorder. PC1 (Ubuntu 22.04, RTX 5080) hosts development, ROS2, and the test harness; PC2 hosts model inference only. The split keeps inference deterministic across iterations and isolates GPU contention from ROS2 timing.

### Q4: URSim runtime — what works today?

**Live execution achieved (S25 Lane 2 closure).** The `enfield_urscript_runtime` package launches the custom URSim image (`enfield-ursim:5.12-urcap-1.0.5`, External Control URCap pre-baked), attaches the ROS2 `ur_robot_driver` (v2.12.0), and records joint-state telemetry to CSV. T001 smoke launch now drives the simulated UR5e end-to-end via direct URScript injection over the Secondary client interface (TCP 30002, `inject_mode='primary_tcp'`).

**T001 acceptance evidence.** A 60-second T001 run produced 19,406 joint-state samples with all six main joints moving above the 0.01 rad threshold. The five with the largest ranges:

- `shoulder_pan` range 0.97 rad (≈ 55°)
- `shoulder_lift` range 1.28 rad (≈ 73°)
- `elbow` range 0.63 rad (≈ 36°)
- `wrist_1` range 0.68 rad (≈ 39°)
- `wrist_3` range 1.64 rad (≈ 94°)

The trajectory terminated in `PROTECTIVE_STOP` (Error C204A3, `qdd` discontinuity) during late waypoint sequencing — a **motion-induced** safety event, not a connection failure. This is itself a useful finding: even hand-crafted T001 URScript triggers URSim's onboard safety monitor under the collaborative-mode 250 mm/s envelope. It establishes a baseline against which LLM-generated URScript can be measured (S26 Lane 3 sim-to-real anchor for paper §V.G).

---

## Section 2 — Empirical Findings

### Q5: Per-model violation rate (paper Figure 1)

![Per-model combined violation rate across baseline, safety, and adversarial conditions for Qwen2.5-Coder-32B, DeepSeek-Coder-V2-16B, and CodeLlama-34B (Q4).](paper/figures/per_model_violation_rate.pdf){width=88%}

Figure 1 plots the combined safety+security violation rate (CVR) per model across three conditions: baseline (no instructions), safety (safety guidance prepended), and adversarial (A6 prompt-injection variants). All three models cluster near the upper bound: baseline CVR sits at ≈ **98.8 %** across the model set, leaving roughly **1.2 percentage points** of headroom for any safety-prompt intervention to reduce. This **ceiling-saturation regime** is what drives the H5 null result discussed in Q8: even a hypothetically perfect safety prompt cannot move the metric far when the binary outcome is already saturated.

The most informative dimension is therefore not the headline CVR but the violation-class breakdown. SM-1 (CWE-20 input validation), SM-2 (CWE-252 unchecked return), and SM-6 (missing safety preamble) feature prominently in baseline outputs across all three models, while adversarial conditions shift mass toward SM-4 and SM-5. CodeLlama-34B differs from the two Qwen-/DeepSeek-family generations on refusal behaviour — see H8 in Q8.

### Q6: Retry-trajectory patterns (paper Figure 3)

![Four convergence patterns observed under watchdog-in-loop retries: T002 (strict two-state oscillation 15→4→15→4), T010 (monotonic convergence), T013 (improve-then-regress), T003 (invariant).](paper/figures/retry_trajectories.pdf){width=95%}

Figure 3 shows four characteristic retry-loop patterns observed in the E3 watchdog-in-loop experiment, one per panel:

- **T002 (top-left) — strict two-state oscillation** (15 → 4 → 15 → 4 rules violated, repeating). The oscillation is **deterministic** (temperature = 0), not LLM stochasticity: it is a tie-breaking artefact in the rule-rank → retry-prompt mapping. Two equivalent rule rankings flip the prompt back and forth between two attractors. This is a documented bug in the priority logic, not a model failure.
- **T010 (top-right) — monotonic convergence** to ≤ 1 violation across retries. The expected, healthy mode.
- **T013 (bottom-left) — improve-then-regress.** Two retries reduce violations, then a third introduces a regression. Indicates the LLM is fitting too closely to the most recent feedback and forgetting earlier fixes.
- **T003 (bottom-right) — invariant.** Retries do not change the violation count. Either the violation is intrinsic to the task spec or the feedback wording is not actionable to the model.

These four modes are exhaustive of the patterns observed across the 15-task suite; per-model breakdown is in paper §V.F.

### Q7: Detection latency — does the watchdog meet the proposal commitment?

The proposal committed the watchdog to a detection latency of **≤ 300 ms mean with ±15 ms 95 % CI**. The S24-D microbenchmark (paper §V.D) measured:

- **Mean: 0.648 ms**
- **95 % CI: [0.638, 0.658] ms**
- N = 1000 calls, deterministic seed, single-threaded, on PC1 hardware.

This is **463× under the latency ceiling** and ≈ **1500× under the precision target**. The headroom is large enough that future watchdog extensions (additional rules, IR pre-pass, AST visitor expansion) can add overhead by an order of magnitude and still satisfy the original commitment with a comfortable margin.

### Q8: Safety-prompt paradox and null-result framing (H5)

**H5 (preregistered):** safety prompt reduces combined VR vs baseline. **Result: null.** **Why:** baseline CVR ≈ 98.8 % across all three models (see Q5) leaves ≤ 1.2 pp of headroom — there is no statistical room for a reduction effect to manifest, regardless of intervention quality. This **ceiling-saturation** outcome is **predicted in the literature** (Qi et al. 2025; Tan et al. 2026); it is not a surprise and does not invalidate the experimental design.

**Writing strategy.** Rather than reduce the paper to its preregistered minimum form, S24-D documents the null in full with methodological transparency. Two **exploratory hypotheses** were added in §VI.K:

- **H7 — baseline-saturation ceiling effect:** when binary VR baselines exceed ≈ 95 %, intervention-vs-baseline contrasts are statistically underpowered by construction. Proposes graded-severity metrics for follow-up work.
- **H8 — near-zero refusal rate in code-specialised LLMs:** Qwen2.5-Coder, DeepSeek-Coder-V2, and CodeLlama all refuse well under 1 % of adversarial prompts in this domain, in contrast to general-purpose chat models.

Both H7 and H8 are flagged exploratory per OSF Amendment 1.

---

## Section 3 — Roadmap to NTNU Visit 1

### Q9: How was the live-execution blocker resolved?

The blocker between S24 telemetry-only state and end-to-end live execution was the External Control URCap install. Three approaches were evaluated in S24; (a) was selected and shipped in S25:

| # | Approach | Verdict |
|---|---|---|
| (a) | Custom URSim Dockerfile, URCap pre-baked into image | **Selected and shipped** (commit `095c08c`, image tag `enfield-ursim:5.12-urcap-1.0.5`). Atomic, reproducible, fits open-science release. URSim base digest plus URCap version documented separately in `docs/replication.md`. |
| (b) | Selenium / VNC PolyScope web-UI automation | Rejected. Fragile, brittle to PolyScope version changes, slow. |
| (c) | URCap runtime install API | Rejected. No public API found; PolyScope-internal only. |

A second technical decision followed once the URCap image was ready: the ROS2 driver topic-based path (`inject_mode='topic'`) does not produce motion in URSim's headless mode without an active URCap dispatcher loop. The `enfield_urscript_runtime` publisher node was therefore extended with a direct-TCP injection mode (`inject_mode='primary_tcp'`, commit `ab78dc5`) that writes the URScript program to the Secondary client port (30002), where URControl auto-runs the `def…end` block as a named program. Trailing entry-point lines in translator output are stripped on the `primary_tcp` path only, since URSim's Secondary parser rejects them as stray statements (closing `TCPReceiver` and cascading to `PROTECTIVE_STOP C154A0`). The acceptance criterion (T001 `joint_pos` range > 0) is now met; see Q4 for the joint-by-joint range table.

### Q10: S25 status and remaining lanes before 2026-06-01

S25 closed the URCap and live-execution blockers (Lane 2). Three lanes remain open before NTNU Visit 1 (2026-06-01): **Lane 3** (LLM-generated URScript live execution, sim-to-real anchor for paper §V.G), **Lane 4** (Georgios status sync + OSF Amendment 3 filing), and **Lane 5** (CycloneDX SBOM + Ollama model SHA-256 hash chain documentation).

| Lane | Task | Estimated effort | Status |
|------|------|------------------|--------|
| 2 | URCap unblock via custom Dockerfile (approach (a)) + `primary_tcp` injection path | 2–4 h | **Done** (commits `095c08c`, `ab78dc5`; T001 live-motion verified) |
| 3 | LLM-generated URScript live execution; sim-to-real anchor for paper §V.G — measure `PROTECTIVE_STOP` rate on Qwen2.5-Coder-32B baseline output for T001/T005/T010 against the hand-crafted T001 baseline (already shown to trigger `C204A3`) | 2–3 h | **Newly unblocked** (S26 priority; the `primary_tcp` path accepts LLM URScript without further code changes) |
| 4 | Status sync to Georgios (this deck + commits since S20) and OSF Amendment 3 filing (URSim direction + H7 + H8 exploratory) | 1–2 h | Pending; OSF Amendment 2 ack from Georgios still outstanding (gates Amendment 3) |
| 5 | CycloneDX SBOM generation in CI + Ollama model SHA-256 hash chain documentation | Variable | Deferrable to post-NTNU if Lane 3 over-runs |

The remaining Lane 3 plus Lane 4 budget (≈ 3–5 hours) leaves comfortable margin within S26 even if Lane 3 surfaces surprises in the LLM-output behaviour.

### Q11: NTNU Visit 1 scope confirmation

**9 weeks: 2026-06-01 → 2026-07-31. Paper-only scope.** No new infrastructure, no new experiments, no new attack types or hypotheses.

**Deliverables at NTNU:**

1. **arXiv preprint:** mid-June 2026.
2. **IEEE RA-L submission:** mid-July 2026.
3. **Replication-package finalisation:** custom URSim image (`enfield-ursim:5.12-urcap-1.0.5`, S25 Lane 2 output, already shipped), CycloneDX SBOM and Ollama model SHA-256 hash chain (Lane 5, deferrable to post-NTNU if Lane 3 over-runs), OSF data deposit linked to preregistered DOI `10.17605/OSF.IO/VE5M2`.

Anything that does not directly serve one of those three outputs is out of scope for the visit. The Lane 2 closure in S25 is what makes this contract realistic: the URCap blocker is resolved, the runtime path is end-to-end verified, and Lane 3 (sim-to-real LLM-output measurement) is the only infrastructure-adjacent work item still open before boarding the flight.

---

*Source: `docs/georgios_s25_demo.md` · Build: `bash scripts/build_demo_deck.sh` · Output: `docs/build/georgios_s25_demo.pdf`*
