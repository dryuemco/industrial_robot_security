---
title: "ENFIELD S25 Pre-NTNU Status"
subtitle: "Demo Deck for Georgios"
author: "Yunus Emre Cogurcu (PhD, Çukurova Üniversitesi)"
date: "May 2026"
geometry: margin=1in
fontsize: 11pt
documentclass: article
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

<!-- TODO[content]: S24 closed at HEAD 85a0530; S25 pre-NTNU sprint in progress;
     NTNU Visit 1 paper-only scope; arXiv preprint June 2026; IEEE RA-L submission July 2026. -->

### Q2: What did S24 deliver?

<!-- TODO[content]: 9 commits across sub-lanes A (URScript runtime, telemetry-only),
     B (figure suite F1/F2/F3), D (paper §V.D detection latency + §IV.C vendor language audit),
     plus 1 TODO close commit. Sub-lane C (this deck) deferred from S24 to S25 Lane 1. -->

### Q3: System architecture overview

![ENFIELD end-to-end architecture: prompt → LLM (Ollama backends) → URScript translator → static watchdog → URSim runtime telemetry.](paper/figures/architecture.pdf){width=92%}

<!-- TODO[content]: 90s walkthrough — 3 layers (LLM eval, static analysis, runtime),
     dual-PC topology (PC1 dev + PC2 Ollama 192.168.1.5:11434), 3 models. -->

### Q4: URSim runtime — what works today, what is blocked?

<!-- TODO[content]: enfield_urscript_runtime package: publisher node + telemetry recorder
     + T001 smoke launch + 8 offline tests passing. Blocked: motion execution requires
     External Control URCap, which is NOT pre-installed in vanilla URSim e-Series 5.12 image
     and is NOT scriptable via Docker volume mounts (PolyScope web UI required).
     S25 Lane 2 will resolve this locally before NTNU. -->

---

## Section 2 — Empirical Findings

### Q5: Per-model violation rate (paper Figure 1)

![Per-model combined violation rate across baseline, safety, and adversarial conditions for Qwen2.5-Coder-32B, DeepSeek-Coder-V2-16B, and CodeLlama-34B (Q4).](paper/figures/per_model_violation_rate.pdf){width=88%}

<!-- TODO[content]: key numbers — baseline ceiling near 98.8% combined VR (H5 saturation),
     ranking order, where each model loses (security vs safety dimension breakdown). -->

### Q6: Retry-trajectory patterns (paper Figure 3)

![Four convergence patterns observed under watchdog-in-loop retries: T002 (strict two-state oscillation 15→4→15→4), T010 (monotonic convergence), T013 (improve-then-regress), T003 (invariant).](paper/figures/retry_trajectories.pdf){width=95%}

<!-- TODO[content]: brief description of each panel; emphasize that oscillation
     (T002) is a *deterministic* tie-breaking artefact, not LLM stochasticity (temperature=0). -->

### Q7: Detection latency — does the watchdog meet the proposal commitment?

<!-- TODO[content]: microbenchmark result: mean 0.648 ms, 95% CI [0.638, 0.658] ms.
     Proposal commitment was ≤300 ms (and ±15 ms precision). Achieved 463× under the
     latency ceiling and 1500× under the precision target. Paper §V.D anchors this. -->

### Q8: Safety-prompt paradox and null-result framing (H5)

<!-- TODO[content]: H5 null result was *predicted* by literature (Qi 2025, Tan 2026)
     and is caused by binary ceiling saturation (98.8% baseline leaves 1.2 pp room).
     Writing strategy: full disclosure + methodological transparency, not minimal
     preregistered-only form. H7 (saturation ceiling) and H8 (near-zero refusal in
     code-specialized LLMs) added as exploratory hypotheses (paper §VI.K). -->

---

## Section 3 — Roadmap to NTNU Visit 1

### Q9: What is blocking live URSim execution, and how do we unblock it?

<!-- TODO[content]: URCap install is the single blocker. Three options evaluated:
     (a) custom URSim Dockerfile with URCap pre-baked — preferred, atomic, but loses
     upstream digest pin; (b) Selenium/VNC PolyScope automation — fragile; (c) runtime
     install API — none found. S25 Lane 2 commits to (a). -->

### Q10: S25 remaining lanes before 2026-06-01

<!-- TODO[content]: Lane 2 — URCap unblock via custom Dockerfile (~2–4 h);
     Lane 3 — LLM-generated URScript live execution post-unblock, sim-to-real
     anchor for paper §V.G (~2 h, stretch); Lane 4 — Georgios status sync +
     OSF Amendment 3 file (URSim direction + H7/H8 exploratory) (~1–2 h);
     Lane 5 — CycloneDX SBOM + Ollama model SHA-256 hash chain documentation
     (deferrable to post-NTNU if time-pressed). -->

### Q11: NTNU Visit 1 scope confirmation

<!-- TODO[content]: 9 weeks (Jun 1 – Jul 31), paper-only.
     Deliverables: (i) arXiv preprint mid-June; (ii) IEEE RA-L submission mid-July;
     (iii) replication-package finalization. NO new infrastructure or experiments at NTNU. -->

---

*Source: `docs/georgios_s25_demo.md` · Build: `bash scripts/build_demo_deck.sh` · Output: `docs/build/georgios_s25_demo.pdf`*
