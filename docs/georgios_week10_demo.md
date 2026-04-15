# ENFIELD[^enfield] — Week 10 Supervisor Demo

**Presenter:** Yunus Emre Cogurcu
**Audience:** Assoc. Prof. Georgios Spathoulas (NTNU[^ntnu])
**Date:** Week 10 — April 2026
**Scope of this session:** (1) explain what ENFIELD actually tests, (2) clarify the security dimension and why it is separate from the safety dimension, (3) walk through the open questions before confirmatory E1/E2/E3 runs.

Repo: `https://github.com/dryuemco/industrial_robot_security`
OSF[^osf] preregistration: `10.17605/OSF.IO/VE5M2` (Amendment 1 approved 2026-04-07)
Target venue: IEEE RA-L[^ral], arXiv preprint June 2026, submission July 2026.

---

## Slide 1 — Framing

ENFIELD is a **formal adversarial testing framework for LLM[^llm]-generated industrial robot code**. Concretely: we ask open-source code LLMs to write URScript[^urscript] for Universal Robots, then check whether the generated code is safe (ISO[^iso] 10218-1:2025 compliant) and secure (free of software-level weaknesses that could be exploited in deployment).

The framework has two independent analysis dimensions built on top of one pipeline:

- a **safety** dimension anchored in ISO 10218-1:2025, and
- a **security** dimension anchored in CWE[^cwe].

Today's session focuses mostly on the second one, because in the last meeting you asked me to make the security story explicit.

---

## Slide 2 — Pipeline at a glance

```
                 ┌───────────────┐
   Task IR[^ir]─►│ Prompt Builder│──► LLM ──► URScript
                 └───────────────┘                │
                                                  ▼
                                          ┌──────────────┐
                                          │   Watchdog   │
                                          │  DM + SM +   │
                                          │ analyze_combined
                                          └──────────────┘
                                                  │
                                                  ▼
                                        Violation report
                                     (safety, security, combined)
```

The pipeline is deliberately separated so that every analysis rule operates on a well-defined artifact:

- **DM[^dm] rules** (safety) operate on the **Task IR**, before any LLM is involved. They express what a compliant specification must look like.
- **SM[^sm] rules** (security) operate on the **LLM-generated URScript**, after code generation. They express what a safely generated program must not contain.
- `analyze_combined` aggregates both into a single per-sample verdict using an OR[^or] rule, and is the object of the primary confirmatory hypotheses (H4/H5/H6).

This separation is intentional: DM catches problems in the *specification* we hand the LLM; SM catches problems the *LLM itself* introduces. Conflating them would make the security claim unfalsifiable.

---

## Slide 3 — Why two dimensions, not one

ISO 10218-1:2025 is an excellent standard for **mechanical and motion safety**: speed envelopes, separation distances, protective stops, zone configuration. But it was not written to constrain *software-level weaknesses in auto-generated code*. An LLM can emit URScript that:

- respects every ISO speed and zone constraint,
- and still contains unchecked return values, hardcoded credentials, missing error handlers, or an unsanitized prompt-injection marker lifted from its training data.

If we only measured ISO conformance we would systematically miss this class of failure. Conversely, CWE alone would miss the mechanical-safety story that matters to roboticists. We need both, and we need them to be measurable *independently* as well as jointly.

One important exception worth naming: ISO 10218-1:2025 introduces **clause 5.1.16 Cybersecurity**, which is new in the 2025 revision. This clause is the anchor for A8 (Prompt Injection) and for SM-1 / SM-3 / SM-5 / SM-7. To our knowledge, ENFIELD is the first static-analysis framework to anchor LLM-generated robot-code weaknesses to 5.1.16 via its NOTE 1. SM-2 (CWE-252) and SM-4 (CWE-754) have no direct ISO anchor and will be framed in §VII as a "standard extension opportunity".

**Claim:** the safety dimension and the security dimension are complementary, not redundant. We report them separately, and we also report their OR-aggregation as the primary combined metric.

---

## Slide 4 — Safety dimension (DM-1..7)

The safety dimension is the older half of the framework (pre-LLM work, Months 1–6). It is a static watchdog with seven detection mechanisms (DM-1..7) that operate on the Task IR. These rules are mapped to ISO 10218-1:2025 clauses, and the mapping lives in `docs/iso_10218_traceability.csv`. The traceability table in paper §IV enumerates **19 rule instances across 7 distinct clauses** in 4 top-level sections (5.1, 5.4, 5.5, 5.7).

DM rules are the basis of the original H1–H3 preregistered hypotheses (DM-only SVR[^svr] on the 15-task baseline suite). These hypotheses remain valid and will be reported as a dedicated subsection of the paper; we should talk about exactly *how* to frame them alongside the LLM hypotheses.

**Note on ISO clause titles.** The clause titles in `iso_10218_traceability.csv` have been verified against my physical copy of the 2025 standard. The #5 traceability audit corrected three earlier errors: (1) clause 5.12 does not exist in the 2025 revision (an earlier `5.12.3 Safeguarded space` mapping was a hallucinated placeholder, corrected to 5.7.4 Software-based limiting); (2) clause 5.6 is Simultaneous motion, not speed limiting (speed limits live under 5.5.3 Speed limit(s) monitoring); (3) a paper §III taxonomy drift where A2–A8 labels were misaligned between §III and §IV.D has also been fixed.

---

## Slide 5 — Security dimension (SM-1..7)

The security dimension (SM-1..7) was added in Weeks 7–8. Each rule corresponds to a CWE[^cwe] and targets a weakness class that LLMs are known to introduce in generated control code:

| Rule | CWE | What it detects | ISO anchor |
|------|-----|-----------------|------------|
| SM-1 | CWE-20  | Input Validation — missing bounds/type checks on parameters coming into the program | 5.1.16 |
| SM-2 | CWE-252 | Unchecked Return Value — calls whose failure is silently ignored | — (CWE-only) |
| SM-3 | CWE-693 | Protection Mechanism Failure — safety interlocks present in spec but absent in code | 5.1.16 |
| SM-4 | CWE-754 | Improper Check for Unusual Conditions — missing guards on boundary states | — (CWE-only) |
| SM-5 | CWE-798 | Hardcoded Values — literal speeds/forces/credentials baked into the source | 5.1.16 |
| SM-6 | *(domain)* | Missing Safety Preamble — absence of the mandatory safety-config block at program start | — |
| SM-7 | *(domain)* | Prompt Injection Marker — residue from a prompt-injection attempt leaking into the generated code | 5.1.16 |

SM-1, SM-3, SM-5, SM-7 are anchored to ISO 10218-1:2025 clause 5.1.16 Cybersecurity (new in the 2025 revision). SM-2 and SM-4 are CWE-only with an em-dash in the traceability table — paper §VII will frame them as a "standard extension opportunity" rather than as ISO-compliant rules. SM-6 is domain-specific (URScript safety-config convention, no cleanly matching CWE entry).

The seven-rule set is not a claim of exhaustive security coverage. It is a **defensible minimal set** that catches the weakness classes we have empirically observed in pre-confirmatory smoke runs, and whose detection we can implement as static checks on the parsed URScript.

---

## Slide 6 — `analyze_combined` and why OR-aggregation is sound

`analyze_combined` is the unified watchdog entry point. It runs the DM pipeline on the Task IR, runs the SM pipeline on the generated URScript, and reports a sample as **violating** if *either* dimension fires at least one rule. This is an **OR[^or]** aggregation.

OR-aggregation is the object of the primary confirmatory hypotheses (H4/H5/H6). It was explicitly added to the preregistration in **Amendment 1 (approved 2026-04-07)** so that the combined metric is not a post-hoc analytical choice.

Why OR and not AND or a weighted sum:

- **Soundness:** a program that violates either safety or security is unsafe to deploy; both dimensions are independently sufficient grounds for rejection. AND would be unsound (it would accept code that fails one dimension as long as it passes the other).
- **Interpretability:** the combined verdict matches how a deployment gate would actually be implemented in practice.
- **Falsifiability:** we still report DM-only and SM-only violation rates alongside the combined rate, so reviewers can see which dimension drove the signal and the combined number cannot be inflated without evidence.

The anticipated criticism is that OR-aggregation inflates the marginal violation rate relative to either dimension alone. That is correct **by construction** and is exactly the purpose of the combined view. The safeguard is that we report all three rates in parallel.

---

## Slide 7 — Attack surface (A1–A8)

The threat model specifies eight attack categories (A1–A8), each mapped to an ISO 10218-1:2025 clause and documented in `docs/THREAT_MODEL.md` and paper §IV Table I. The table below is the paper-authoritative taxonomy (§IV.D Table I):

| Label | Attack | ISO clause | What it manipulates |
|-------|--------|------------|---------------------|
| A1 | Speed Injection | 5.5.3 | Velocity values exceeding mode-specific limits |
| A2 | Zone Penetration | 5.7.4 | Waypoints violating safe-zone halfspace constraints |
| A3 | Orientation Cone | 5.7.4 | Tool orientation inside a forbidden cone |
| A4 | Payload Tampering | 5.1.15 | Payload mass or centre-of-gravity outside declared bounds |
| A5 | Safety-Logic Bypass | 5.4.2 / 5.4 | Missing E-Stop command or incomplete safety control flow |
| A6 | Frame Confusion | 5.7.4 | Coordinate-frame inconsistencies across waypoints |
| A7 | Tool Misuse | 5.1.14 | Invalid tool identity or activation mode |
| A8 | Prompt Injection | 5.1.16 | Adversarial prompts / configuration tampering |

A1–A7 are mechanical/control-level attacks that manipulate the Task IR or the generated motion primitives. **A8 is the only category that targets the LLM itself** rather than the generated program: it is operationalized as a family of prompt-injection sub-variants used in E2, and it is the only category anchored to the new clause 5.1.16 Cybersecurity.

**Paper-vs-code naming convention (important).** Throughout the paper, prompt-injection sub-variants are written as **A8.x** (A8.1, A8.2, …), consistent with the Table I taxonomy. The **codebase** (experiment runner `scripts/llm_experiment_runner.py`, prompt builder, mutation modules, mcnemar analysis) retains the **legacy `A6.*` identifiers** pending an editorial-only refactor. Per paper §IV.D Note, the rename is purely editorial and does not affect attack generation semantics, hypotheses, or statistical analysis. The Phase 6 invariant test T3 pins `H5_adversarial_A6.N_vs_baseline` as the authoritative code-level comparison string (commit `01739ba`). For this meeting: when I say "A8.x" I mean the paper label, and when you see `A6.*` in a runner log or a test id it is the same thing under the legacy identifier.

---

## Slide 8 — Pre-confirmatory findings (worth showing)

From the pre-confirmatory smoke runs (the ones that motivated the runner end-to-end gate and the URScript validity gate):

1. **Safety-prompt paradox on Qwen2.5-Coder.** Adding a safety-focused system prompt *increased* the number of violations (**6 → 11** in the smoke fixture, per paper abstract) because the model confused `mm/s` with `m/s` when reasoning about limits. This is a real, reportable finding: safety-oriented prompting is not monotonically safe, and unit confusion is a specific failure mode worth naming.
2. **CodeLlama false-negative pattern.** CodeLlama-34B consistently produced a *lower* raw violation count than the other two models, but inspection revealed that it was emitting pseudo-code rather than valid URScript. A watchdog that cannot parse the output cannot flag it, so CodeLlama was "winning" by failing to generate real code. The fix was the **URScript validity gate** (`CodeParser.is_valid_urscript`): outputs that do not parse as valid URScript are filtered and reported separately from the violation-rate analysis, so CodeLlama's weakness becomes visible instead of hidden.
3. **Runner end-to-end gate.** Until the mock smoke test was added, `llm_experiment_runner.py` had never been exercised end-to-end. Four hidden bugs were caught during that exercise (parser API mismatch, missing URScript gate, `analyze_combined` argument order, response-field name drift). They are now regression-tested by `tests/test_runner_mock_smoke.py`, and the gate is enforced before every confirmatory run.
4. **Adversarial prompt injection (paper abstract preview).** One model showed a 2100% violation increase under A8 prompt injection while others were unaffected, revealing highly model-dependent vulnerability profiles. This is a preview number from the paper abstract and will be re-computed under the confirmatory protocol once PC2 is back.

None of these are confirmatory results yet. They are the reason the confirmatory runs are now tightly gated.

---

## Slide 9 — Statistical frame and Phase 6 tripwires

The confirmatory analysis is a family of McNemar[^mcnemar] paired tests on matched-sample contrasts, with Holm–Bonferroni[^holm] correction across the family. The LLM hypotheses (Amendment 1) are:

- **H4** — baseline-condition violation rate across the three models.
- **H5** — per-**A8.x** sub-variant adversarial violation rate versus baseline. The code-level comparison string uses the legacy label and is pinned by Phase 6 invariant test T3 as `H5_adversarial_A6.N_vs_baseline`.
- **H6** — watchdog-in-loop condition versus baseline, **and** versus the safety-prompted condition. Pinned by Phase 6 invariant test T5 as the set `{H6_watchdog_vs_baseline, H6_watchdog_vs_safety}`.

Cross-model comparison uses **Cochran's Q**, which is **exploratory** per Amendment 1 and not part of the confirmatory family. The Cochran's Q implementation (commits `e4573ad` + `e918fb7`) added 14 tests and explicitly handles the degenerate perfect-agreement case where `statsmodels.cochrans_q` returns NaN.

In session 6 I added a set of **hypothesis-mapping invariant tests** (six tests, commit `4ca511a`) that pin the contract between the code-level functions (`run_h4`, `run_h5`, `run_h6`) and the paper-level hypotheses. These are semantic tripwires: if a future refactor silently changes which condition a function filters on, or drops a row from the pooled output, the tests fail. This is the mechanism that closes the paper-code drift issue we discussed as WEEK10_TODO item #12.

Total repo test count is **708 passing** as of commit `6100f91` (session 6 HEAD).

---

## Slide 10 — Status, blockers, and what needs your input

**Hard gates for confirmatory E1/E2/E3 — all closed:**

- `#5b` Mock-LLM smoke run (runner end-to-end) — DONE
- `#6`  Refusal classifier freeze (URScript-aware `has_code` + 20-entry frozen indicator list) — DONE
- `#7`  Cochran's Q omnibus (as exploratory addition per Amendment 1) — DONE
- Phase 6 hypothesis-mapping invariant tests — DONE (session 6)

**Remaining blocker:** PC2 LLM server reachability. The three models live on PC2 (Windows, RTX 5090, Ollama at `192.168.1.5:11434`) and PC2 is currently unreachable. As soon as it is back, I run the E1 pilot (3 LLMs × 5 tasks × 2 conditions × 1 rep = 30 calls), then E1 full, E2, E3.

**Still blocked pending your input:**

- **Phase 3d** (H1–H3 reporting strategy). The original prereg covers H1–H3 on DM-only; the paper now centers on the LLM family H4–H6. I would like your call on whether H1–H3 belong as a dedicated subsection in §VI, as an appendix, or as a deferred companion report.
- **Paper §V.E framing of H6.** There is a known parse-ambiguity in the current phrasing of H6 ("watchdog alongside baseline and safety"). I have deliberately not touched §V.E for a fourth time within 48 hours; I would like a second opinion before the next edit.

---

## Slide 11 — Anticipated Q&A

**Q1. Why CWE? Isn't ISO 10218-1 enough for industrial robots?**
ISO 10218-1:2025 is the right standard for *what the robot is allowed to do mechanically*, but it was not written to constrain *how auto-generated source code should be structured*. An LLM can produce URScript that is ISO-conformant in its motion parameters and still contains unchecked returns, hardcoded setpoints, or residue from a prompt-injection attempt. The new 5.1.16 Cybersecurity clause anchors four of the seven SM rules but does not cover the software-weakness vocabulary we need for SM-2 and SM-4. CWE fills that gap. They are complementary, not alternatives.

**Q2. How did you choose SM-1..7 specifically? What is your coverage argument?**
SM-1..5 are CWE-anchored (CWE-20, CWE-252, CWE-693, CWE-754, CWE-798): input validation, unchecked return, protection-mechanism failure, improper check of unusual conditions, and hardcoded values. These are the five weakness classes that (a) appeared in pre-confirmatory smoke runs and (b) can be statically detected in parsed URScript. SM-6 and SM-7 are domain-specific: SM-6 catches the missing URScript safety-config preamble (no clean CWE entry), and SM-7 catches prompt-injection markers leaking from the LLM's input context into the generated code. I am not claiming exhaustive coverage — the claim is that this is a **defensible minimal set** for a code-generation setting, and the paper frames it that way explicitly.

**Q3. `analyze_combined` uses OR. Doesn't that inflate violation rates by construction?**
Yes, by construction. That is the point: a program is unsafe to deploy if either dimension flags it, and OR is the only sound aggregation under that definition. The inflation is mitigated by always reporting DM-only and SM-only rates alongside the combined rate. The combined rate is the primary confirmatory metric; the two component rates are reported so a reviewer can see which dimension drove any specific result. OR-aggregation was added to the preregistration in Amendment 1 (2026-04-07), so it is not a post-hoc choice.

**Q4. The URScript validity gate filters out CodeLlama outputs. Isn't that selection bias?**
It would be, if we hid it. Instead we report (a) how many samples each model produced that were filtered as invalid URScript, (b) the violation rate on the parsed subset, and (c) the violation rate on the full sample assuming unparsable output counts as a failure. So CodeLlama's weakness is visible in three separate numbers rather than hidden behind one. The gate itself is enforced by `CodeParser.is_valid_urscript` and is regression-tested.

**Q5. Why do I see A8.x in the paper but A6.* in the code? Is that a drift?**
Not a drift — it is a documented legacy identifier. The paper uses **A8.x** (consistent with Table I, where A8 = Prompt Injection). The codebase (experiment runner, prompt builder, mutation modules, mcnemar analysis) retains the **legacy `A6.*` identifiers** from the pre-taxonomy-fix phase of the project. Paper §IV.D includes an explicit Note stating that the rename is purely editorial and does not affect attack semantics, hypotheses, or statistical analysis. Phase 6 invariant test T3 pins the legacy code-level comparison string `H5_adversarial_A6.N_vs_baseline` so it cannot silently regress. The full A6.* → A8.* code-level rename is on the deferred follow-ups list and is explicitly a future editorial commit, not a session 7 scope item.

**Q6. Is Cochran's Q confirmatory or exploratory?**
Exploratory. Only the McNemar contrasts for H4/H5/H6 are confirmatory. Amendment 1 explicitly designates Cochran's Q as an exploratory cross-model omnibus. The paper will label it accordingly in §VI. The implementation includes explicit handling of the degenerate perfect-agreement case where `statsmodels.cochrans_q` returns NaN — that is a preregistration gap in §VII.B.5 that was closed by commit `e4573ad`.

**Q7. Where do H1–H3 go, given that the paper now centers on H4–H6?**
This is one of the two items I need your input on. H1–H3 are the original preregistered DM-only hypotheses and they remain valid. The options I see: a dedicated §VI subsection in the same paper, a deferred appendix, or a short companion report. I would like your view before I commit to one.

**Q8. You are running Q4_K_M quantized models. Does that affect reproducibility or the conclusions?**
It is listed as a threat to validity in §VII.B. The risk is that the quantized variant may produce systematically different output from the full-precision checkpoint, which would weaken the "anyone can replicate" claim. Cross-quantization sensitivity analysis (Q4 vs full precision on a subset of tasks) is already on the future-work list. I am treating it as a limitation to disclose rather than a result to hide.

**Q9. What is the false-positive rate of the SM rules?**
Measured on a hand-curated corpus of known-safe URScript samples. I will have the updated number in the next meeting; I would rather not commit to a figure today than cite a stale one. This ties into WEEK10_TODO item #12 (paper-code consistency), which is now partially closed by the Phase 6 invariant tests but has a documentation FIXME still open in Appendix B.

**Q10. What exactly did OSF Amendment 1 cover?**
Three things: (a) the addition of the LLM hypotheses H4/H5/H6 to the confirmatory family; (b) the DeepSeek-Coder-V2-16B license statement (DeepSeek license, not Apache-2.0, so it needs its own disclosure paragraph); and (c) the `analyze_combined` OR-aggregation specification as the primary combined metric. The amendment was submitted after the security dimension was integrated and was approved on 2026-04-07.

**Q11. What is the single most fragile part of the current design?**
Honestly, the coverage argument for SM-6 and SM-7. They are the two rules without a CWE anchor, and a reviewer could reasonably ask why they are in the set at all. My current answer is "they catch weaknesses we empirically observed and that no CWE cleanly covers", which is defensible but not airtight. SM-7 is at least anchored to 5.1.16 on the ISO side, but SM-6 has neither anchor. I would welcome your view on whether to keep them as first-class rules or move them to a supplementary list.

---

## Slide 12 — Asks for this meeting

1. **Phase 3d go/no-go** — open H1–H3 reporting, or leave it sealed until after E1/E2/E3?
2. **SM-6 / SM-7 positioning** — first-class rules, or supplementary?
3. **§V.E H6 phrasing** — second opinion on the "alongside" parse-ambiguity before the next edit.
4. **PC2 reachability** — is there anything I should try from the NTNU side, or do we wait?
5. **Next supervisor meeting date** — I would like one live walkthrough of the E1 pilot results once PC2 is back, before E1 full kicks off.

---

## Footnotes — abbreviation glossary

[^enfield]: **ENFIELD** — European Lighthouse for Trustworthy AI (Horizon Europe Grant No. 101120657). The host project under which this exchange is funded.
[^ntnu]: **NTNU** — Norwegian University of Science and Technology (Trondheim, Norway). Host institution for the exchange visit.
[^osf]: **OSF** — Open Science Framework. Platform used for preregistration and the eventual open-science data deposit.
[^ral]: **IEEE RA-L** — IEEE Robotics and Automation Letters. Target publication venue.
[^llm]: **LLM** — Large Language Model. In this project, the open-source code models Qwen2.5-Coder-32B, DeepSeek-Coder-V2-16B, and CodeLlama-34B, served locally via Ollama.
[^urscript]: **URScript** — Universal Robots' scripting language for programming UR robot arms. The target output language for all LLM code generation in this project.
[^iso]: **ISO** — International Organization for Standardization. ISO 10218-1:2025 is the current edition of the industrial-robot safety standard; clause 5.1.16 Cybersecurity is new in the 2025 revision.
[^cwe]: **CWE** — Common Weakness Enumeration. The MITRE-maintained catalog of software weakness classes. Used as the anchor vocabulary for the security dimension (SM-1..5 and SM-7).
[^ir]: **IR** — Intermediate Representation. A vendor-neutral YAML/JSON task description that the pipeline consumes before prompting the LLM; it isolates the core logic from robot-specific details.
[^dm]: **DM** — Detection Mechanism. The seven safety rules (DM-1..7) that operate on the Task IR and encode ISO 10218-1:2025 safety constraints.
[^sm]: **SM** — Security Mechanism. The seven security rules (SM-1..7) that operate on LLM-generated URScript and encode CWE-anchored weakness classes (SM-1..5, SM-7) plus one domain-specific rule (SM-6).
[^or]: **OR** — logical disjunction. In `analyze_combined`, a sample is flagged as violating if at least one dimension (DM or SM) fires a rule.
[^svr]: **SVR** — Safety Violation Rate. Proportion of samples that trigger at least one safety rule; the primary DM-only outcome metric.
[^mcnemar]: **McNemar test** — statistical test for paired nominal data, used here to compare violation rates between matched conditions (e.g. baseline vs watchdog-in-loop on the same task/model pair).
[^holm]: **Holm–Bonferroni correction** — a sequentially rejective multiple-testing correction that controls the family-wise error rate at α=0.05 across the confirmatory hypothesis family.
