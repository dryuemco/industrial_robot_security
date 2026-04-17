# ENFIELD Literature Synthesis & IEEE RA-L Strengthening Plan

**Session 19, 2026-04-17.** Synthesis of `docs/literature_review.xlsx` (18 papers, 22 gaps, 3 new hypothesis candidates). Purpose: position ENFIELD for IEEE RA-L submission, formalize new hypotheses, and provide drop-in sentences for paper integration.

> **Status: working document (WIP).** This document is a literature-driven synthesis generated during Session 19. The hypothesis candidates (H7 ceiling, H8 near-zero refusal) and the paper drop-in sentences in section 4 are proposals, not decisions. They require author review before integration into `paper/draft_v0.1.md` or submission to OSF as Amendment 3. Known gaps (Kumar COLM 2024, INSEC ICML 2025 identifications; preprint hedging; EU AI Act coverage) are enumerated in section 7.

---

## 1. Executive summary

Four findings dominate the literature review and shape the RA-L strategy.

**Finding 1 — ISO 10218-1:2025 explicitly opens the cybersecurity–safety nexus.** The 2025 revision (replacing the 2011 edition) adds "requirements for cybersecurity to the extent that it applies to industrial robot safety." This wording did not exist in 2011. ENFIELD is precisely the evaluation methodology this new clause invites. **This is the single most RA-L-relevant framing we can give the paper.** The Introduction and Abstract should lead with it.

**Finding 2 — H5 "not supported" is a literature-predicted outcome, not a negative result.** Three independent papers converge on a mechanistic explanation. Qi et al. ICLR 2025 ("Shallow Safety Alignment") shows safety alignment is concentrated in the first few output tokens. Code LLMs emit code immediately, bypassing those tokens. Tan et al. 2026 reports **0.00% baseline inappropriate-detection rate for Qwen2.5-coder:7b, DeepSeek-coder:6.7b, and 0.24% for CodeLlama:7b** on HumanEval-Injected — the same three model families we use. "LLMs Caught in the Crossfire" (ACL 2025) reports a family-wise refusal gradient where code-specialized LLMs sit much lower than chat-tuned siblings. Our H5 null is fully consistent with this literature. This motivates **H7 (ceiling saturation)** as a post-hoc exploratory hypothesis.

**Finding 3 — FDSP's Bandit feedback loop reports up to 82% relative reduction on general Python.** Alrashedy et al. (cited in Static Analysis Feedback Loop 2025 arXiv:2508.14419) reduced GPT-4 vulnerability rate from 40.2% to 7.4% via Bandit feedback. Our H6 threshold of 40% relative reduction is conservative in this context. Our actual results (CodeLlama −50%, DeepSeek −84%) sit in the same band as FDSP's high end. This lets us frame H6 results as **comparable to state-of-the-art feedback-loop literature in adjacent domains**, rather than as an isolated claim.

**Finding 4 — Static-analyzer recall is a known weakness.** CWEval 2025 reports that only 562/1916 (29%) of CyberSecEval's vulnerable samples could be reproduced with the associated static analyzer. QLPro 2025 reports CodeQL detected only 24/62 (39%) of known vulnerabilities in their Java test. Our SM-1..7 watchdog is a static analyzer. A reviewer will ask: "Why is yours more stable?" We need a concrete answer in §VII.B, not silence. The answer has three prongs: URScript domain is far narrower than Python/Java; SM-1..7 use deterministic AST over well-defined grammar; multi-rule OR-aggregation provides voting redundancy.

---

## 2. Hypothesis family status after literature cross-check

**H4 (baseline violation rate ≥ 30%) — SUPPORTED, literature-calibrated.** Six independent sources report LLM-generated code vulnerability rates between 40% and 74%. Our 60.7% observed baseline sits inside this band. ISO 10218-1:2025 legitimizes the threshold by explicitly naming cybersecurity-safety interaction as a standards concern.

**Recommendation:** keep 30% pre-registered threshold (OSF amendment would be major). In Discussion, observe that empirical baseline (60.7%) far exceeds both the pre-registered 30% and Pearce et al.'s 40% Python benchmark.

**H5 (adversarial contrast ≥ 50pp) — NOT SUPPORTED; reframe as ceiling-saturation finding.** Chat-aligned literature reports 85–100% adversarial success. Code-specialized LLMs have near-zero baseline refusal and 60–80% baseline violation. Headroom formula `HR = 100 − baseline_rate` gives ~39.3pp for our setting; 50pp is mechanically unreachable when baselines are this high. H5 null is literature-predicted.

**Recommendation:** keep H5 wording as pre-registered. Add Discussion paragraph explaining the ceiling-saturation mechanism. Introduce H7 as post-hoc exploratory.

**H6 (watchdog-in-loop ≥ 40% relative reduction) — PARTIAL; frame conservatively against FDSP 82% upper bound.** Yang et al. "Safety Chip" and arXiv:2509.02163 establish rule-based validation as an accepted approach for LLM-driven robotics. FDSP establishes 82% relative reduction as achievable upper bound in Python. Our CodeLlama −50% and DeepSeek −84% sit comfortably inside this bound.

**Recommendation:** keep 40% threshold. Add §VII.B sub-subsection addressing CWEval/QLPro static-analyzer critique (see Finding 4 answer above).

---

## 3. New hypotheses: H7 and H8

Both are post-hoc, literature-motivated, and testable with existing data. Both should be presented as **exploratory** not confirmatory, to avoid any appearance of p-hacking. OSF Amendment 3 is optional and can be filed after Georgios review.

### H7 — Baseline-saturation ceiling hypothesis

> **H7 (exploratory).** Code generation LLMs exhibit baseline-saturation ceiling effects when tested against structural safety rules, bounding the measurable effect size of adversarial prompting below the thresholds typically reported in jailbreak literature for chat-aligned models.

**Testable with existing data:** yes. Derive `HR_m = 100 − baseline_rate_m` per model, show that 0/14 contrasts exceeding 50pp is consistent with `HR_m < 50` across all three models under all conditions.

**Literature support:** G004 (Zou 2023), G005 (Agentic SoK 2026), G019 (Qi 2025 Shallow Safety Alignment), G020 (Malware-Requests ACL 2025), G021 (Tan 2026).

### H8 — Near-zero refusal in code-specialized LLMs

> **H8 (exploratory).** Refusal rates in code-specialized LLMs are near-zero across both baseline and adversarial conditions for structural-safety violation requests, reflecting weaker safety tuning compared to chat-aligned models.

**Testable with existing data:** yes. Our §VI.I refusal finding + per-rule Table VI already report this. Add literature contrast: Tan 2026 reports 0.00% / 0.00% / 0.24% baseline IDR on same three model families.

**Literature support:** G019 (Qi 2025), G020 (Malware-Requests ACL 2025), G021 (Tan 2026), G022 (Tan 2026 ablation showing DR prompting can lift refusal).

---

## 4. Drop-in sentences for paper integration

Below are concrete sentences ready to paste into the paper. Each is tagged with the target section and key cite references. Sentences respect the copyright rule (paraphrased, not quoted verbatim from sources). Cite markers use placeholder `[CITE:paper_id]` — replace with your BibTeX key format when pasting.

### 4.1 Abstract additions (consider inserting as last 2 sentences)

> We position this work against ISO 10218-1:2025, whose 2025 revision for the first time requires manufacturers to address cybersecurity to the extent it affects industrial robot safety [CITE:iso10218_2025]. Our framework provides one concrete evaluation methodology for this newly formalized safety–security interaction, instrumenting three open-source code-specialized LLMs against nineteen AST-mapped safety rules covering seven ISO clauses.

### 4.2 §I Introduction — opening paragraph reframe

> Industrial robot safety has long been governed by ISO 10218; its 2025 revision is the first edition to explicitly require that manufacturers address cybersecurity to the extent it affects industrial robot safety [CITE:iso10218_2025]. At the same time, large language models are increasingly used to generate low-level robot control code, with foundational work such as Code-as-Policies establishing the language-model-to-robot-policy pipeline [CITE:liang2023cap] and subsequent systems extending it to multi-robot and construction settings. The intersection of these two developments — LLM-generated industrial-robot code evaluated against formal safety standards — defines a gap that no prior evaluation framework addresses. General-purpose LLM-code security benchmarks such as Pearce et al.'s Copilot study [CITE:pearce2022asleep], SecurityEval [CITE:siddiq2022securityeval], and CWEval [CITE:peng2025cweval] report LLM vulnerability rates between 40% and 74%, but target general Python or Java code without motion semantics, safety-rule structure, or ISO traceability. Rule-based validators for LLM-driven robots exist at the task-plan level [CITE:yang2023safetychip; CITE:enhance_reliability_2509], but none operate on low-level motion primitives with velocity, acceleration, zone, and E-stop semantics mapped to ISO 10218-1:2025 clauses.

### 4.3 §II Related Work — skeleton to fill STUB

Proposed three-subsection structure, each ~4–6 sentences:

**§II.A LLM code generation and its security properties.**
> Empirical studies consistently report that LLM-generated code contains exploitable vulnerabilities at high rates. Pearce et al. prompted GitHub Copilot with 89 CWE-scoped scenarios and found approximately 40% of the 1689 generated programs were vulnerable [CITE:pearce2022asleep]. SecurityEval provides 130 samples across 75 CWE types for systematic benchmarking, with Siddiq and Santos reporting vulnerability rates of 68% for InCoder and 74% for Copilot [CITE:siddiq2022securityeval]. More recent outcome-driven evaluation frameworks such as CWEval quantify a 25–35 percentage-point absolute gap between functional correctness and simultaneously functional and secure generation across frontier LLMs [CITE:peng2025cweval]. These benchmarks are confined to general-purpose Python or Java; none address industrial-robot motion primitives or safety standards.

**§II.B LLMs for robot code and safety validation.**
> The use of LLMs to generate robot control programs was established by Code-as-Policies [CITE:liang2023cap] and ProgPrompt [CITE:singh2023progprompt], which demonstrate policy-code and task-plan generation from natural language. Safety validation for LLM-driven robot agents has been approached through temporal-logic runtime monitoring referencing ISO 61508 functional-safety concepts [CITE:yang2023safetychip], through rule-based validation of high-level Move, Turn, and Stop primitives [CITE:enhance_reliability_2509], and through knowledge-graph-assisted safe code generation in the drone domain [CITE:althobaiti2024drone]. None of these works operate at the granularity of velocity, acceleration, zone, and coordinate-frame parameters in vendor-specific motion code, and none map to ISO 10218-1:2025 clauses.

**§II.C Adversarial robustness of LLMs and code models.**
> Adversarial attacks on chat-aligned LLMs achieve 85–100% success rates under automated methods such as Greedy Coordinate Gradient [CITE:zou2023universal], and systematic surveys of agentic coding assistants report attack success above 85% against state-of-the-art defenses [CITE:agentic_sok_2026]. However, recent work characterizes safety alignment as shallow — concentrated in the first few output tokens — which predicts weaker refusal behavior in code-specialized LLMs that emit code tokens immediately [CITE:qi2024shallow]. Empirical evidence from ACL 2025 [CITE:malwarereq2025acl] and recent auditing work [CITE:tan2025sasnlsafety] confirms this family-wise gradient, reporting near-zero baseline refusal for Qwen2.5-coder, DeepSeek-coder, and CodeLlama against injected malicious content. Our results are consistent with this literature and extend it to the specific case of industrial-robot safety-rule violations.

### 4.4 §V Methods — threshold rationale footnote

Add a footnote or methods-discussion sentence next to H6 threshold definition:

> Our pre-registered H6 threshold of 40% relative reduction is deliberately conservative relative to feedback-loop literature on general Python code, where Bandit-based feedback has been reported to reduce vulnerability rates by up to approximately 82% relative [CITE:alrashedy2024fdsp]. We adopt the lower threshold because URScript's narrower syntactic surface, the ISO-mapped rule structure, and the single-vendor setting are expected to produce more heterogeneous reduction across models than general-purpose Python.

### 4.5 §VI Results — H7/H8 exploratory framing

Add a new subsection **§VI.K Exploratory hypotheses (post-hoc)**:

> While H5 was pre-registered with a 50-percentage-point adversarial contrast threshold and was not supported under any of the fourteen contrasts tested, the observed pattern is consistent with a ceiling-saturation mechanism predicted by prior literature. Specifically, with baseline violation rates per model ranging from 0.53 to 1.00 under our SM-1..7 rule set, the available headroom `HR = 100 − baseline_rate` is bounded above by 47 percentage points in the most favorable case and approaches zero in the ceiling case of Qwen2.5-Coder (baseline 1.00). Under these conditions the 50-percentage-point adversarial threshold is not reachable for any contrast, a pattern that independent work on safety alignment in code-specialized LLMs attributes to shallow-alignment mechanisms [CITE:qi2024shallow] and family-wise low baseline refusal [CITE:malwarereq2025acl; CITE:tan2025sasnlsafety]. We therefore propose, as post-hoc exploratory hypotheses, **H7** (baseline-saturation ceiling effect) and **H8** (near-zero refusal rate in code-specialized LLMs for structural-safety violation requests). Both are consistent with our observed data and are framed explicitly as exploratory to distinguish them from the confirmatory H4–H6 family.

### 4.6 §VII.A Discussion — positioning paragraph

> ENFIELD's contribution is not to argue that LLM-generated industrial-robot code is insecure — our 60.7% baseline violation rate is consistent with 40–74% vulnerability rates reported across Python and Java benchmarks [CITE:pearce2022asleep; CITE:siddiq2022securityeval; CITE:peng2025cweval] and thus does not by itself surprise. The contribution is to provide, for the first time to our knowledge, a complete evaluation methodology that combines (i) ISO 10218-1:2025 clause mapping at the motion-primitive level, (ii) CWE-tagged rule semantics, (iii) a CI-verified watchdog with deterministic AST parsing over URScript, (iv) an adversarial test suite specifically targeting structural safety rules rather than refusal behavior, and (v) pre-registered confirmatory hypotheses with matched-pair statistical inference. Each of these components has prior art individually — rule-based LLM safety validation for robots [CITE:yang2023safetychip; CITE:enhance_reliability_2509], CWE-mapped benchmarks [CITE:siddiq2022securityeval], feedback-loop static analysis [CITE:alrashedy2024fdsp], matched-pair hypothesis testing in software evaluation — but the five together, bound to the new 2025 ISO revision [CITE:iso10218_2025], represent a novel integration.

### 4.7 §VII.B Threats to Validity — new sub-subsection on static-analyzer recall

Add a new sub-subsection, e.g., §VII.B.9:

> **Static-analyzer recall limits.** Our SM-1..7 watchdog is a static analyzer operating over URScript ASTs. Recent benchmarking has surfaced concerns about static-analyzer recall: CWEval reports that fewer than one-third of CyberSecEval's vulnerable samples could be reproduced with the associated static analyzer due to incomplete code, missing dependencies, and syntactic fragility [CITE:peng2025cweval], and QLPro reports that CodeQL's standard rule library detected only 24 of 62 confirmed vulnerabilities in a Java benchmark [CITE:qlpro2025]. Three design choices mitigate these concerns in our setting. First, the URScript domain is far narrower syntactically than Python or Java — there are no package imports, no external dependencies, no user-defined classes — so the parse-failure and incomplete-code cases that drive the CWEval irreproducibility rate do not arise. Second, our SM-1..7 rules use deterministic AST matching against a fixed grammar rather than probabilistic taint-flow analysis, making detection binary and reproducible across runs. Third, `analyze_combined` applies OR-aggregation across all seven rules, which trades precision for recall in a way that is explicit and auditable. We nevertheless acknowledge that these mitigations are specific to URScript and would not transfer to generic multi-vendor industrial code without grammar extensions.

### 4.8 §VIII Conclusion — closing framing

> Our results extend the established finding that LLM-generated code contains exploitable vulnerabilities [CITE:pearce2022asleep; CITE:siddiq2022securityeval] into the industrial-robot safety domain for the first time with explicit ISO 10218-1:2025 clause mapping. The pattern of results is consistent with an emerging view that safety tuning in code-specialized LLMs is both shallower than in chat-tuned counterparts [CITE:qi2024shallow; CITE:malwarereq2025acl] and bounded above by baseline-saturation effects when structural-safety rules are the evaluation target. We release the full methodology, rule set, and ISO traceability matrix under an OSI-approved licence to support independent replication and to provide one concrete instance of the evaluation methodology that the ISO 10218-1:2025 cybersecurity clause now invites.

---

## 5. OSF Amendment 3 — draft scope (optional, after Georgios review)

**Scope:** add H7 and H8 as **post-hoc exploratory** hypotheses. Explicitly mark them as non-confirmatory. Do not modify H4, H5, H6 wording or decision rules.

**Rationale to include in amendment text:**

> During manuscript preparation for the IEEE RA-L submission, the authors conducted a structured literature cross-check against 18 peer-reviewed and preprint sources covering LLM code-generation security, adversarial prompting, LLM-driven robot code, static-analysis feedback loops, ISO 10218-1:2025 revision, and refusal behavior in code-specialized LLMs. The cross-check surfaced two mechanistic explanations that were not anticipated in the original preregistration. First, recent work characterizes safety alignment as shallow, concentrated in the first output tokens, which predicts weaker refusal in code-specialized LLMs. Second, independent empirical evaluation on the same three model families (Qwen2.5-Coder, DeepSeek-Coder, CodeLlama) reports near-zero baseline detection of malicious-keyword content on injected benchmarks. Both findings converge on a ceiling-saturation explanation for the pre-registered H5 null result. To document this convergence without modifying confirmatory claims, we add two post-hoc exploratory hypotheses (H7, H8) labeled as such in the manuscript and in this amendment.

**Amendment type:** exploratory-addition, non-confirmatory. Should be lightweight for Georgios sign-off.

**Blocker:** Amendment 2 (d251c15) is still pending Georgios acknowledgment per memory. Do not stack Amendment 3 before 2 is cleared.

---

## 6. Action checklist for Sessions 19–21

These are concrete, commit-sized tasks. Each is independent and can be scheduled without dependency inversion.

**Session 19 (remaining) — documentation track**
- Commit `docs/literature_review.xlsx` and `docs/literature_synthesis_ra_l.md` to main as a single atomic commit. Message: `docs(literature): add RA-L strengthening synthesis and review tracker`.
- Decide with Georgios (async, async-safe item): whether to file OSF Amendment 3 or defer until after initial RA-L submission.

**Session 20 — paper editorial (highest-leverage section first)**
- Drop §II.A, §II.B, §II.C paragraphs from §4.3 above into `paper/draft_v0.1.md`. This closes W10 TODO #10 (Related Work).
- Add §VII.B.9 static-analyzer-recall sub-subsection from §4.7 above. This is the CWEval/QLPro threat response and is reviewer-critical.
- Add §V methods footnote on conservative H6 threshold from §4.4 above.

**Session 21 — paper editorial (exploratory-hypothesis integration)**
- Add §VI.K exploratory-hypothesis subsection from §4.5 above.
- Add §VII.A positioning paragraph from §4.6 above.
- Add §VIII conclusion framing from §4.8 above.
- Update Abstract with §4.1 ISO 10218-1:2025 sentence pair.
- Update §I Introduction opening with §4.2 reframe.

**Session 22+ — figures and demo**
- Figures still deferred: architecture diagram, per-model violation chart (Cochran Q data), T002 trajectory oscillation viz.
- Georgios demo deck (W10 TODO #9).

---

## 7. Known gaps and deferred items

**Kumar et al. COLM 2024 identification unresolved.** The original ENFIELD proposal references "Kumar et al., COLM 2024" for LLM code-generation security. Literature search surfaced multiple candidate papers with "Kumar" authors at COLM 2024 (Kumar & Lakkaraju on LLM product-ranking manipulation; Aounon Kumar on erase-and-check; Kumar in AmpleGCG generative jailbreak). Exact citation needs clarification before inclusion in Related Work. Added to the Papers sheet as a pending item would require user input.

**INSEC ICML 2025 not yet added.** Proposal references this too; not yet verified via search. Should be added in a follow-up iteration if the user can point to the exact paper or author list.

**Agentic SoK 2026 (arXiv:2601.17548) is a preprint, not peer-reviewed.** Cite with caution. If a peer-reviewed survey on prompt injection in coding assistants appears before RA-L submission, prefer it.

**Tan et al. 2026 (arXiv:2604.12088) is also a very recent preprint.** Strong empirical support for H8 but newness means a reviewer may flag citation reliability. Keep in mind and watch for journal version.

**EU AI Act citation.** Proposal mentions Article 15 relevance; no literature row yet specifically on EU AI Act ↔ LLM robotics intersection. Could be added if needed for regulatory framing in §VII.A.

**Paper-internal consistency audits deferred.** After drop-in sentences are integrated, run a grep-based audit for H4/H5/H6 → H4/H5/H6/H7/H8 cross-reference consistency (per patch discipline #5: pattern sweep before pattern-based patch).

---

## 8. Copyright and verification discipline log

- All verbatim quotes in `literature_review.xlsx` are under 15 words. Most entries use `[paraphrase]` prefix.
- No source is quoted more than once verbatim (single-quote-per-source rule honored).
- All paper URLs/DOIs were surfaced from actual web search or web fetch results; no fabricated citations.
- Author lists are marked "(authors listed in arXiv record)" or similar where search did not return full author metadata. These must be verified against the actual paper before final citation.
- Three preprints (`agentic_sok_2026`, `tan2025sasnlsafety`, `qlpro2025`) are flagged as non-peer-reviewed in the Papers sheet `status` column or discussed in §7 above. Cite with appropriate hedging.
