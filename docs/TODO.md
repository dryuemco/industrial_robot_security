# ENFIELD Active TODO

**Authoritative forward-looking tracker.** Supersedes `WEEK10_TODO.md` and `WEEK11_SPRINT.md` (both in `docs/archive/`).

**Last updated:** Session 24 close (URSim integration partially blocked + figure suite + paper audit verify), 2026-04-27.
**Tests:** 748 passing (740 baseline + 8 offline publisher/telemetry tests in S24-A).
**Phase:** Paper editorial (data + figures complete; latency + vendor-language audit closed). Pre-NTNU sprint in S25 covers URSim URCap unblock + demo deck + sim-to-real validation. NTNU Visit 1 scoped to paper editing only.

---

## Next sessions

### Session 20 — Reviewer-critical paper integrations (CLOSED 2026-04-17)

Drop-in sentences were pre-drafted in `docs/literature_synthesis_ra_l.md` section 4. Session expanded beyond originally-scoped 3 items to also include literature research (URSim precedent, static-vs-dynamic debate) and the URSim vs Gazebo decision.

**Paper editorial (originally scoped):**
- [x] §II.A / §II.B / §II.C Related Work expansion (commit `53bcb17`)
- [x] §VII.B.9 static-analyzer-recall sub-subsection (commit `05d1433`)
- [x] §V.F threshold rationale subsection (commit `0db953c`; became full subsection, not footnote)
- [x] Test count cleanup 708 → 740 across paper + OSF (commit `ba4db4b`)

**Literature research + synthesis (scope expansion):**
- [x] Literature review: +6 papers, +6 gap rows (commit `472b498`) — URSim precedent (Le & Le 2025), RA-L precedent (Hu et al. CodeBotler 2024), static analyzer threats (adversarial_secure_code_2026, secure_code_eval_rethink_2025), alternative LLM-robot safety paradigms (roboguard_2025, reachability_llm_safety_2025)
- [x] §4.9 Simulator choice rationale synthesis (commit `fa4481f`) — two-paragraph drop-in for §V.B

**Infrastructure scope correction:**
- [x] Gazebo → URSim replacement in `THREAT_MODEL.md` (ASCII box + prose) and `iso_clause_mapping.md` (evidence table) (commit `d2726eb`). `OSF_PREREGISTRATION.md` line 59 deliberately untouched; to be handled via Amendment 3 once Amendment 2 is cleared by Georgios.

**Deferred to S21 (bounded Excel patch task):**
- [ ] Add Kumar 2024 COLM paper to `docs/literature_review.xlsx` Papers sheet (identity resolved via `literature_notes.md` ref [2]: "Certifying LLM Safety against Adversarial Prompting", arXiv:2309.02705, Kumar A., Agarwal C., Srinivas S., Li A.J., Feizi S., Lakkaraju H.)

**Key S20 decision:** Gazebo/Isaac Sim deferred to post-project paper with real-robot integration. URSim 5.12.6 LTS + ROS2 Humble + ur_robot_driver selected as runtime-validation stack for ENFIELD Phase 3-4. Rationale in synthesis §4.9 (controller fidelity + scope alignment + reproducibility). Le & Le 2025 EAI provides academic precedent; CodeBotler 2024 RA-L provides venue precedent for execution-trace evaluation without physics. Gazebo stretch deliberately excluded from paper scope unless S23 go/no-go decision (below) says otherwise.

### Session 21 — Paper editorial cascade kickoff (CLOSED 2026-04-26)

S21 was paper editorial only; URSim sprint deferred to S23. Two atomic commits, both paper text additions to anchor S22 cascade:

- [x] §VI.K post-hoc exploratory hypotheses H7/H8 (commit `69adb7f`) — H7 ceiling-saturation full paragraph + H8 near-zero refusal minimal naming, cross-ref §VI.E + §II.C, no new CITE
- [x] §I Introduction opening paragraph with ISO 10218-1:2025 cybersecurity framing (commit `4ccd847`) — RA-L novelty hook, +1 CITE [iso10218_2025], multi-cite convention established (adjacent single brackets, not comma-joined)

S21 deferred to S22:
- §I Motivation/Gap/Contribution body refresh (only opening paragraph done in S21)
- Abstract ISO 2025 framing
- §VIII Conclusion refresh
- Kumar 2024 COLM Excel patch
- URSim sprint lane (all 5 items + S23 decision gate) — moved to S23

### Session 22 — Paper editorial finalization (CLOSED 2026-04-27)

Four atomic commits, all paper editorial cascade. 740 tests stable across 4 measurement points; 16 unique CITE / 26 occurrence stable; no locked-artefact disturbance.

- [x] §I body refresh (commit `9c146f7`) — Motivation compress (drop S21 opening paragraph overlap), Gap refine (drop redundant negative-survey, remove non-CITE plaintext refs INSEC/SayCan), Contribution bullet 5 add baseline-saturation ceiling effect (H7 implicit reference)
- [x] Abstract ISO 2025 framing + H4-H7 confirmatory results (commit `73d546d`) — replace smoke-test era claims (6→11 violations, 2100% increase) with confirmatory data, 152 word RA-L target, single-paragraph Markdown convention preserved
- [x] Retire safety prompt paradox claim, reframe model-dependent finding (commit `b14b138`) — §I.Contribution bullet 5 rewrite (paradox italic dropped, ceiling reordered first), §VII.A item 1 rewrite (model-dependent header + confirmatory rate triple from §VI.G L423: Qwen 10.36→10.38 unchanged, DeepSeek 9.50→3.69, CodeLlama 10.00→3.19), §VIII Conclusion minimal paradox fix (full refresh deferred to S23)
- [x] Kumar 2024 COLM Excel patch (commit `2b22fe0`) — Aounon Kumar et al. "Certifying LLM Safety against Adversarial Prompting" added as row 26 in Papers sheet, paper_id `kumar2024certifying`, topic_area `adversarial_prompting`, status `read`. Synthesis §7 Kumar gap closed. Paper L601 plaintext title correction deferred to bib tur.

S22 deferred to S23:
- §VIII Conclusion full refresh (multi-paragraph + future work paragraph) — minimal fix done in S22 C3 set sufficient baseline
- Synthesis §7 retroactive note: Kumar gap resolved
- Bib tur (16 [CITE:*] placeholders → numeric IEEE References)
- Figures part 1 (architecture diagram + per-model chart + T002 oscillation) — shifted from S22 plan
- URSim sprint lane (all 5 items + S23 decision gate) — shifted from S21 plan

### Session 23 — Paper text-complete + URSim sprint reactivation + replication kit (CLOSED 2026-04-27)

Six commits, two major lanes closed atomically.

**Paper editorial lane (Lane 1) — DONE:**
- [x] §VIII Conclusion full refresh (commit `c5fdd63`): 4-paragraph structure with framework recap, three findings, future work axes, ISO 2025 closing.
- [x] Synthesis §7 Kumar gap retroactive close (commit `d0561f6`): Aounon Kumar et al. "Certifying LLM Safety against Adversarial Prompting" COLM 2024 resolved.
- [x] Bib tur (commit `8069556`): 17 [CITE:*] placeholders → IEEE numeric [N] references; §II.C Kumar inline cite added; §I L22 adjacent multi-cite [8] [9] → [8, 9]; References block fully rewritten in IEEE format with metadata authoritative from `docs/literature_review.xlsx` Papers sheet (rows R002-R026); paper L601 plaintext "Adversarial Attacks on Code Generation" entry corrected. Paper draft is text-complete (0 unresolved citations).

**URSim sprint lane (Lane 3) — DONE (paper + repo + drift):**
- [x] Docker Engine 29.4.1 installed on PC1 from official APT repo; `docker compose` plugin verified; sudoless docker via `usermod -aG docker yunus`.
- [x] URSim e-Series pulled: `universalrobots/ursim_e-series:5.12` (image internal version 5.12.8, digest `sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51`, 3.37 GB).
- [x] `ros-humble-ur` apt meta-package installed: `ur_robot_driver` 2.12.0, `ur_client_library` 2.7.0, plus MoveIt2 stack as transitive dependency.
- [x] Smoke test passed: URSim → Dashboard `power on` + `brake release` → `Robotmode: RUNNING`, `Safetystatus: NORMAL`; `ur_control.launch.py headless_mode:=true` connected via RTDE port 30004; `/joint_states` and `/tcp_pose_broadcaster/pose` topics verified live.
- [x] Paper §V.G "Execution Setup" sub-subsection added (commit `8b13398`); §VIII URSim version drift fixed: "5.12.6 LTS" → "5.12.8", `/tcp_speed` → `/tcp_pose_broadcaster/pose`, "ROS2" → "ROS 2".
- [x] README "Runtime Stack & Simulation Environment" section + replication kit table rows in `docs/open_science_release.md` (commit `8e5aff4`).
- [x] Test count cascade fix in `docs/open_science_release.md` (commit `97c8643`): 38 → 95, 23 → 132, 574 → 740.

**Lane 1 deferral that became Lane 3 by-product:**
- Georgios "async email" item: rendered optional. Verbal approval was already given for the URSim direction during pre-S23 conversation; a written follow-up can ride along with the S24 NTNU Visit 1 demo-deck prep if useful.

**Items NOT done in S23 (moved to S24):**
- [ ] Figures part 1 (architecture diagram, per-model violation rate chart, T002 trajectory oscillation visualization). Moved to S24 because the demo deck consumes the same figures; producing them once for both targets avoids duplicate work.
- [ ] URScript batch execution harness design document. Promoted from sketch to full design now that the URSim + `ur_robot_driver` stack is validated.

**S24 decision gate — Gazebo stretch go/no-go:**
- [ ] Review buffer at S24 end. If ≥4 weeks until arXiv freeze and URSim integration clean, revisit Gazebo stretch (§VI.L or Appendix D, 1-2 paragraphs + 1 figure). If <4 weeks, Gazebo stays out of ENFIELD paper, goes to post-project extension paper.

**Polish + cross-ref audit deferred to S25+ (post-NTNU Visit 1):**
- [ ] Second-pass figure refinement.
- [ ] Table audit (Table II ISO mapping verification, Table III-naive caveat, Table VI refusal per-rule).
- [ ] Paper-internal cross-reference audit (§ numbers, figure numbers, H4-H8 references all consistent).
- [ ] Paper §V.E H6 "alongside... and" parse ambiguity fix (W10 #13, low-priority stylistic).
- [ ] Paper line ~291 "Zero refusals across all models and conditions" FIXME — verify against §VI.I + Table VI.
- [ ] `THREAT_MODEL.md` line ~91 A8 range drift check (A8.1-A8.6 vs A8.1-A8.8).
- [ ] Fix `tests/test_mcnemar_analysis.py::TestH5::test_large_effect_significant` "if a66:" guard hygiene.
- [ ] `docs/open_science_release.md` enfield_llm breakdown footer (5+14+12+7=38 → enumerate 95 components).
- [ ] `docs/open_science_release.md` line 84 `[REPLICATE.md](http://REPLICATE.md)` autolink artefact cleanup.
### Session 24 — NTNU Visit 1 demo: URSim live task execution + figures + paper audit verify (CLOSED 2026-04-27)

Closed with **8 commits**, ~5-6 hours work. Sub-lanes B and D fully closed; Sub-lane A frozen on URCap install blocker; Sub-lane C deferred to S25 per new pre-NTNU strategy (see Session 25 below).

**Sub-lane A — URSim live task execution pipeline (FROZEN at S24 close, URCap blocker):**
- [x] URScript publisher harness `enfield_urscript_runtime/urscript_publisher_node.py` + 7 offline tests (commit `0ed5feb`).
- [x] colcon `ament_pytest` test discovery fix (commit `87318aa`).
- [x] Telemetry recorder `telemetry_recorder_node.py` + T001 smoke launch + design doc (commits `b46b474`, `797e4b6`).
- [blocked] End-to-end T001 motion: software pipeline functional (publisher OK, driver OK, recorder OK) but URSim e-Series 5.12 vanilla image lacks External Control URCap; three URCap mount strategies attempted, none triggered Felix OSGi install. 60-s T001 smoke confirmed joint_pos range = 0. **RESOLUTION DEFERRED to S25 Lane 2** (custom Dockerfile / Selenium-driven PolyScope install / runtime install API research).
- [deferred-S25] 15-task batch execution script (gate-passing subset).
- [deferred-S25] URScript batch execution harness design document (paper §V.G extension).

**Sub-lane B — Figure suite (~1.5 hours, CLOSED):**
- [x] Figure 1: per-model violation rate chart (commit `f9480b0`). Source: `results/stats_e3_full/cochran_results.csv`. 4 conditions × 3 models grouped bars; H4 threshold reference line.
- [x] Figure 2: architecture diagram via Graphviz DOT (commit `2649eb4`). `scripts/plot/architecture.dot` + `render_architecture.sh` + `paper/figures/architecture.{pdf,png}`. Top-to-bottom pipeline overview; URSim cluster labelled "telemetry only".
- [x] Figure 3: retry trajectories — 4-panel small multiples (T002 strict 2-state oscillation 15→4→15→4, T010 monotonic, T013 improve+regress, T003 invariant) from `results/e3_confirmatory/e3_results.csv` (commit `4a0ec2c`).

**Sub-lane C — Demo deck (DEFERRED to S25 Lane 1):**
- [deferred-S25] All sub-items deferred. URSim status updated to "telemetry-only, motion blocked on URCap" pending S25 Lane 2 unblock.

**Sub-lane D — Paper audit verify (~30 min, CLOSED):**
- [x] §V.D Detection latency added with measured numbers (commit `9bb2038`): **mean 0.648 ms**, 95 % CI [0.638, 0.658] ms, median 0.603 ms, p95 1.046 ms, p99 1.080 ms (1500 timed calls; n_pairs=15 × iter=100 after 5-call warm-up). Proposal commitment "≤300 ms / ±15 ms" met by 463× and 1500× margins.
- [x] §IV.C "Vendor language selection" added with three-factor URScript-vs-RAPID rationale: open-source ROS 2 driver + URSim availability, permissive licensing vs RAPID/KRL IP encumbrance, single robot family stable instruction set (commit `9bb2038`).
- [x] `scripts/watchdog_microbenchmark.py` reproducible at zero cost; `paper_block` JSON summary on stdout for downstream patching.

**S24 commit summary (HEAD: `9bb2038`):**
- Sub-lane A: `0ed5feb`, `87318aa`, `b46b474`, `797e4b6`
- Sub-lane B: `f9480b0`, `2649eb4`, `4a0ec2c`
- Sub-lane D: `9bb2038`

**Strategic decision recorded at S24 close:** NTNU Visit 1 (2026-06-01 to 2026-07-31) is now scoped to **paper editing only**. arXiv preprint (June 2026) and IEEE RA-L submission (July 2026) finalize at NTNU. All infrastructure / experiment / URSim work — including S24-A URCap unblock — must complete in S25 before 2026-06-01.

**Rolled into S25 (originally S24 optional follow-ups):**
- [deferred-S25] Written status update to Georgios consolidating S20-S24 progress.
- [deferred-S25] OSF Amendment 3 draft (URSim selection + H7/H8 exploratory). Blocked by Amendment 2 (`d251c15`) pending Georgios acknowledgment.

### Session 25 — Pre-NTNU sprint: demo deck + URCap unblock + LLM URSim validation

**Strategic constraint (NEW at S24 close):** NTNU Visit 1 (2026-06-01 to 2026-07-31, 9 weeks) is scoped to **paper editing / revision / submission only**. arXiv preprint (June 2026) and IEEE RA-L submission (July 2026) finalize at NTNU. No infrastructure / experiment / URSim work at NTNU — all such work must complete in S25 before 2026-06-01.

Estimated ~6-10 hours over 1-2 sessions. Five lanes; Lanes 1-4 are pre-NTNU mandatory, Lane 5 is the original S25 scope (preserved as carry-over, deferrable to post-NTNU if S25 budget exhausts).

**Lane 1 — Demo deck (~2 hours, carried over from S24-C):**
- [ ] Format choice: Markdown→pandoc→PDF / Reveal.js HTML / Beamer LaTeX. Decision before drafting.
- [ ] Deck structure: problem → ISO 10218-1:2025 cybersecurity framing → methodology → H4-H6 confirmatory + H7-H8 exploratory → URSim live demo (post-Lane-2 unblock) → next steps.
- [ ] Reuse Q&A skeleton from archived `docs/georgios_week10_demo.md` (11 questions; post-E1 answers).
- [ ] Embed Figures 1-3 from `paper/figures/`.
- [ ] Add literature synthesis findings (H7/H8 framing, FDSP comparison, static-analyzer threat).
- [ ] If Lane 2 unblocks before deck finalization, swap "telemetry-only" caveat for live execution screenshots; otherwise keep honest "telemetry pipeline functional, motion pending URCap" framing.
- [ ] Dry run with Yunus before sending Georgios.

**Lane 2 — URCap install local workflow (~2-4 hours, S24-A reactivation):**
- [ ] Approach 1 (preferred): custom URSim Dockerfile pre-installing `externalcontrol-1.0.5.urcap` into PolyScope OSGi cache. Trade-off: loses upstream digest pin `sha256:b7ad69f5...51`; mitigated by pinning the new derived image's digest and committing the Dockerfile.
- [ ] Approach 2 (fallback): Selenium/VNC automation of PolyScope web UI URCap install workflow. Brittle but preserves vanilla URSim image.
- [ ] Approach 3 (research): runtime install API or undocumented hook investigation.
- [ ] Goal: T001 deterministic translator output → URSim live → robot moves; telemetry CSV joint_pos range > 0.
- [ ] Pre-experiment gate: re-run `tests/test_runner_mock_smoke.py` before any further URSim experiment (mandatory rule per S20 lesson).

**Lane 3 — LLM-generated URScript live execution (~2 hours, post-Lane-2):**
- [ ] Step 1: Translator T001 → URSim live → telemetry capture. Validates paper §V.G as empirical anchor (sub-lane A's original deliverable).
- [ ] Step 2 (stretch goal): LLM-generated URScript (qwen baseline rep=1 retry=0) → URSim live → observe outcome (PROTECTIVE_STOP? safety violation? successful execution?). DeepSeek 60% violation rate URSim'de gerçekten zarar verir mi?
- [ ] If LLM outputs reach URSim: 3 models × 1-3 tasks × baseline = 3-9 live executions, capture telemetry CSVs. Sim-to-real validation finding for paper §V.G or §VII.A.
- [ ] If sim-to-real result is publishable, add 1-paragraph subsection to paper §V.G before arXiv freeze cutoff.

**Lane 4 — Communication & administrative (~1-2 hours):**
- [ ] Status update email to Georgios: S20-S24 commits, paper §V.D + §IV.C audit additions, figure suite, URCap blocker + S25 unblock plan. Include §V.D detection latency numbers (mean 0.648 ms, CI [0.638, 0.658]).
- [ ] OSF Amendment 2 ack chase (`d251c15` pending). Once cleared, file Amendment 3 (URSim simulator selection + H7/H8 exploratory hypotheses).

**Lane 5 — Original S25 carry-overs (~2 hours, deferrable to post-NTNU if S25 budget exhausts):**
- [ ] **CycloneDX SBOM as packaged deliverable.** CI's `sbom-and-scan` job already generates the artefact; commit a snapshot to `docs/sbom/enfield-cyclonedx.json` (or upload to OSF as a release asset) so the replication kit "SBOM artifact (CycloneDX JSON)" row in `docs/open_science_release.md` flips from `Pending` to `Done`.
- [ ] **Ollama model weights SHA-256 hash chain.** The proposal commits to "SHA-256 hash chains for model weights and environment". Environment is covered (Docker image digest pinned in S23 C5: `sha256:b7ad69f5...51`). Models are not. Generate `docs/replication/MODEL_DIGESTS.txt` with the three model digests via `ollama show <model> --modelfile | grep digest` for `qwen2.5-coder:32b`, `deepseek-coder-v2:16b`, `codellama:34b`. Reference from README Runtime Stack section and `docs/open_science_release.md` Replication Package table.
- [ ] Verify both deliverables appear in the OSF deposit checklist (Hafta 20-24 release).

---

## Async items (no session gate)

### Requires Georgios input

- [ ] **Phase 3d decision: H1-H3 reporting strategy.** Original prereg covers H1-H3 (DM-only AST watchdog claims); current paper centers H4-H6 (LLM family). Options per `georgios_week10_demo.md` Q7 and `h_numbering_audit_2026_04_11.md` §5.4: dedicated §VI subsection / deferred appendix / separate companion report. Critical for paper completeness and Appendix A coherence.
- [ ] **OSF Amendment 2** (candidate `d251c15`) Georgios acknowledgment nudge — scope: 8→7 A8 subtypes.
- [ ] **OSF Amendment 3** (H7/H8 exploratory) — draft in synthesis §5; submit after Amendment 2 clears.

### Reference enrichment

- [ ] Resolve INSEC ICML 2025 reference identification (synthesis §7 known gap; Kumar COLM 2024 resolved via `literature_notes.md` ref [2])
- [ ] EU AI Act Article 15 literature row if needed for §VII.A regulatory framing
- [ ] ISO clause titles verification — 7 PROVISIONAL entries in `iso_10218_traceability.csv` need cross-check against Yunus's physical ISO 10218-1:2025 document before Table II is final

### Task / code cleanup (deferred, not paper-blocking)

- [ ] Task IR review (W10 #11): verify 10 non-pilot tasks written post-freeze via git log
- [ ] Code-level A6.* → A8.* rename (editorial-only per paper §IV.D Note; deferred post-submission per `georgios_week10_demo.md` Q5)
- [ ] URScript validity-gate FPR measurement on hand-curated safe URScript corpus (`georgios_week10_demo.md` Q9 — committed to update by "next meeting"; update paper §IV.B.3 if numbers available)
- [ ] Post-approval OSF clarification note (optional) describing refusal classifier implementation (commits `ad88356` + `7c76025`) and Cochran's Q exploratory status (`h_numbering_audit_2026_04_11.md` §5.5)

### Logged paper limitations (not blocking; documented in §VII.B)

- [ ] CodeLlama T012 timeout root cause (out-of-memory vs context-length vs model-specific slow path) — deferred, logged as §VII.B limitation
- [ ] Cross-quantization sensitivity analysis Q4 vs full precision on subset of tasks — future-work list per `georgios_week10_demo.md` Q8; candidate for follow-up paper or §VII.B expansion

---

## Milestones

| Date | Milestone | Notes |
|---|---|---|
| 2026-06-01 to 2026-07-31 | NTNU exchange visit | 9 weeks in Trondheim. Demo deck + Georgios paper review + main writing + submission work. |
| 2026-06 | arXiv preprint upload | After Georgios review feedback incorporated. |
| 2026-07 | IEEE RA-L submission | Target venue per revised plan v2.1. |

---

## Retired items (reference only; see git history)

Items completed or superseded and moved to `docs/archive/` at Session 19 close:

- `WEEK10_TODO.md` — items #1-#8, #5b, #6, #7, #8, #9, #15 completed; #10/#11/#13/#14 migrated above as active items; #12 resolved via Phases 3b/3c/3e/5/6-prereq (drift no longer exists; formal close via archive)
- `WEEK11_SPRINT.md` — Sessions 8-19 closure blocks preserved in archive + git history
- `attack_definitions_A1_A4.md` + `attack_definitions_A5_A8.md` — superseded by `THREAT_MODEL.md` + A8.1-A8.7 taxonomy drift
- `brief_2026-04-16_h5_finding.md` — Georgios req 3 (E3 design) superseded by actual E3 run; reqs 1-2 (Amendment 2, writing strategy) reflected in async above
- `E3_DESIGN.md` — E3 experiments complete; variant-IR protocol noted in paper §VII.B.8 future work
- `georgios_week10_demo.md` — will be superseded by Session 24 demo deck; Q&A skeleton reused
- `h_numbering_audit_2026_04_11.md` — all remediation phases complete; residual items migrated above (H1-H3 decision, line ~291 FIXME verify, THREAT_MODEL A8 range, post-approval clarification)
- `literature_notes.md` — superseded by `literature_review.xlsx` + `literature_synthesis_ra_l.md`; Kumar COLM 2024 reference resolved (see Session 20 above)
- `revised_plan_v2.md` — plan evolution captured in git log and synthesis document

---

## Discipline active

Patch discipline (8 rules in memory #6), commit directly to main, ASCII commit messages, locked artefacts list below. Runner mandatory gate for any confirmatory experiment (`tests/test_runner_mock_smoke.py`). See `SESSION_20_HANDOFF_PROMPT.md` for full context if starting new session.

### Locked artefacts (untouchable per Amendment 1)

- Paper §V.E
- Paper §VI.G H5 decision sentence
- Paper Appendix A H5 description
- OSF preregistration Amendment 1 block
- Paper §VI.H watchdog-via-invalidation paragraph
