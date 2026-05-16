# ENFIELD Active TODO

**Authoritative forward-looking tracker.** Supersedes `WEEK10_TODO.md` and `WEEK11_SPRINT.md` (both in `docs/archive/`).

**Last updated:** S28 closed (Tier 0 fulfilment: TODO sync + MODEL_DIGESTS.txt; Tier 1.5 4/5: Q4 audit resolution, responsible_disclosure.md stub, SBOM scope doc, RA-L anonymity verdict), 2026-05-11.
**Tests:** 788 passing (740 baseline + 14 urscript_runtime + 18 task_complexity + 16 complexity_correlation; +48 from S26 Lane 6).
**Phase:** Proposal-defensibility hardening. S27 + S28 swept the end-of-S27 audit hits via 10 atomic commits across paper / README / OSF Amendment 3 / replication artefacts (MODEL_DIGESTS.txt, responsible_disclosure.md, open_science_release.md Supply Chain Transparency subsection). NTNU Visit 1 (2026-06-01) scope: paper editorial pass + anonymized RA-L submission derivative on `submission/ral-2026` branch + replication kit dry-run + Appendix B expansion or `docs/REPLICATION.md`.

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
- [x] OSF Amendment 3 draft (URSim selection + H7/H8 exploratory). Filed to OSF 2026-05-12 (S29); Amendment 2 acknowledged by Georgios 2026-05-12.

### Session 25 — Pre-NTNU sprint: demo deck + URCap unblock (CLOSED 2026-05-04)

**Strategic constraint (NEW at S24 close):** NTNU Visit 1 (2026-06-01 to 2026-07-31, 9 weeks) is scoped to **paper editing / revision / submission only**. arXiv preprint (June 2026) and IEEE RA-L submission (July 2026) finalize at NTNU. No infrastructure / experiment / URSim work at NTNU — all such work must complete in S25 before 2026-06-01.

Estimated ~6-10 hours over 1-2 sessions. Five lanes; Lanes 1-4 are pre-NTNU mandatory, Lane 5 is the original S25 scope (preserved as carry-over, deferrable to post-NTNU if S25 budget exhausts).

**Lane 1 — Demo deck (~2 hours, CLOSED):**
- [x] Format choice: Markdown → pandoc → xelatex PDF (commit `7fd813d`).
- [x] Deck structure: 11-question scaffold derived from archived `docs/georgios_week10_demo.md`; sections — Project Status (Q1-Q4), Empirical Findings (Q5-Q8), Roadmap to NTNU Visit 1 (Q9-Q11) (commit `b5a2f17`).
- [x] Reuse Q&A skeleton from archived `docs/georgios_week10_demo.md`.
- [x] Embed Figures 1-3 from `paper/figures/`.
- [x] Add literature synthesis findings (H7/H8 framing surfaces in Q8; ceiling-saturation written into the empirical narrative).
- [x] Lane 2 unblocked before deck finalization; "telemetry-only" caveat replaced with live URScript execution evidence in S26 commit `fdc19ef` (Q4/Q9-Q11 update; T001 trajectory range table + C204A3 PROTECTIVE_STOP framing).
- [ ] Dry run with Yunus before sending Georgios. **Pending — gates send to Georgios in Lane 4.**

**Lane 2 — URCap install local workflow (~2-4 hours, CLOSED):**
- [x] Approach 1 shipped: custom URSim Dockerfile bakes `externalcontrol-1.0.5.urcap` into the image (commit `095c08c`, image tag `enfield-ursim:5.12-urcap-1.0.5`). Upstream digest pin lost; new derived image documented in `docs/replication.md`.
- [skipped] Approach 2 (Selenium/VNC) — not needed; Approach 1 succeeded.
- [skipped] Approach 3 (runtime install API) — no public hook found during S24 investigation.
- [x] Goal met: T001 trajectory live execution verified — joint_pos range > 0 across all 6 joints; range table in S26 demo deck Q4 (commit `fdc19ef`). Trajectory terminated in `PROTECTIVE_STOP` (Error C204A3, qdd discontinuity) under collaborative-mode 250 mm/s envelope; this is itself a useful baseline for Lane 3 sim-to-real anchor.
- [x] Pre-experiment gate held: 740 tests stable across all S25 commits.
- [x] Beyond Approach 1: ROS2 driver topic-mode path did not produce motion in URSim headless mode without an active URCap dispatcher loop. Publisher node extended with direct-TCP injection mode (`inject_mode='primary_tcp'`, commit `ab78dc5`). Trailing entry-point lines stripped on the `primary_tcp` path only (URSim Secondary parser rejects them otherwise; closes `TCPReceiver` and cascades to `PROTECTIVE_STOP C154A0`).
- [x] Telemetry recorder `duration_s` integer/float ROS2 Humble strict-type bug investigated (`a47e199`, `65d84c6` attempts) and reverted to source-pristine state (`14e4d8d`). Wrapper-level workaround documented in S26 README "Known Limitations" (commit `5c7be5e`).

**Lane 3 — LLM-generated URScript live execution (DEFERRED to S26):**
- [deferred-S26] Step 1 closed inside Lane 2 (hand-crafted T001 already executed live; trajectory + C204A3 baseline observed).
- [deferred-S26] Step 2 (LLM-generated URScript live exec) — promoted to S26 priority. New infrastructure shipped in S26: telemetry recorder `safety_mode` + `robot_mode` subscriptions (commit `3da634d`), publisher `urscript_path` parameter for verbatim LLM URScript injection + generic `run_urscript_pilot.sh` wrapper (commit `d72022e`).
- [deferred-S26] Pilot scope: T001 + T004 + T005 (collaborative-mode only, brutal-recommended; matches hand-crafted T001 safety envelope). Paper §VI sim-to-real preliminary subsection. Faz B/C in S26 (currently in-progress).

**Lane 4 — Communication & administrative (DEFERRED to S26):**
- [deferred-S26] Status update email to Georgios — gated on Lane 1 dry-run (above) and Lane 3 sim-to-real result (below); all three close in S26.
- [deferred-S26] OSF Amendment 2 ack chase + Amendment 3 filing — scope expanded in S26 to also cover IP drift documentation (see async items below).

**Lane 5 — Original S25 carry-overs (DEFERRED to post-NTNU; was already declared deferrable):**
- [ ] **CycloneDX SBOM as packaged deliverable.** CI's `sbom-and-scan` job already generates the artefact; commit a snapshot to `docs/sbom/enfield-cyclonedx.json` (or upload to OSF as a release asset) so the replication kit "SBOM artifact (CycloneDX JSON)" row in `docs/open_science_release.md` flips from `Pending` to `Done`.
- [ ] **Ollama model weights SHA-256 hash chain.** The proposal commits to "SHA-256 hash chains for model weights and environment". Environment is covered (Docker image digest pinned in S23 C5: `sha256:b7ad69f5...51`). Models are not. Generate `docs/replication/MODEL_DIGESTS.txt` with the three model digests via `ollama show <model> --modelfile | grep digest` for `qwen2.5-coder:32b`, `deepseek-coder-v2:16b`, `codellama:34b`. Reference from README Runtime Stack section and `docs/open_science_release.md` Replication Package table.
- [ ] Verify both deliverables appear in the OSF deposit checklist (Hafta 20-24 release).

**S25 commit summary (HEAD `14e4d8d`):**
- Lane 1 deck (Sub-lane C): `7fd813d`, `b5a2f17`
- Lane 2 URCap + injection: `095c08c`, `ab78dc5`
- Telemetry duration_s investigation: `a47e199` (descriptor attempt), `65d84c6` (read-site fallback), `14e4d8d` (revert to pristine; see "RE-ATTEMPTED ROUTES" guidance in S26 handoff)

---

### Session 26 — Lane 3 sim-to-real pilot + Demo deck refresh + README discipline + Lane 6 Task Complexity (CLOSED 2026-05-11)

**Strategic position:** S25 unblocked URSim live execution; S26 promotes Lane 3 to priority and threads three smaller administrative commits around it. Pre-NTNU sprint window remains (2026-05-05 to 2026-05-31, ~26 days).

**Closed in S26 so far (HEAD `8a1f31f`):**
- [x] Demo deck Q4/Q9-Q11 update reflecting Lane 2 closure (commit `fdc19ef`). Q4 swapped from "telemetry-only blocked" to live execution with T001 joint range table + C204A3 framing; Q9 reframed past-tense as resolved; Q10/Q11 lane status table updated; replication-package row points at `enfield-ursim:5.12-urcap-1.0.5`.
- [x] README "Known Limitations" section documenting `telemetry_recorder` `duration_s` float-only requirement under rclpy Humble (commit `5c7be5e`). Closes the editorial gap left by S25 revert `14e4d8d`. Marker for ROS2 Iron upgrade revisit.
- [x] Telemetry recorder `safety_mode` + `robot_mode` subscriptions (commit `3da634d`). CSV schema extended from 23 to 25 columns (`safety_mode`, `robot_mode` raw integer codes); INFO-level transition logging on every state change. `ur_dashboard_msgs` added to `package.xml`. Test cascade fix included a defensive filter on the fake-publisher stamp range (sec 2000-2099) to make the offline test resilient against latched messages from any external `ur_robot_driver` controller in the same DDS domain — pre-existing `run_tests.sh` coverage gap (urscript_runtime not in CI list) had hidden this latent bug for two weeks.
- [x] Publisher `urscript_path` parameter + generic pilot wrapper (commit `d72022e`). Module-level `_load_script` helper dispatches IR translation vs verbatim file read; `task_ir_path` and `urscript_path` are mutually exclusive (validation early-return). Launch arg added with `default_value=''` (backward compat preserved). New `enfield_urscript_runtime/scripts/run_urscript_pilot.sh` wrapper takes `URSCRIPT_PATH` env, threads it through `t001_smoke.launch.py`, and reports `PROTECTIVE_STOP` row count in step-5 summary. New unit-test file with 6 cases covering both paths and validation errors. **8 → 14 colcon tests in this package; 740/740 still green via `run_tests.sh --fast`.**

- [x] **Lane 6 — Task Complexity Characterization (Georgios feedback) (CLOSED 2026-05-11).** Responds to host-supervisor question whether task complexity modulates the violation patterns reported under H4–H6. Six atomic commits `b979881` (TCS skeleton), `e5bc8e2` (schema-calibrated to task_ir_v1), `57f620b` (Spearman + tertile bootstrap CVR analysis), `3a0756d` (continuous metrics count/severity bypassing CVR ceiling), `d5cc83b` (TASK_COMPLEXITY_SPEC + OSF_AMENDMENT_3 drafts), `8a1f31f` (paper §IV.E + §VII.C anchor-based insertion). Built deterministic outcome-blind TCS from baseline Task IR (15 features, A1–A8 attack-surface aligned, frozen weights at `configs/tcs_weights.yaml`); joined per-task TCS to E1/E2/E3 outcomes; reported Spearman ρ + tertile-stratified mean with percentile bootstrap 95% CI for 3 outcome metrics (CVR binary, violation count, severity_max). **Three findings:** (1) CVR ceiling in 26/30 strata with constant CVR=1.0 (Spearman undefined); (2) counter-intuitive **negative** complexity-count correlation in 3 baseline strata (E3 Qwen2.5: ρ=−0.54, p=0.037, n=15; E3 DeepSeek: ρ=−0.53; E1 Qwen2.5: ρ=−0.45) + severity ρ=−0.64 (p=0.010) in E1 DeepSeek safety — reported as **exploratory conjecture**, NOT preregistered as directional; (3) E2 adversarial strata show no significant complexity-density association. Paper: §IV.E "Task Complexity Characterization" + §VII.C "Exploratory Complexity-Stratified Subgroup Analysis (H8)" inserted; §VII.C "EU AI Act Alignment" renamed to §VII.D. Docs: `docs/TASK_COMPLEXITY_SPEC.md` (reviewer-facing spec, frozen weight table, sensitivity-analysis plan); `docs/OSF_AMENDMENT_3_DRAFT.md` (H7 URSim simulator lock + H8 exploratory analysis, ready to file once Amendment 2 is acknowledged). Tests: 740 → 788 (+48 new, all passing).

**Open lanes — S26 priority order:**

**Lane 3 (continued) — sim-to-real pilot (in-progress, ~3-5 hours remaining):**
- [x] Faz A1: telemetry recorder safety_mode + robot_mode (commit `3da634d`).
- [x] Faz A2: publisher urscript_path mode + pilot wrapper (commit `d72022e`).
- [x] Determinism check (this session): `qwen2.5-coder:32b` Q4_K_M under temperature=0.0 is bit-deterministic for the T001 baseline prompt; same prompt produces identical SHA-256 across two consecutive calls. **However**, the current digest `b92d6a0b...` (modified 2026-05-05) differs from the digest active during session 12 (2026-04-15), so the historical `e1_pilot_session12` artifacts cannot be bit-reproduced. Lane 3 pilot therefore generates fresh baselines under the pinned current digest rather than reusing rep1 files (decision: B-fresh).
- [x] Faz B: T001+T004+T005 collaborative baselines generated via `qwen2.5-coder:32b` Q4_K_M, manifest digest `b92d6a0b...`, temp=0.0. Output under `results/lane3_pilot/code/` with provenance front-matter (gitignored payload).
- [x] Faz B: live execution against URSim e-Series 5.12 (image `enfield-ursim:5.12-urcap-1.0.5`, container `172.17.0.2`) completed for all 3 tasks. Telemetry CSVs captured under broad `results/` ignore; URControl logs corroborate every outcome.
- [x] Faz C (S27 commit `638e12a`): paper §VI.L "URSim Pre-Execution Failure Modes" inserted; §VIII (i) future-work refreshed in same commit. N=3 collab pilot documents three pre-execution failure classes (C1 API misuse, C2 URCap orchestration race, C3 reachability) all evading static watchdog DM-1..7. Paper 743 -> 755 lines.

**Faz B closure (2026-05-06) — 3 pre-execution failure classes identified.** Full evidence in [`results/lane3_pilot/notes/2026-05-06_faz_b_c_findings.md`](../results/lane3_pilot/notes/2026-05-06_faz_b_c_findings.md) (force-tracked under broad `results/` ignore rule).

- **T001 — Class 1 (Syntactic-OK semantic API misuse):** generated code passes a joint vector to `get_inverse_kin`, which requires a Pose `p[x,y,z,a,b,c]`. URControl rejected the script pre-execution; reproduced 3× deterministically. Joint range = 0 (no motion). Static watchdog DM-1..7 has no API-type checker — coverage gap motivates DM-8 design.
- **T004 — Class 2 (Timing/state-coupling failure):** URScript injected while URCap was in `PROGRAM_STATE_PAUSED`; reverse_socket connection not yet established. Silent abandon to `STOPPED`; no runtime error logged. Joint range = 0. Race window is wrapper/PolyScope timing — motivates auto-Play before injection.
- **T005 — Class 3 (Workspace reachability failure):** generated waypoints place TCP at Z = −0.198 m (below robot base). IK solver rejected the request pre-execution. Joint range = 0.

All three failures evade static watchdog DM-1..7 and only surface at the URSim runtime layer. This motivates a URCap-mediated execution gate as a complementary detection layer to the static watchdog — direct anchor for Faz C paper §VI insertion. Lane 3 Faz B closes; Faz C remains the open lane item.

**Lane 4 (continued from S25) — Georgios sync + OSF Amendment 3 (~1-2 hours):**
- [x] Status update email sent end of S27 (Variant 1 "Steady", three attachments). Two open asks: (1) Amendment 2 acknowledgment chase, (2) Amendment 3 review. **Reply received 2026-05-12: Amendment 2 acknowledged, Amendment 3 approved.**
- [x] OSF Amendment 3 draft committed at `d5cc83b` (`docs/OSF_AMENDMENT_3_DRAFT.md`): H7 URSim simulator lock + H8 complexity-stratified exploratory analysis (non-directional). Operational Disclosures section (IP drift + model digest drift) appended in S27 commit `6a323b4`. Quantization Documentation Drift subsection appended in S28 commit `5678f5a`. IP drift wording updated for 2026-05-11 DHCP refresh in S29 commit `5147bca`. **Filed to OSF 2026-05-12 (S29), revision id `6a02e67cf944b975e74bbdbd`, awaiting admin moderator approval (48h SLA).**

**TODO refresh (this commit):** S25 close + S26 in-progress entry + 6 new async items (below).



### Session 27 — Lane 3 Faz C + Amendment 3 expand + locked-artefact cosmetic override + Appendix B fix (CLOSED 2026-05-11)

Four atomic commits. Lane 3 fully closed; Lane 4 email sent (awaiting reply); cosmetic-only override of locked artefact §VI.H precedented.

- [x] `638e12a` — `docs(paper)`: insert §VI.L URSim live-exec pilot + refresh §VIII (i). Paper 743 -> 755 lines.
- [x] `6a323b4` — `docs(osf)`: expand Amendment 3 draft with Operational Disclosures (IP drift + model digest drift).
- [x] `a3c83ce` — `docs(paper)`: strip emoji from §VI.H H5 decision tables (IEEE RA-L compliance). **First locked-artefact cosmetic-only override** with explicit Yunus approval; establishes precedent for non-semantic touches (emoji removal, typo correction, formatting consistency, test-count updates) on locked content. Semantic changes remain blocked.
- [x] `b0dd4b2` — `docs(paper)`: Appendix B integrity fix (test count 740 -> 788 + duplicate prose removal). Surfaced by end-of-S27 brutal audit; establishes audit pattern of checking Appendix B + README for stale test counts whenever commits change the count.

**Tests:** 788 stable across all four commits.

**S27 deferred to S28:**
- TODO.md update (this commit).
- S27 closure commit (this commit).
- `docs/replication/MODEL_DIGESTS.txt` populate — blocked: PC2 (`192.168.1.4:11434`) unreachable during S27 audit and again during S28 probe (HTTP 000). Deferred to S29 once PC2 is back up.

### Session 28 — Tier 0 fulfilment + Tier 1.5 sweep + Q4 audit resolution (CLOSED 2026-05-11)

Six atomic commits closed S28:

- [x] `64e9acd` — S27 closure cleanup (TODO sync, S28 audit followups recorded).
- [x] `bf2626e` — Tier 0 #3: `docs/replication/MODEL_DIGESTS.txt` populate. Three model pins (qwen2.5-coder:32b Q4_K_M, deepseek-coder-v2:16b Q4_0, codellama:34b Q4_0). Q4 quantization mismatch flagged for Tier 1.5 followup.
- [x] `5678f5a` — Tier 1.5 (bonus): Q4 quantization audit RESOLVED via documentation correction. Eight edits across four files (paper §V.A Table III + §VII.B.4 + §VII.B.7 prose, README.md table, OSF Amendment 3 new Quantization Documentation Drift subsection — third operational disclosure). No re-pull, no re-run; OSF_PREREGISTRATION.md L321-323 not back-edited.
- [x] `1d409b2` — Tier 1.5 #1: `docs/responsible_disclosure.md` v0.1 stub (117 lines, 6 sections). Three-tier access (public / verified researcher / vendor) + 90-day vendor advance-notice timeline + reporting channel. v1.0 revision items deferred to Month-6 OSF release.
- [x] `a200cfe` — Tier 1.5 #2: SBOM scope documentation. New Supply Chain Transparency subsection in `docs/open_science_release.md` (4 mechanisms: CycloneDX SBOM, Trivy scan, MODEL_DIGESTS.txt, URSim digest pin) + scope gaps + replicator guidance. Pre-existing internal inconsistency at L74 fixed (Pending→Done).
- [x] `90f7e08` — Tier 1.5 #3: RA-L anonymity verdict (web-confirmed: RA-L IS double-blind per IEEE RAS Feb 2025 policy). Workflow decision: arXiv preprint = named, RA-L submission = anonymized derivative on `submission/ral-2026` branch. Pre-submission anonymization pass queued in Tier 2 with six explicit edit categories.

**Carried forward to S29:** Tier 1.5 #4 (Appendix B expansion or `docs/REPLICATION.md`, ~30-60 min); Tier 2 NTNU Visit 1 prep (3 items, including the new pre-submission anonymization). Lane 4 closed in S29 (Georgios reply received 2026-05-12; Amendment 3 filed to OSF moderator queue).

**Tests:** 788 stable across all six commits. **Paper:** 755 lines (one editorial commit at 5678f5a; data + claims unchanged). **New artefacts in repo:** `docs/replication/MODEL_DIGESTS.txt`, `docs/responsible_disclosure.md`, Supply Chain Transparency subsection in `docs/open_science_release.md`, Quantization Documentation Drift subsection in `docs/OSF_AMENDMENT_3_DRAFT.md`.

Six audit hits, categorized by tier. Tier 0 fulfils Amendment 3 wording; Tier 1.5 advances proposal-commitment integrity; Tier 2 is NTNU Visit 1 prep (lands 2026-06-01); Tier 3 is post-NTNU backlog.

**Tier 0 — Amendment 3 fulfilment (this sprint):**
- [x] TODO.md update (this commit).
- [x] S27 closure commit (this commit).
- [x] `docs/replication/MODEL_DIGESTS.txt` populated (S28; this commit). PC2 reachable at `192.168.1.5:11434` (memory + OSF prereg recorded `.4`; IP drift observed again — to be reflected in memory after Yunus confirmation, and addressed in Amendment 3 Operational Disclosures historical-IP table). Source: Ollama `/api/tags` JSON endpoint (more robust than the originally-proposed `ollama show --modelfile` scrape, which returned empty during S27 audit due to CLI version drift). File records three model tags with quantization, sha256 manifest digest, and modified_at timestamp. **Q4 quantization mismatch surfaced — see Tier 1.5 below.**

**Tier 1.5 — Proposal-commitment integrity (advance opportunistically):**
- [x] `docs/responsible_disclosure.md` stub created (S28; this commit). v0.1 draft covering: §1 Tier 1 public default (paper, watchdog rules, threat model, baseline IRs, attack prompt-template library, aggregated results, replication kit); §2 Tier 2 verified researcher access with institutional-email gate + research-purpose statement + 14-day response SLA (current concrete Tier 2 artefacts: raw E1/E2/E3 prompt-and-output corpus, future variant-IR scripts); §3 Tier 3 vendor pre-notification with 90-day timeline (Day 0 written notice, Days 1–60 fix window, Days 60–90 coordination, Day 90 release); vendor scope (Alibaba/DeepSeek/Meta/Ollama + Universal Robots, extending to ABB+KUKA on future work); §4 reporting channel (project-lead email + PGP TBD); §5 out-of-scope clarifications; §6 revision policy with five outstanding items for v1.0. **v1.0 revision items (deferred to Month-6 OSF release):** project-alias email, published PGP key, concrete Tier 2 verification workflow template, vendor contact directory, host-supervisor sign-off on 90-day timeline, reconciliation with NTNU institutional disclosure policy.
- [x] SBOM scope documentation completed (S28; this commit). New subsection `### Supply Chain Transparency (SBOM Scope, Model Pins, Image Pins)` inserted in `docs/open_science_release.md` between Code Release and Replication Package. Four mechanisms documented: (1) CycloneDX SBOM (Python-deps via requirements.txt only); (2) Trivy scan (full enfield:scan Docker image, CRITICAL+HIGH severity, broader than SBOM — audit hit had missed this); (3) Model weight pins (MODEL_DIGESTS.txt with quantization + sha256 + modified_at; Amendment 3 cross-ref); (4) URSim image digest pin (Dockerfile L24 sha256:b7ad69f5...; enfield.ursim.base.digest + enfield.urcap.sha256 labels). Scope gaps acknowledged: no docker-build-ursim CI job (post-NTNU backlog), no model-weight SBOM standardisation (community gap), no apt-level SBOM (post-NTNU backlog). Same commit also fixes pre-existing internal inconsistency at L74 (SBOM artifact status "Pending" → "Done" with cross-ref to the new subsection).
- [ ] Appendix B expansion or `docs/REPLICATION.md`. Current Appendix B covers smoke test only; E1/E2/E3 confirmatory reproduction commands needed. ~30-60 min.
- [x] Anonymity check for IEEE RA-L submission RESOLVED (S28; this commit). Web-confirmed verdict: **RA-L IS double-blind**. Per IEEE RAS policy effective 1 February 2025 (https://www.ieee-ras.org/publications/rules-for-the-double-anonymous-review-process/), all fully RAS-sponsored journals (RA-L, RAM, T-ASE, T-RO, T-FR, RA-P) employ double-anonymous peer review. Names, affiliations, and funding-agency acknowledgments must not appear anywhere in the manuscript, appendix, or supplementary material before submission. **Scope is broader than the audit hit's Appendix B GitHub URL** — also affected: author names (Yunus, Cogurcu), supervisor (Spathoulas, Georgios), affiliations (NTNU, Çukurova), funding (Horizon Europe Grant 101120657), GitHub handle (`dryuemco`), and the project name "ENFIELD" itself (traceable via Horizon Europe grant DB to authors). **Workflow decision:** maintain current `paper/draft_v0.1.md` as the **arXiv preprint version** (June 2026 target; named, full attribution). Produce an **anonymized derivative** for RA-L submission (July 2026 target). Anonymization pass is a Tier 2 NTNU Visit 1 prep item (see new entry below) so Georgios can sign off on the strategy during the exchange visit. Do NOT anonymize the main draft now; do it in a submission branch.
- [x] **Q4 quantization audit** RESOLVED (S28; this commit). (a) Grep enumeration: 22 Q4_K_M hits across 10 files; 5 Q4_0 hits across 3 files; paper §V.A Table III row 233 already listed CodeLlama as Q4_0 (pre-existing internal inconsistency with §VII.B.4 prose). (b) Drift timeline from MODEL_DIGESTS.txt `modified_at`: codellama:34b since 2026-03-25, deepseek-coder-v2:16b since 2026-04-03, qwen2.5-coder:32b since 2026-05-05; DeepSeek + CodeLlama never were Q4_K_M on PC2 — Ollama default tags resolved to Q4_0 at initial pull, no explicit `-q4_K_M` variant ever requested. (c) Decision: **option (c)** documentation correction. Rationale: paper §VII.B.4 already treats Q4-class as a fixed protocol parameter, no inference depends on which Q4 variant, and re-pull + full E1/E2/E3 re-run is data-invalidating with no scientific gain. Edits in this commit: paper §V.A Table III (L232 DeepSeek Q4_K_M→Q4_0), paper §VII.B.4 + §VII.B.7 prose (single-Q4_K_M → mixed Q4 with disclosure), README.md L82-83 (DeepSeek + CodeLlama Q4_K_M→Q4_0), `OSF_AMENDMENT_3_DRAFT.md` preamble "two"→"three" + appended `### Quantization Documentation Drift — Disclosure` subsection. `OSF_PREREGISTRATION.md` L321-323 not back-edited; Amendment 3 is authoritative correction per no-prereg-touch policy.

**Tier 2 — NTNU Visit 1 prep (3 weeks out):**
- [ ] `scripts/build_paper_pdf.sh`. Replace S27 `/tmp` ad-hoc pandoc with repo'd script; xelatex + DejaVu Serif + DejaVu Sans Mono pinned; output to `paper/build/`.
- [ ] Replication kit dry-run. Fresh clone -> tests -> results bit-reproducible.
- [ ] **Pre-submission anonymization pass for RA-L** (surfaced by S28 Tier 1.5 #3 verdict; NTNU Visit 1 scope). Produce an anonymized derivative of `paper/draft_v0.1.md` for double-anonymous RA-L submission. Required edits: (a) replace Appendix B GitHub URL `https://github.com/dryuemco/industrial_robot_security` with `[anonymized for double-anonymous review; repository URL provided upon acceptance]` or an anonymous-mirror (e.g. `https://anonymous.4open.science/`) if Georgios prefers; (b) remove or generalise author and supervisor names; (c) remove or generalise affiliations (NTNU, Çukurova); (d) remove funding acknowledgment (Horizon Europe Grant 101120657 — IEEE RAS policy explicitly requires this); (e) decide on project-name handling: keep `ENFIELD` but soften with `a Horizon Europe-funded framework`-style phrasing, or replace with generic descriptor (`our framework`) throughout; consult Georgios on which approach RA-L editors typically accept. (f) Verify own-work citations are third-person (`It was shown in [N]...` not `We showed in [N]...`). Workflow: maintain the named draft on `main`; the anonymized version is produced on a `submission/ral-2026` branch and only merged or tagged at the submission moment. Estimated effort: 1-2 hours (substantial because identifier surface is broad).

**Tier 3 — Post-NTNU backlog:**
- [ ] DM-8 API-type checker design doc (closes §VI.L C1 gap).
- [ ] Lane 3 model matrix expansion (T001/T004/T005 for DeepSeek-Coder-V2-16B + CodeLlama-34B).
- [ ] TCS sensitivity analysis (0.7/0.3 + 0.3/0.7 blends, Amendment 3 H8 commitment).
- [ ] CycloneDX SBOM scope extension (model weights + URSim image layers).
- [ ] Wrapper auto-Play (C2 race window remedy).

### Requires Georgios input

- [ ] **Phase 3d decision: H1-H3 reporting strategy.** Original prereg covers H1-H3 (DM-only AST watchdog claims); current paper centers H4-H6 (LLM family). Options per `georgios_week10_demo.md` Q7 and `h_numbering_audit_2026_04_11.md` §5.4: dedicated §VI subsection / deferred appendix / separate companion report. Critical for paper completeness and Appendix A coherence.
- [x] **OSF Amendment 2** (candidate `d251c15`) Georgios acknowledgment **received 2026-05-12** (S29) — scope: 8→7 A8 subtypes.
- [x] **OSF Amendment 3** (H7/H8 exploratory) — **filed to OSF 2026-05-12** (S29), revision id `6a02e67cf944b975e74bbdbd`, awaiting admin moderator approval (48h SLA). Operational Disclosures expanded across S26-S29 to cover three categories: (1) model digest drift (session 12 `qwen2.5-coder:32b` digest unrecoverable; confirmatory-run digest `b92d6a0b...` pinned for Month-6 deposit); (2) Ollama host IP drift (PC2 `.5`-`.4`-`.5` across 2026-05-05 and 2026-05-11 DHCP refreshes; IP framed as operational metadata, not protocol parameter); (3) Q4 quantization documentation drift (deepseek-coder-v2:16b and codellama:34b pulled as Q4_0 not Q4_K_M; S28 commit `5678f5a` corrected paper §V.A Table III + §VII.B.4 + §VII.B.7 + README; OSF_PREREGISTRATION.md L321-323 preserved unedited per no-retroactive-edit policy).

### Reference enrichment

- [ ] Resolve INSEC ICML 2025 reference identification (synthesis §7 known gap; Kumar COLM 2024 resolved via `literature_notes.md` ref [2])
- [ ] EU AI Act Article 15 literature row if needed for §VII.A regulatory framing
- [ ] ISO clause titles verification — 7 PROVISIONAL entries in `iso_10218_traceability.csv` need cross-check against Yunus's physical ISO 10218-1:2025 document before Table II is final

### Task / code cleanup (deferred, not paper-blocking)

- [ ] Task IR review (W10 #11): verify 10 non-pilot tasks written post-freeze via git log
- [ ] Code-level A6.* → A8.* rename (editorial-only per paper §IV.D Note; deferred post-submission per `georgios_week10_demo.md` Q5)
- [ ] URScript validity-gate FPR measurement on hand-curated safe URScript corpus (`georgios_week10_demo.md` Q9 — committed to update by "next meeting"; update paper §IV.B.3 if numbers available)
- [ ] Post-approval OSF clarification note (optional) describing refusal classifier implementation (commits `ad88356` + `7c76025`) and Cochran's Q exploratory status (`h_numbering_audit_2026_04_11.md` §5.5)
- [ ] **`scripts/llm_experiment_runner.py` IP drift in docstring** — four `OLLAMA_HOST=http://192.168.1.5:11434` usage examples in the module header docstring point at the old PC2 IP. Pure documentation; runtime is `OLLAMA_HOST` env-driven, so functional impact is zero. Single-replace patch when next touching the file.
- [ ] **`docs/georgios_s25_demo.md` Q3 architecture paragraph** still names PC2 IP as `192.168.1.5:11434`. Cosmetic; the IP is referenced inside a one-paragraph architecture description, not used by code. Can fold into the next demo deck patch if any.
- [ ] **`enfield_llm/factory.py` `seed` parameter mock-only bug** — `create_client(..., seed=N)` accepts the kwarg in the live-provider path but only forwards it to `mock_kwargs`; the Ollama client never sees the seed. Functional impact today is zero (temperature=0 already gives bit-identical output for fixed weights — verified S26 determinism check). Becomes a real bug if any future experiment intentionally requests sampling at temperature > 0 with seed control. Fix scope: small — wire the seed through `OpenAICompatibleClient` to Ollama's `seed` request param.
- [ ] **`scripts/run_tests.sh` orphan: `enfield_urscript_runtime` not in CI test list** — surfaced in S26 when the telemetry test broke under the new `safety_mode` cascade and `--fast` reported 740/740 green. The package's tests only run via direct `colcon test --packages-select enfield_urscript_runtime`. Add a `run_suite "enfield_urscript_runtime" "enfield_urscript_runtime" "enfield_urscript_runtime/test/"` line (line 80-85 region) when next touching the script.
- [ ] **README `run_t001_smoke.sh` reference path** — README "Known Limitations" mentions the wrapper by basename; the actual location is `enfield_urscript_runtime/scripts/run_t001_smoke.sh` (not the repo-root `scripts/`). Cosmetic; can be expanded with explicit relative path on the next README pass.

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


---

### Session 29 — Lane 4 closure + Amendment 3 OSF filing (2026-05-12)

Single-thread sprint focused on the long-running Lane 4 blocker. Georgios reply arrived end of 2026-05-11 / start of 2026-05-12: Amendment 2 explicitly acknowledged, Amendment 3 draft approved, no other action items. Amendment 3 IP wording updated for the 2026-05-11 DHCP refresh and filed to OSF on 2026-05-12.

Commits this session (chronological):

- [x] `5147bca` — `docs(s29)`: append 2026-05-11 DHCP drift to Amendment 3 IP clarification (Obs 1 from S28 closure followups).
- [x] OSF Amendment 3 filed via web UI on 2026-05-12. Revision id `6a02e67cf944b975e74bbdbd`. Form steps 1-8 (Justification + Overview + Research Design + Sampling + Variables + Analysis Plan + Other + Review) populated cumulatively; updated registration questions: Research questions/hypotheses, Study design, Data collection procedures, Starting and stopping rules, Indices, Additional blinding, Sample size, Transformations, Inference criteria, Manipulated variables, Explanation of foreknowledge, Other planned analysis, Sample size rationale, Context and additional information, Data inclusion and exclusion, Missing data, Measured variables, Statistical models. Admin moderator approval pending (48h SLA from filing timestamp).
- [x] `docs(s29)`: this commit. Lane 4 closure documentation across `docs/TODO.md`.

End-of-S29 state:

- OSF Amendment 3: filed, in admin moderator queue, no further action required from us until OSF returns an approval or rejection notification.
- Lane 4 (Georgios sync + OSF Amendment 3 filing): **CLOSED** after spanning S25 (deferred) → S26 (deferred) → S27 (email sent, awaiting reply) → S29 (reply received, Amendment 3 filed). Total elapsed: ~4 sessions, blocked on async supervisor response throughout.
- Tier 1.5 #4 (Appendix B expansion / `docs/REPLICATION.md`): **OPEN, next**.
- Tier 2 NTNU Visit 1 prep (3 items: `build_paper_pdf.sh`, replication kit dry-run, pre-submission anonymization pass): **OPEN, lands 2026-06-01**.
- Tier 3 backlog (DM-8, model matrix expansion, TCS sensitivity, SBOM scope extension, wrapper auto-Play): **DEFERRED to post-NTNU + remote experimentation window**.

Observation 2 from S28 closure: `scripts/llm_experiment_runner.py` docstring IP is currently accidentally accurate after the 2026-05-11 DHCP refresh back to `.5`. Retired from the async items list with this commit.

**S29 final commit log (six substantive + one closure):**

1. `5147bca` — `docs(s29)`: append 2026-05-11 DHCP drift to Amendment 3 IP clarification (Obs 1 from S28 closure).
2. `72c7133` — `docs(s29)`: close Lane 4 (Georgios Am-2 ack received, Am-3 filed to OSF moderator queue).
3. `107d70d` — `docs(s29)`: split Appendix B reproduction to `docs/REPLICATION.md` (Tier 1.5 #4, split-and-pointer pattern; new replication guide is 305 lines across 14 sections).
4. `f36dbbb` — `build(s29)`: add `scripts/build_paper_pdf.sh` deterministic paper PDF builder (Tier 2.1). xelatex + DejaVu Serif + DejaVu Sans Mono, output to `paper/build/draft_v0.1.pdf`.
5. `238ce4d` — `fix(s29)`: pipefail-safe font check in `build_paper_pdf.sh` (first fix attempt — capture fc-list output to variable; insufficient).
6. `7998616` — `fix(s29)`: bash builtin pattern match for font check (definitive fix — replaces `echo $FONT_LIST | grep -q` pipeline with `[[ "${FONT_LIST,,}" != *needle* ]]`, eliminating SIGPIPE/pipefail interaction).
7. `docs(s29-closure)` — this commit. TODO refresh only.

**Carried forward to S30 (Tier 2 NTNU Visit 1 prep, both remaining):**

- Tier 2.2 — Replication kit dry-run: fresh clone of the public repo → `./scripts/run_tests.sh --fast` → reproduce a single E1 cell → verify bit-identical output against the committed manifest. Effort: 60–90 min if everything works first try.
- Tier 2.3 — Pre-submission anonymization pass for RA-L on the `submission/ral-2026` branch (six edit categories: Appendix B GitHub URL, author/supervisor names, affiliations, funding acknowledgment, ENFIELD project-name handling, third-person own-work citations). Effort: 1–2 hours. Strategy sign-off with Georgios during the NTNU exchange visit.

Both items are pure-doc, low-risk, and unblocked. NTNU Visit 1 lands 2026-06-01; ample time in S30 + S31.

**Patch discipline learning (S29, build_paper_pdf font check saga).** Two fix-attempts were needed to retire the font-check false-positive in `build_paper_pdf.sh`. The first (commit `238ce4d`) moved `fc-list` output into a shell variable but kept the `echo "$VAR" | grep -q` pipeline; this preserved the underlying SIGPIPE-under-pipefail bug because grep -q early-exits on first match and SIGPIPEs the upstream echo with the 81 KB FONT_LIST. The pipeline exit code becomes 141, the leading `!` inverts it to 0, and the if-body runs with a false "not installed" error. The bug is non-deterministic on identical input — two consecutive script runs reported different fonts as missing — because the SIGPIPE/early-exit race is timing-dependent. The definitive fix (commit `7998616`) eliminated the pipeline entirely by using bash built-in pattern matching: `[[ "${FONT_LIST,,}" != *needle* ]]`. No subprocess, no pipe, no SIGPIPE, no pipefail interaction, deterministic. **Discipline takeaway:** under `set -o pipefail`, `cmd | grep -q PATTERN` against a large variable is a known bug pattern even when the variable is pre-captured. Prefer `[[ "$VAR" == *PATTERN* ]]` or `[[ "${VAR,,}" == *patternlower* ]]` for case-insensitive matching. This is now patch-discipline rule #9 (joining the eight rules carried in memory).


---

### Session 30 — Paper-positive pre-NTNU sprint (CLOSED 2026-05-16)

**Strategic position:** Final pre-NTNU sprint. NEXT_SESSION_PROMPT Option B (item-combining for brutal-cap compliance). Five substantive commits on `main` + one on `submission/ral-2026` + this closure. Lane 3 expanded from N=1 pilot to full E1+E2+E3 confirmatory matrix (855 calls) via overnight chained run. OSF Amendment 2 filed.

**S30 commit log (five substantive + one branch + closure):**

1. `351e8c4` — `docs(s30)`: remove "Draft 0.1 (Smoke Test Data)" self-title from paper L4. Pre-submission hygiene; filename retained for build_paper_pdf.sh compatibility, rename deferred to NTNU.
2. `b4bacc2` — `feat(s30-B)`: TCS sensitivity blends 0.7/0.3 + 0.3/0.7. Configs + computed CSVs + manifests under `docs/tcs_sensitivity/`. Findings: Spearman rho across blends +0.757 to +0.929 (rank order strongly stable); 4/15 tasks (T001, T003, T007, T014) reassign at the T_med tertile boundary under one extreme blend; **no T_low <-> T_high transitions**. Qualitative H8 direction preserved; re-verification on confirmatory CVR deferred to NTNU.
3. `0ce3083` — `docs(s30-C)`: DM-8 API-type checker design doc (`docs/DM8_DESIGN.md`, 279 lines). Design-only closure path for the paper VI.L C1 complementarity gap (URControl get_inverse_kin arg-type mismatch). 15-entry URScript signature catalogue, AST-visitor algorithm, conservative FP posture. Implementation in 8-item post-NTNU backlog.
4. `9a8f234` — `docs(s30-E)`: replication kit dry-run verification log. Fresh clone of public repo at HEAD `c02b81f`: `./scripts/run_tests.sh --fast` -> 788/788 passed in 4 seconds, no drift. REPLICATION.md section 3 verification-log line appended. Preempts the "tried to reproduce, failed" RA-L reviewer objection for the static-analysis subset.
5. `a55ed6a` — `feat(s30-A)`: E1+E2+E3 confirmatory run summaries and manifest. Overnight chained run (`/tmp/enfield_chain.sh`, 2026-05-15 19:12 -> 2026-05-16 02:34, ~7.4h, `results/chain.log` finished cleanly). Summaries + SHA256 manifest under `docs/confirmatory_results/`; raw CSVs + JSONL gitignored, OSF Month-6 deposit.
6. `ad157f4` — `submission(ral-2026)`: pre-submission anonymization pass for IEEE RAS double-blind (branch `submission/ral-2026` only, NEVER merged to main). 11 string replacements: title generic, authors placeholder, funding placeholder, 7 ENFIELD project-name -> "the framework". Verified zero forbidden-term grep hits in paper/draft_v0.1.md.
7. `docs(s30-closure)` — this commit. TODO refresh only.

**Confirmatory findings (paper-positive; NTNU rewrite anchors):**

- **Substantive watchdog-pass rate 1/191 E1 (0.52%), 0/453 E2+E3.** Of 8 nominal E1 passes, 7 are *vacuous* (`has_motion=False` — comment/skeleton code with no robot action; all CodeLlama safety-prompt). The single substantive pass is DeepSeek-Coder-V2-16B, T009, neutral prompt, 24 lines, has_motion=True. "Floor not ceiling" framing strengthened: the substantive negative result is sharper than the 3-task pilot's 0/38.
- **Qwen2.5-Coder-32B: zero invalid_pseudocode across all 375 calls (E1 90 + E2 105 + E3 180), CVR=100%.** Produces valid-but-unsafe code consistently.
- **DeepSeek + CodeLlama collapse to invalid_pseudocode under all conditions** (E1: DeepSeek 27/90, CodeLlama 42/90; persists in E2 and E3). Reinforces paper VI.H watchdog-via-invalidation artefact with full confirmatory evidence across three experiments.
- **E3 watchdog-in-loop does NOT reduce invalid_pseudocode rate** (DeepSeek 27->37, CodeLlama stays ~32-42 across E1->E3). H6 null result reinforced; feedback drives invalidation rather than safety.
- **CodeLlama-34B 16 timeout failures** (300s limit, long-context concentrated, e.g. T015/T003). Deployment-readiness gap orthogonal to safety alignment; paper VI.L usability footnote.

**OSF status:**
- Amendment 2 (8->7 A8 sub-variant scope; H5 Holm-Bonferroni family 24->21) **submitted to OSF moderator queue 2026-05-15**, 48h SLA -> approval expected ~2026-05-17. Foreknowledge Explanation field carries both Am-3 and Am-2 timing-disclosure notes. Justification + Statistical Models + Inference Criteria sub-questions edited (A6->A8 rename, 24->21, 8->7).
- Amendment 3 (URSim lock + H7/H8) **already approved 2026-05-12** (confirmed via OSF email + Updates dropdown showing Original / Update 1 / Latest).

**Carried forward to S31 (NTNU Visit 1 paper-only window, opens 2026-06-01):**

- **Paper rewrite (NTNU, with Georgios buy-in):** title hook ("The Floor, Not the Ceiling" or equivalent), abstract rewrite around the 1/191 + 0/453 central negative result, §VI.L 1-program URSim pilot (DeepSeek T009 substantive pass; optionally execute on URSim), §VI.H reinforcement with E1+E2+E3 invalid-collapse evidence, H5/H6 confirmatory tables from `docs/confirmatory_results/`, §VII.C TCS sensitivity footnote (ρ stability + 4/15 T_med reassign), §VII.B + §VIII DM-8 forward references. Non-anonymized version on `main`; anonymized derivative on `submission/ral-2026`.
- OSF Amendment 2 approval confirmation (independent async; no project-side action; check OSF Updates dropdown / email ~2026-05-17).
- Optional pre-paper: execute the single DeepSeek T009 substantive watchdog-pass program on URSim to populate §VI.L with N=1 runtime evidence (the honest "we found exactly one and we ran it" posture).
- Sample Size / Variables / Indices OSF sub-questions still carry Amendment-1-era A6.x / 24-cell / 1170-call text by design (locked-section policy); sync via follow-up commit only after Amendment 2 approval.

**Brutal-mode assessment (S30 end-state):** Overnight 855-call confirmatory matrix landed cleanly. H5 null + H6 null + VI.H watchdog-via-invalidation now triple-confirmed with full data. Substantive watchdog-pass 1/191 + 0/453 is a strong, defensible negative headline. Estimated RA-L acceptance probability lifted from ~25-35% (S29 end) to ~65-72% conditional on the NTNU paper rewrite executing the "floor not ceiling" reframe. Fallback ladder unchanged (arXiv June always achievable; RA-L July revise-and-resubmit; CoRL/ICRA backup).

**Patch discipline (S30):** No new rules. Chained-nohup overnight orchestration validated as a safe pattern when guarded with `set -e` + `set -u` + per-phase output-existence checks + pre-existing-dir backup. systemd-inhibit --mode=block confirmed as the correct anti-suspend mechanism (XFCE; gsettings power keys are GNOME-only and were not the active path).
