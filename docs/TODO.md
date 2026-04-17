# ENFIELD Active TODO

**Authoritative forward-looking tracker.** Supersedes `WEEK10_TODO.md` and `WEEK11_SPRINT.md` (both in `docs/archive/`).

**Last updated:** Session 19 close (comprehensive archive scan), 2026-04-17.
**HEAD:** `d8fb0bb` on main.
**Tests:** 740 passing.
**Phase:** Paper editorial (data-complete; remaining is writing + figures).

---

## Next sessions

### Session 20 — Reviewer-critical paper integrations

Estimated ~3 hours. Drop-in sentences drafted in `docs/literature_synthesis_ra_l.md` section 4.

- [ ] §II.A / §II.B / §II.C Related Work expansion → paste from synthesis §4.3 (closes W10 #10)
- [ ] §VII.B.9 static-analyzer-recall sub-subsection → paste from synthesis §4.7 (reviewer-critical)
- [ ] §V methods footnote on H6 threshold rationale → paste from synthesis §4.4
- [ ] Add Kumar 2024 COLM paper to `docs/literature_review.xlsx` Papers sheet (identity resolved via `literature_notes.md` ref [2]: "Certifying LLM Safety against Adversarial Prompting", arXiv:2309.02705, Kumar A., Agarwal C., Srinivas S., Li A.J., Feizi S., Lakkaraju H.)

Discipline: grep paper for current §VII.B sub-subsection count before adding §VII.B.9; replace `[CITE:paper_id]` placeholders with BibTeX keys.

### Session 21 — H7/H8 integration + paper opening/closing

Estimated ~3 hours.

- [ ] §VI.K exploratory hypothesis subsection (H7 ceiling, H8 refusal) → paste from synthesis §4.5
- [ ] Abstract ISO 2025 cybersecurity framing → paste from synthesis §4.1
- [ ] §I Introduction reframe → paste from synthesis §4.2
- [ ] §VIII Conclusion closing → paste from synthesis §4.8

### Session 22 — Figures part 1

Estimated ~3 hours.

- [ ] Architecture diagram (Mermaid or TikZ)
- [ ] Per-model violation rate chart (source: `results/stats_e3_full/cochran_results.csv`, 4 rows × 3 models = 12 data points)
- [ ] T002 trajectory oscillation visualization (15→4→15→4 byte-identical two-state oscillation from §VI.H)

### Session 23 — Polish + cross-ref audit + minor fixes

Estimated ~3 hours. Collects low-priority stylistic and hygiene items.

- [ ] Second-pass figure refinement
- [ ] Table audit (Table II ISO mapping verification, Table III-naive caveat, Table VI refusal per-rule)
- [ ] Reference consistency sweep (BibTeX keys match cite commands)
- [ ] Paper-internal cross-reference audit (§ numbers, figure numbers, H4-H8 references all consistent)
- [ ] Paper §V.E H6 "alongside... and" parse ambiguity fix (W10 #13, low-priority stylistic)
- [ ] Paper line ~291 "Zero refusals across all models and conditions" FIXME — verify against §VI.I refusal data + Table VI from Session 16 (likely resolved by commit `20dbe9a`; confirm and either update prose or strip FIXME marker)
- [ ] `THREAT_MODEL.md` line ~91 A8 range drift check (A8.1-A8.6 vs A8.1-A8.8 — align with paper Table I after Session 8 #15 fix)
- [ ] Fix `tests/test_mcnemar_analysis.py::TestH5::test_large_effect_significant` "if a66:" guard test hygiene weakness (W10 #14) — assert match exists instead of silently passing on no-match

### Session 24 — Georgios demo deck prep

Estimated ~3 hours. Closes W10 #9. For NTNU Visit 1 (May 11-25).

- [ ] Deck structure: problem → ISO 10218-1:2025 cybersecurity framing → methodology → H4-H6 confirmatory + H7-H8 exploratory → next steps
- [ ] Reuse Q&A skeleton from archived `georgios_week10_demo.md` (11 questions; update post-E1 answers)
- [ ] Add new literature synthesis findings (H7/H8 framing, FDSP comparison, static-analyzer threat)
- [ ] Dry run with Yunus before sending Georgios

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
| 2026-05-11 to 2026-05-25 | NTNU Visit 1 | Yunus in Trondheim. Demo deck + Georgios paper review. |
| 2026-06-08 to 2026-07-31 | NTNU extended visit | ~8 week stay. Main writing + submission work. |
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
