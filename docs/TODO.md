# ENFIELD Active TODO

**Authoritative forward-looking tracker.** Supersedes `WEEK10_TODO.md` and `WEEK11_SPRINT.md` (both moved to `docs/archive/`).

**Last updated:** Session 19 close, 2026-04-17.
**HEAD:** `0609f5b` on main.
**Tests:** 740 passing.
**Phase:** Paper editorial (data-complete; remaining is writing + figures).

---

## Next sessions

### Session 20 — Reviewer-critical paper integrations

Estimated ~3 hours. All drop-in sentences already drafted in `docs/literature_synthesis_ra_l.md` section 4.

- [ ] §II.A / §II.B / §II.C Related Work expansion → paste from synthesis §4.3 (closes W10 #10)
- [ ] §VII.B.9 static-analyzer-recall sub-subsection → paste from synthesis §4.7 (reviewer-critical)
- [ ] §V methods footnote on H6 threshold rationale → paste from synthesis §4.4

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
- [ ] T002 trajectory oscillation visualization (15→4→15→4 byte-identical two-state oscillation case study from §VI.H)

### Session 23 — Figures part 2+ polish

Estimated ~2 hours.

- [ ] Second-pass figure refinement with feedback
- [ ] Table audit (Table II ISO mapping verification, Table III-naive caveat, Table VI refusal per-rule)
- [ ] Reference consistency sweep (BibTeX keys match cite commands)
- [ ] Paper-internal cross-reference audit (§ numbers, figure numbers, H4-H8 references all consistent)
- [ ] H-numbering drift audit in `scripts/mcnemar_analysis.py` (`run_h6` → paper-level H4, bounded mechanical cleanup; closes W10 #12)

### Session 24 — Georgios demo deck prep

Estimated ~3 hours. Closes W10 #9.

- [ ] Deck for NTNU Visit 1 (May 11-25 2026)
- [ ] Structure: problem → ISO 10218-1:2025 cybersecurity framing → methodology → H4-H6 confirmatory + H7-H8 exploratory → next steps
- [ ] Dry run with Yunus before sending Georgios

---

## Async items (no session gate, progress whenever)

- [ ] Resolve Kumar COLM 2024 reference identification (synthesis §7 known gap)
- [ ] Resolve INSEC ICML 2025 reference identification (synthesis §7 known gap)
- [ ] EU AI Act Article 15 literature row if needed for §VII.A
- [ ] OSF Amendment 2 (candidate `d251c15`) Georgios acknowledgment nudge
- [ ] OSF Amendment 3 (H7 + H8 exploratory) — draft scope in synthesis §5, submit after Amendment 2 clears
- [ ] Task IR review (W10 #11)
- [ ] CodeLlama T012 timeout root cause — currently logged as §VII.B limitation, deferred for follow-up paper
- [ ] ISO clause titles verification — 7 PROVISIONAL entries in `iso_10218_traceability.csv` need cross-check against Yunus's physical ISO 10218-1:2025 document before Table II is final

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

Items completed or superseded and moved to `docs/archive/`:

- `WEEK10_TODO.md` — all W10 items either completed or migrated above
- `WEEK11_SPRINT.md` — Sessions 8–19 closure blocks preserved
- `attack_definitions_A1_A4.md` + `attack_definitions_A5_A8.md` — superseded by `THREAT_MODEL.md` + A8.1-A8.7 taxonomy drift
- `brief_2026-04-16_h5_finding.md` — superseded by `literature_synthesis_ra_l.md`
- `E3_DESIGN.md` — E3 experiments complete
- `georgios_week10_demo.md` — will be superseded by Session 24 demo deck
- `h_numbering_audit_2026_04_11.md` — audit completed (per memory)
- `literature_notes.md` — superseded by `literature_review.xlsx` + `literature_synthesis_ra_l.md`
- `revised_plan_v2.md` — plan evolution captured in git log and synthesis

---

## Discipline active

Patch discipline (8 rules in memory #6), commit directly to main, ASCII commit messages, locked artefacts list in synthesis §3. Runner mandatory gate for any confirmatory experiment (`tests/test_runner_mock_smoke.py`). See `SESSION_20_HANDOFF_PROMPT.md` for full context if starting new session.
