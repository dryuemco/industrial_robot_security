# ENFIELD docs archive

Historical documents from earlier project phases, preserved via `git mv` so commit history is retained. These files are not actively maintained. For current status see `docs/TODO.md`.

## Contents

| File | Archived at | Reason |
|---|---|---|
| `WEEK10_TODO.md` | Session 19 close | All items completed or migrated to `docs/TODO.md` |
| `WEEK11_SPRINT.md` | Session 19 close | Sessions 8–19 closure blocks; session closure now tracked via git commit history instead |
| `attack_definitions_A1_A4.md` | Session 19 close | Superseded by `docs/THREAT_MODEL.md` (Apr 14) + A8.1-A8.7 taxonomy drift |
| `attack_definitions_A5_A8.md` | Session 19 close | Same as above |
| `brief_2026-04-16_h5_finding.md` | Session 19 close | Superseded by `docs/literature_synthesis_ra_l.md` which has deeper literature-backed analysis |
| `E3_DESIGN.md` | Session 19 close | E3 experiments complete; design doc is historical |
| `georgios_week10_demo.md` | Session 19 close | Will be superseded by Session 24 demo deck for NTNU Visit 1 |
| `h_numbering_audit_2026_04_11.md` | Session 19 close | Audit task completed |
| `literature_notes.md` | Session 19 close | Superseded by `docs/literature_review.xlsx` + `docs/literature_synthesis_ra_l.md` |
| `revised_plan_v2.md` | Session 19 close | Plan evolution captured in git log and synthesis document |

## If you need to recover

These files are preserved in place under `docs/archive/`. To see earlier state of any file, use `git log --follow docs/archive/FILENAME` to trace it back to its original path. To revive a file as active, move it back: `git mv docs/archive/FILENAME docs/FILENAME`.
