# ENFIELD Week 10 — LLM-Server-Independent Backlog

> Local working TODO. LLM inference server (Ollama @ 192.168.1.6:11434)
> şu an bağımsız çalıştığında E1/E2/E3 başlatılacak. Bu liste sunucu
> yokken yapılabilecek tüm işleri ve durumlarını izler.
> **Update bu dosyayı her commit'te.** Bitmiş işleri silme — `[x]` + commit hash bırak.

## Bu hafta tamamlanan

- [x] **Paper draft v0.1: H4–H6 + sensitivity + validity gate framing** — 4 commit (df91f6d, d4ad66d, 378912e, ca873c6)
- [x] **OSF Amendment 1 APPROVED cascade** — ade5190
- [x] **Runner kırık `parser.parse()` → `extract()` fix** — a4fdacd (smoke_test farklı yol kullanıyordu, runner hiç koşulmamıştı)
- [x] **URScript validity gate (kod + 5 test + runner branch)** — 068bb0a, cascade 5dead70
- [x] **mcnemar_analysis.py synthetic-data testleri** — already existed (TestH4/H5/H6 + TestEdgeCases, 45 test)
- [x] **mcnemar markdown auto-fill** — already existed (`generate_markdown_report`)
- [x] **Runner `analyze_combined` arg bug fix** — 92c0d5c. Runner kodu URScript'i `task` arg olarak geçiyordu + var olmayan `task_id=` kwarg. Üçüncü gizli runner bug'ı (önceki ikisi: parser.parse→extract a4fdacd, validity gate yokluğu 068bb0a).
- [x] **Paper §IV.C DM list + validity-gate scope düzeltmesi** — dd754dc. 6/7 DM yanlış etiketlenmiş; validity gate yanlışlıkla DM+SM diyordu, gerçekte sadece SM.

## Sıradakiler (öncelik sırasıyla)

- [x] **#4 §VI Results placeholder iskeleti** — done 84628ef. §VI.F (H4 baseline CVR), §VI.G (H5 adversarial uplift), §VI.H (H6 watchdog-in-loop McNemar), §VI.I (sensitivity), §VI.J (Cochran's Q). Tüm hücreler '—' placeholder, mcnemar_analysis.py dolduracak.
- [x] **#5 ISO 10218-1:2025 traceability matrix** — verified against published ISO text and propagated across the repo. Code: c44e3a5 (watchdog rule clause literals), 9f916b3 (attack mutation classes), 83130f4 (rapid_parser + document retention). Docs: ec49fa2 (traceability CSV, paper §III Table I + §IV.C semantic clarification + §IV.D Table II, README, THREAT_MODEL, OSF_PREREGISTRATION, iso_clause_mapping, attack_definitions_*, UR5e YAML configs, SM-5 comment). Key findings: (1) clause 5.12 does not exist in the 2025 revision — previous '5.12.3 Safeguarded space' mapping was a hallucinated placeholder, corrected to 5.7.4 Software-based limiting; (2) clause 5.6 is Simultaneous motion, not speed limiting — speed limits live under 5.5.3 Speed limit(s) monitoring; (3) clause 5.1.16 Cybersecurity, newly introduced in 2025, anchors A8 and SM-1/SM-3/SM-5/SM-7 via NOTE 1 (first known static-analysis framework to anchor LLM-generated robot-code weaknesses to 5.1.16); (4) SM-2 (CWE-252) and SM-4 (CWE-754) have no direct ISO anchor and are flagged CWE-only with an em-dash — to be framed in §VII as "standard extension opportunity"; (5) paper §III taxonomy drift also fixed (A2–A8 labels were misaligned between §III and §IV.D). 19 rule instances, 7 distinct clauses, 4 top-level sections (5.1, 5.4, 5.5, 5.7). Deferred follow-ups: prose cleanup in `attack_definitions_A5_A8.md:~611` (says ISO does not address LLM code generation — 5.1.16 now partially does); `THREAT_MODEL.md:~91` A8.1–A8.6 range should be A8.1–A8.8; code-level A6.* → A8.* rename in llm_experiment_runner / prompt builder / mutation modules (paper uses A8.* throughout, footnoted).
- [x] **#5b Runner end-to-end smoke run with mock LLM client** — DONE. Mandatory green gate for confirmatory E1/E2/E3 runs is in place. Five-commit epic: dde5eb3 (MockLLMClient + 21 unit tests), ef8ec7c (factory mock branch + 12 factory tests), 1c6e9bb (runner --provider / --mock-seed CLI), b6ac8f2 (fourth hidden runner bug fix: response.content → response.raw_response), 518a76d (end-to-end smoke suite tests/test_runner_mock_smoke.py, 11 tests). Repo total 624 → 671. Four hidden runner bugs total have now been caught by static review + mock runs before any live execution: (1) parser.parse vs .extract (a4fdacd, Week 10 early), (2) missing URScript validity gate (068bb0a, Week 10), (3) analyze_combined argument order (92c0d5c, Week 10), (4) response.content vs .raw_response (b6ac8f2, Week 10 late, caught during PR-4 preparation for this item). The smoke suite's five code-path coverage tests act as hard regression guards for bugs #2 and #3: if the validity gate ever stops firing on the PSEUDOCODE mock template, or if analyze_combined is miscalled again, the suite fails and blocks any confirmatory run. Deterministic rotation uses seed=42 by default and is documented in mock_client.py for paper and preregistration reference. Deferred: refusal-detector URScript gevşekliği (base_client.py has_code check'i movej substring'ini yakalıyor ama URScript için dar) — ayrı audit item; code-level A6.* → A8.* rename hâlâ bekliyor (paper §III.C footnote'unda kayıtlı).
- [ ] **#6 Refusal classifier freeze + test** — prereg "deterministic refusal classifier" diyor; kuralları (regex, "I cannot", "as an AI", boş code block) bir yerde donmuş + test coverage olmalı. Confirmatory başlamadan şart.
- [ ] **#7 Cochran's Q implementation** — H6 cross-model exploratory için. mcnemar_analysis.py içine küçük fonksiyon + test.
- [ ] **#8 §VII Threats to Validity subsection** — Q4 quantization, single-robot (UR5e), simulation-only, temperature=0 non-determinism, validity-gate sınırları. Reviewer'ı önden silahsızlandırır.
- [ ] **#9 Georgios Week 10 demo deck** — 5–6 slide: Security ✓, LLM integration ✓, Amendment 1 APPROVED ✓, paper draft v0.1, validity gate keşfi.
- [ ] **#10 Related Work §II'yi genişlet** — RoboBench, RoboPAIR, vb. literatür taraması.
- [ ] **#11 15 task IR review** — Pilot calibration bias kontrolü: 10 non-pilot task'ın gerçekten freeze sonrası yazıldığını git log ile doğrula.

## Sonra (LLM server geldiğinde)

- [ ] E1 pilot run: 3 model × 5 task × 2 condition × 1 rep = 30 çağrı (pipeline doğrulama)
- [ ] E1 full: 270 çağrı (H4)
- [ ] E2 full: 360 çağrı (H5)
- [ ] E3 full: ~540 çağrı (H6)
- [ ] mcnemar_analysis.py'ı gerçek veriyle çalıştır + paper §VI'yı doldur

## Notlar

- **Rule freeze:** Validity gate (068bb0a) artık donmuş set'te. Confirmatory başladıktan sonra dokunulmaz.
- **Runner kırık keşfi:** smoke_test_llm.py runner'dan ayrı yol kullanıyor (extract direkt çağırıyor), bu yüzden runner'ın AttributeError'ı 9-call smoke testte yakalanmamıştı. Ders: confirmatory öncesi runner'ı ayrı bir mock-LLM ile uçtan uca koşturmak lazım.
