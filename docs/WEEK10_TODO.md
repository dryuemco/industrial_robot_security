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

- [ ] **#4 §VI Results placeholder iskeleti** — H4/H5/H6 için boş tablolar, model × condition matrisi, figure caption'ları. E1/E2/E3 verisi gelince doldurma kalır.
- [x] **#5 ISO 10218:2025 traceability matrix** — done in this commit. `docs/iso_10218_traceability.csv` (machine-readable) + paper §IV.D Table II. Clause titles marked PROVISIONAL — Yunus'un ISO doc kopyasından doğrulaması gerek.
- [ ] **#5b Runner end-to-end smoke run with mock LLM client** — *NEW, freeze öncesi şart.* Üç gizli bug Week 10'da peş peşe çıktı (parser.parse→extract, validity gate yokluğu, analyze_combined yanlış arg). Runner uçtan uca hiç koşmamış. Ollama bağımsız bir mock client (canned URScript döndüren) ile tüm kod yolunu en az bir kez geçir, başka bug var mı gör. Confirmatory başlamadan zorunlu.
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
