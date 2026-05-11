# Responsible Disclosure — ENFIELD Project

**Status:** Draft stub (v0.1, Session 28, 2026-05-11). Subject to revision pending host-supervisor sign-off and operational refinement before the Month-6 OSF release. The 90-day vendor advance-notice commitment in §3 and the tiered access framework in §§1–2 are the operative commitments of this document; specific contact channels (project alias, PGP), Tier 2 verification workflow templates, and the vendor contact directory will be finalised in v1.0.

**Project:** ENFIELD — Formal Adversarial Testing of LLM-Generated Code for Industrial Robots
**Funding:** European Union Horizon Europe Research and Innovation Programme, Grant Agreement No. 101120657
**Repository:** https://github.com/dryuemco/industrial_robot_security
**Pre-registration:** OSF DOI [10.17605/OSF.IO/VE5M2](https://doi.org/10.17605/OSF.IO/VE5M2)

---

## Why this policy exists

ENFIELD produces three classes of artefact:

1. **Scientific outputs** — the paper, the aggregated experimental results, the detection-mechanism descriptions, the statistical analyses, the pre-registration. These have no dual-use sensitivity and are openly available.
2. **Adversarial test artefacts** — the A8 adversarial prompt family targeting LLM-generated industrial-robot control code, the per-task confirmatory-run outputs in `results/`, and detailed bypass-class descriptions. These are intended for security research, but a complete enumeration of working attack instances against named LLMs and named robot controllers can also serve as a ready-made offensive corpus.
3. **Defensive artefacts** — the AST-based static watchdog (`enfield_watchdog_static`) with its DM-1..7 and SM-1..7 rules, the runtime monitoring components, and the validation harness. These are defensive and openly available.

Class 2 artefacts are the reason this policy exists. We balance open-science replicability (a core funder commitment under Horizon Europe and an explicit reviewer expectation for IEEE RA-L) against the risk that the most concentrated, ready-to-run offensive content could be misused. The three-tier access model below makes Tier 1 the default and gates only the highest-leverage offensive content behind a lightweight verification step.

---

## §1. Tier 1 — Public (default)

Everything in the public ENFIELD GitHub repository is Tier 1. This currently includes:

- The paper draft (`paper/draft_v0.1.md`) and pre-registration (`docs/OSF_PREREGISTRATION.md`, OSF Amendments 1–3).
- The threat-model document (`docs/THREAT_MODEL.md`) describing the A1–A8 attack categories at a methodological level, including the A8 sub-taxonomy.
- All detection-rule definitions and unit tests in `enfield_watchdog_static/` (DM-1..7, SM-1..7).
- All baseline Task IRs in `enfield_tasks/` and the IR-to-URScript translator in `enfield_translators/`.
- The adversarial prompt-template library in `enfield_attacks/` and its unit tests. The library encodes the *family* of adversarial perturbations; the *per-task per-condition prompt strings as run* and the corresponding LLM responses are Tier 2 (see §2).
- Aggregated experimental results (CVR, SVR, count statistics, per-model breakdowns) as reported in paper §VI.
- The replication kit metadata: `docs/replication/MODEL_DIGESTS.txt`, the CI workflow files (`.github/workflows/`), the SBOM scope documentation in `docs/open_science_release.md`, and the Docker setup in `docker/ursim/`.

No registration, no contact, no waiting. Clone, read, replicate.

---

## §2. Tier 2 — Verified researcher access

A small set of artefacts is held back from the public tier:

- The complete per-task adversarial prompt corpus *as run during the E1, E2, and E3 confirmatory experiments* (the concrete prompt strings, including any model- or task-specific variation introduced by the prompt builder), together with the corresponding LLM-output corpus. The paper reports aggregate statistics over these; the raw text is currently held outside the public repository.
- Future variant-IR exploit artefacts as standalone reproducible scripts, should the project produce them in the future-work scope defined in paper §VII.B.8.

Tier 2 is forward-looking as well as descriptive: the framework is in place for future artefacts that may warrant gating, not only for those currently assigned.

**How to request Tier 2 access:**

1. The request comes from an institutional email address (`*.edu`, `*.ac.*`, recognised national-lab domains, or equivalent verifiable affiliation).
2. The request includes a one-paragraph statement of research purpose. We do not vet the scientific merit of the proposed work; we ask only that the purpose be a research purpose.
3. The requester acknowledges that they will not redistribute the Tier 2 artefacts outside their named research group without separate authorisation.

We aim to respond to Tier 2 requests within 14 working days. Refusals are at our discretion; we maintain a record of refusal reasons. There is no appeal process.

---

## §3. Tier 3 — Vendor pre-notification

When the project identifies a class of issue that materially affects a specific upstream vendor's product — for example, a robust adversarial prompt that consistently elicits unsafe URScript from a specific Ollama-distributed model, or an evasion of a specific industrial-robot-controller execution gate — we will privately notify the affected vendor before public release.

**Vendor scope (current):**

- LLM providers: Alibaba Cloud (Qwen family), DeepSeek AI (DeepSeek-Coder family), Meta AI (CodeLlama family), Ollama Inc. (distribution layer).
- Industrial robot vendors: Universal Robots (URSim, PolyScope, URCap ecosystem). The scope will extend to ABB and KUKA when the project's future-work timeline reaches RAPID and KRL.

**90-day advance-notice timeline:**

- **Day 0** — Vendor receives written notification with (a) a description of the issue at a level sufficient for the vendor to reproduce it, (b) the affected product or model version and the conditions under which the issue was observed, (c) a proposed public-release date no earlier than Day 90, and (d) project contact details for technical questions.
- **Days 1–60** — Vendor's primary window for triage, fix development, and patch deployment.
- **Days 60–90** — Coordination window: vendor and project agree on synchronisation of public-release messaging, CVE assignment (if applicable), and any required redaction of demonstration code. The vendor may request a release-date extension; we will consider such requests in good faith but may decline.
- **Day 90** — Public release proceeds as planned unless the vendor and we have explicitly agreed on a later date.

**Vendor request channel:** vendors who wish to be added to the notification list, or who wish to register a specific product version for advance consideration, may contact the project as described in §4.

---

## §4. How to report a security issue

If you have identified a security issue **in the ENFIELD codebase itself** — for example, a vulnerability in the watchdog evaluation pipeline, the experiment runner, the URSim Docker integration, or the CI/CD workflows — please contact us privately rather than opening a public GitHub issue.

- **Email:** yunusemrecogurcu@gmail.com (project lead). Subject line prefix: `[ENFIELD security]`.
- **PGP:** to be published before the Month-6 OSF release; until then, please contact via email and we will arrange an encrypted channel if the report contents warrant it.

We aim to acknowledge security reports within 5 working days and to provide an initial triage response within 14 working days.

Reports of issues in **upstream LLM products** (Qwen, DeepSeek-Coder, CodeLlama) or in **upstream simulator / controller products** (URSim, PolyScope, URCaps) should be sent to the respective vendor's security channel directly. The ENFIELD project is happy to act as a technical relay if the report concerns a specific bypass observed through our framework and the reporter wishes us to facilitate the vendor handoff.

---

## §5. Out of scope

The following are explicitly out of scope for this disclosure policy:

- Reports describing **how the documented attack categories work**. The A1–A8 taxonomy is an intentional research output documented in the threat model and the paper, not a vulnerability in the ENFIELD framework.
- Reports about LLM **hallucination, factual error, low quality of generated code, or stylistic preference**. These are model-behaviour observations, not security issues, and they belong with the upstream LLM provider's feedback channel rather than this policy.
- Performance issues, non-security bugs, build failures, or feature requests — please use public GitHub issues.

---

## §6. Revision policy

This document is a stub. It will be revised before the Month-6 OSF public release to add:

1. A project-alias contact email (not the project lead's personal address) and a published PGP key.
2. A concrete Tier 2 verification workflow: request form template, response service-level commitment, accept/refuse criteria record-keeping policy.
3. A vendor contact directory: specific security-team email addresses for each vendor in §3 scope.
4. Host-supervisor sign-off on the 90-day timeline and on the refusal-record retention policy.
5. Reconciliation with NTNU institutional disclosure policy (host institution during the project exchange visits).

Until v1.0 is filed, this stub represents the project's operative commitments to a tiered access model and to coordinated vendor disclosure.

---

*Document version: v0.1 (S28 stub, 2026-05-11).*
*Authoritative location:* `docs/responsible_disclosure.md` *in the ENFIELD GitHub repository.*
