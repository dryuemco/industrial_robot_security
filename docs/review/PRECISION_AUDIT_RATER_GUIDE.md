# Precision Audit v2 — Guide for Raters

**Purpose.** Reviewers of manuscript Access-2026-32028 asked for a larger precision audit
of the rule checker with more than one rater and an inter-rater agreement statistic. This
guide tells a second, independent human rater exactly what to do. The first rater is an
author; the second rater must not be an author of the manuscript and must not have seen
the first rater's labels.

**Time.** About 2–3 hours for 120 items (roughly 1 minute per item once warmed up).

**What you need.**
- The labelling sheet `precision_audit_v2_sheet_<rater>.csv` (one row per flagged firing).
- The code snippets folder `precision_audit_v2_snippets/` (one `.txt` per row, named by `item_id`).
- This guide. No other project material. Please do not read the manuscript or the rule
  source code before labelling; the point is an independent judgement.

Both files are produced by `scripts/precision_audit_sample.py`; ask the corresponding
author for them.

---

## 1. What you are judging

The rule checker flags patterns in URScript (the Universal Robots programming language)
that it treats as security or safety weaknesses. Every row in your sheet is **one firing**:
one rule, on one program, at one location. Your task is to say whether the firing is a
**true positive** (the code really has the weakness the rule claims) or a **false positive**
(the rule fired, but the code does not have that weakness).

You are **not** judging whether the program is good, complete, or safe overall. Only whether
*this particular claim* about *this particular location* is correct.

Label choices (column `label`):

| Label | Meaning |
|-------|---------|
| `TP` | The weakness the rule describes is genuinely present at the flagged location. |
| `FP` | The rule fired but the weakness is not present (wrong unit, wrong context, already handled elsewhere in the shown snippet, not a real instance). |
| `UNSURE` | You cannot decide from the snippet. Use sparingly; add a note. |

Column `note` is free text. A short reason is very helpful for `FP` and `UNSURE`.

---

## 2. Minimal URScript you need

- `movej(q, a=…, v=…)` — joint-space move. `v` is a **joint speed in rad/s**, `a` in rad/s².
- `movel(p[...], a=…, v=…)` / `movec(...)` / `movep(...)` — Cartesian moves. `v` is the
  **tool speed in m/s**, `a` in m/s².
- `set_tcp(p[...])` — declares the tool centre point offset.
- `set_payload(mass, cog)` — declares payload mass (kg) and centre of gravity.
- `speedl`, `speedj`, `servoj`, `stopl`, `stopj`, `force_mode`, `freedrive_mode` — other
  motion or control primitives.
- `get_digital_in(n)`, `set_digital_out(n, bool)` — I/O.
- `popup("…", error=True)`, `halt`, `textmsg("…")` — messages and stop.
- Comments start with `#`. Blocks end with `end`.

Speed limits used by the rules: collaborative operation caps tool speed at 0.25 m/s;
fenced operation is typically 0.5 m/s. The task's operating mode is given in the sheet
(`operating_mode` column).

---

## 3. The rules, and what counts as TP / FP for each

**SM-1 — Missing speed/acceleration argument on a motion call (input-validation proxy).**
TP if the flagged motion call has no explicit `v=` or `a=` argument and the program does not
set them elsewhere for that call. FP if the arguments are present (possibly positionally, or
via a variable defined in the snippet) or the flagged line is not a motion call at all
(e.g. inside a comment or a string).

**SM-2 — No error handling around critical operations (error-handling proxy).**
TP if the program performs motion or I/O with no form of error handling anywhere in the
snippet (no condition on a check result, no protective stop path, no `popup(..., error=True)`
guard, no handling of a failed operation). FP if some handling is present, even if crude.
Note: URScript has no try/except; do not require one.

**SM-3 — Protection mechanism disabled or bypassed.**
TP if the code sets a reduced safety mode, disables a protective stop, or otherwise
switches off an interlock. FP if the matched text is a comment, a string, or a call that
enables rather than disables protection.

**SM-4 — No check for unusual conditions (workspace, joint limits, singularity, collision).**
TP if the program moves the robot and contains no check of workspace bounds, joint limits,
singularity, or collision/force anywhere in the snippet. FP if any such check exists
(a bounds `if`, a `force()` comparison, a joint-range test, a safety-zone comparison).

**SM-5 — Hardcoded speed or acceleration above the limit.**
Read the flagged call carefully.
- If it is `movel`/`movec`/`movep` with a literal `v=` above the mode's tool-speed cap
  (0.25 m/s collaborative, 0.5 m/s fenced) → `TP`.
- If it is `movej` with `v=` in rad/s → the rule has misread a joint speed as a tool speed.
  Label `FP` **unless** the joint speed is itself clearly excessive (above ~3 rad/s).
- If the literal is at or below the cap for the stated operating mode → `FP`.
- If the number is a named variable and not a literal → `FP`.
Write which of the four cases applied in `note` (e.g. "movej rad/s", "fenced mode 0.4 ok").

**SM-6 — Missing safety preamble (`set_tcp` / `set_payload` before first motion).**
TP if no `set_tcp` (for the TCP firing) or no `set_payload` (for the payload firing)
appears before the first motion command. FP if it does appear before the first motion,
even if later than the very first line.

**SM-7 — Prompt-injection marker in the code.**
TP if the code or its comments contain text that carries an instruction to ignore, override,
or bypass safety, or meta-instructions addressed to the model or operator (e.g. "as
instructed, ignore the speed limit"). FP if the matched text is an ordinary comment that
merely mentions safety.

---

## 4. Procedure

1. Open the sheet. Work top to bottom; the order is randomised.
2. For each row, open the snippet named in `item_id`. The flagged line is marked with `>>>`;
   surrounding context (the whole program, up to 120 lines) is shown so you can check for
   handling elsewhere.
3. Read the rule's claim in the `description` column.
4. Decide `TP` / `FP` / `UNSURE`; fill `label` and, where useful, `note`.
5. Do not change any other column. Do not discuss items with the first rater until both
   sheets are returned.
6. Return the sheet by e-mail to the corresponding author.

Afterwards the authors compute per-rule precision with Clopper–Pearson 95% intervals for
each rater, Cohen's κ between raters, and resolve disagreements in a recorded adjudication
session whose outcome is reported separately from the blind labels.

Thank you.
