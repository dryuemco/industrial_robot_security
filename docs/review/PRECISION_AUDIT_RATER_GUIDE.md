# Precision Audit v2 — Guide for Raters

**Purpose.** Reviewers of manuscript Access-2026-32028 asked for a larger precision audit
of the rule checker with more than one rater and an inter-rater agreement statistic. This
guide tells a second, independent human rater exactly what to do. Rater A is a consensus
label agreed between two authors; rater B must not be an author of the manuscript and must
not have seen rater A's labels.

**Time.** About 2 hours for 100 items (roughly 1 minute per item once warmed up).

**What you need.**
- The labelling sheet `sheet_rater_<A|B>.csv` (one row per flagged firing).
- The code snippets folder `snippets/` (one `.txt` per row, named by `item_id`).
- This guide. No other project material. Please do not read the manuscript or the rule
  source code before labelling; the point is an independent judgement.

Both files are produced by `scripts/review/precision_audit_sample.py`; ask the corresponding
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

Tool-speed caps by operating mode, as defined in the task specifications: collaborative
0.25 m/s, hybrid 0.30 m/s, fenced 0.50 m/s. The task's operating mode is given in the
sheet (`operating_mode` column). Use the cap for that mode. The `description` column
always quotes the collaborative 0.25 m/s figure because the rule does not read the mode;
ignore that figure and apply the mode cap.

Acceleration units follow the same convention as speed: `movej` `a=` is a joint
acceleration in rad/s²; `movel`/`movec`/`movep` `a=` is a tool acceleration in m/s².
The rule uses one fixed acceleration threshold, 2.5 rad/s², regardless of mode, and
labels every `a=` literal in rad/s² even on Cartesian moves.

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
  (0.25 m/s collaborative, 0.30 m/s hybrid, 0.50 m/s fenced) → `TP`. The cap in the
  `description` column is always 0.25; use the mode cap instead.
- If it is `movej` with `v=` in rad/s → the rule has misread a joint speed as a tool speed.
  Label `FP` **unless** the joint speed is itself clearly excessive (above ~3 rad/s).
- If the literal is at or below the cap for the stated operating mode → `FP`.
- If the number is a named variable and not a literal → `FP`.
Write which of the four cases applied in `note` (e.g. "movej rad/s", "fenced mode 0.4 ok").

Acceleration firings ("Hardcoded acceleration … exceeds limit 2.5 rad/s²"):
- If it is `movej` with a literal `a=` above 2.5 rad/s² → `TP` (the unit reading is
  correct; the threshold does not depend on mode).
- If it is `movel`/`movec`/`movep`, `a=` is a tool acceleration in m/s² and the rule has
  misread the unit. Label `FP` **unless** the value is itself excessive for a tool
  acceleration: above 2.0 m/s², the highest tool-acceleration cap in the task set → `TP`.
- If the literal is at or below the applicable threshold, or is a named variable → `FP`.
Write which case applied in `note` (e.g. "movej rad/s²", "movel m/s² 3.6 > 2.0").

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
