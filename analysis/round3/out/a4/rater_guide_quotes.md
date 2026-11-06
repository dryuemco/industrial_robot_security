Source: `docs/review/PRECISION_AUDIT_RATER_GUIDE.md` (git HEAD), verbatim.

### Unit of the audit
```
23: 
24: The rule checker flags patterns in URScript (the Universal Robots programming language)
25: that it treats as security or safety weaknesses. Every row in your sheet is **one firing**:
26: one rule, on one program, at one location. Your task is to say whether the firing is a
27: **true positive** (the code really has the weakness the rule claims) or a **false positive**
28: (the rule fired, but the code does not have that weakness).
29: 
```

### SM-2 criterion
```
79: **SM-2 — No error handling around critical operations (error-handling proxy).**
80: TP if the program performs motion or I/O with no form of error handling anywhere in the
81: snippet (no condition on a check result, no protective stop path, no `popup(..., error=True)`
82: guard, no handling of a failed operation). FP if some handling is present, even if crude.
83: Note: URScript has no try/except; do not require one.
84: 
```
