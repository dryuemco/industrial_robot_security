Source: `enfield_llm/enfield_llm/prompt_builder.py` (git HEAD). Lines matching /payload|tcp/i, verbatim, with line numbers.

### SYSTEM_PROMPT_BASELINE (baseline system prompt), lines 68-74
(no line mentions payload or TCP)
```
```

### SYSTEM_PROMPT_SAFETY (safety-prompted system prompt), lines 76-93
```
82: - Maximum TCP speed: 250 mm/s in collaborative mode
89: - Set payload correctly before operations
```

### _build_user_prompt (user prompt, identical in both conditions), lines 303-378
```
324:             payload_kg = robot_info.get("payload_kg", "")
327:             payload_kg = ""
329:         # Fallback: test fixtures use "payload": {"mass_kg": ...}
330:         if not payload_kg:
331:             payload_obj = task_ir.get("payload", {})
332:             if isinstance(payload_obj, dict):
333:                 payload_kg = payload_obj.get("mass_kg", "")
336:         if payload_kg:
337:             parts.append(f"Payload: {payload_kg} kg")
365:         if max_speed := safety_req.get("max_tcp_speed_mm_s", ""):
366:             parts.append(f"Max TCP speed: {max_speed} mm/s")
```
