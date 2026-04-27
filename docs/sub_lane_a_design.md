# Sub-lane A Design: URScript Live Execution Pipeline

**Session:** S24 (NTNU Visit 1 demo prep)
**Status:** initial design; T001 single-task smoke validated; batch
harness deferred to S25 or NTNU on-site work.

## Architecture

```
Task IR (JSON)
      |
      v
IRToURScriptTranslator         (enfield_translators)
      |
      v
URScriptPublisherNode  -- publishes once -->  /urscript_interface/script_command
                                                       |
                                                       v
                                              ur_robot_driver
                                                       |
                                                       v
                                                URSim e-Series
                                                  (digest-pinned)
                                                       |
                                                       v
                            /joint_states  +  /tcp_pose_broadcaster/pose
                                                       |
                                                       v
                                          TelemetryRecorderNode --> CSV
```

## Design decisions

### Why a single-shot publisher (not a streaming RTDE client)
URScript-over-`/urscript_interface/script_command` gives us the full
program semantics including procedure declarations, control flow,
and `set_tcp` / `set_payload` calls - enough for end-to-end
adversarial testing without per-waypoint streaming. RTDE streaming
would tightly couple the framework to UR vendor SDKs and break the
robot-agnostic core principle.

### Why interleaved CSV (one row per message, two sources)
Joint states arrive at ~125 Hz; TCP pose at ~500 Hz. Locking these
to a synchronous timer would either drop samples or interpolate
fictitious data. Interleaved per-message rows preserve native
sampling rates and let post-hoc analysis reconstruct trajectories
without measurement artifacts. The `source` column distinguishes
rows; downstream tools filter by source.

### Why TRANSIENT_LOCAL on the TCP pose subscription
`tcp_pose_broadcaster` advertises with TRANSIENT_LOCAL durability.
A default RELIABLE/VOLATILE subscription will silently drop messages
due to QoS mismatch. The recorder explicitly mirrors the publisher
QoS to avoid this class of subtle failure.

## Batch execution harness (S25 scope)

For the 15-task suite, the wrapper extends as follows:

### Gate-passing subset selection
A task is eligible for live execution only if:
1. Translator produces non-empty URScript (no IR error)
2. Static watchdog DM-1..7 + SM-1..7 reports zero violations
   *or* a documented deliberate-violation row in the manifest
3. ISO clauses listed in `safety_requirements.iso_clauses` map to
   detection mechanisms in the watchdog

### Per-task timeout and recovery
Default 60 s per task; configurable via `DURATION_S` env var. On
PROTECTIVE_STOP detection (Dashboard `safetystatus` poll between
tasks) the harness:
1. Aborts the current task and records the stop event in CSV header
2. Sends Dashboard `stop` + `unlock protective stop`
3. Re-issues `power on` + `brake release`
4. Continues with the next task (configurable `--abort-on-stop`)

### Telemetry path naming
`results/ursim_telemetry/<task_id>_<YYYYMMDDTHHMMSS>.csv`
gitignored; manifest CSV
(`results/ursim_telemetry/manifest.csv`) tracks runs with
columns: task_id, started_utc, ended_utc, exit_code,
final_safetystatus, csv_path, urscript_path, sha256.

## Paper Section V.G extension hooks

A new subsection (Section V.H or VI.K extension) can describe live
execution outcomes with the table:

| Task | Exit | Duration | Safetystatus | Telemetry rows |
|------|------|----------|--------------|----------------|
| T001 | 0    | 32.4 s   | NORMAL       | 4128           |

This converts Section V.G from setup declaration to evidence-backed
outcome reporting - directly addressing the IEEE RA-L
reproducibility expectation.

## CI strategy

Live URSim CI is deferred (requires GPU-less x86_64 runner with
~4 GB VRAM-equivalent CPU memory and Docker socket access).
Offline tests for both nodes (publisher + recorder) run on every
push under the existing `colcon test` job.

## Reproducibility commitments (open-science)

- URSim image pinned by SHA-256 digest in the smoke wrapper:
  `universalrobots/ursim_e-series@sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51`
- All emitted URScripts are reproducible by re-running the
  translator on the same Task IR (translator is deterministic;
  no LLM in this path).
- Telemetry CSV format is stable and documented in the recorder
  module docstring.
- OSF Amendment 3 (pending Amendment 2 ack) will reference this
  document and the smoke wrapper as the URSim integration evidence.
