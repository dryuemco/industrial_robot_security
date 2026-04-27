# Sub-lane A Design: URScript Live Execution Pipeline

**Session:** S24 (NTNU Visit 1 demo prep)
**Status:** S24 — pipeline functional through telemetry capture; live joint motion blocked by URSim URCap dependency, deferred to NTNU on-site work (see "Known limitation" section below).
**Updated:** 2026-04-27 batch
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

## Known limitation: URSim URCap dependency

T001 smoke validates the URScript publish path end-to-end:

1. Translator emits a deterministic 59-line URScript program from the
   Task IR JSON.
2. The publisher node loads the URScript and publishes to
   `/urscript_interface/script_command` (`std_msgs/String`,
   1737 bytes for T001).
3. The `ur_robot_driver` subscribes and forwards the script to URSim.
4. The telemetry recorder captures `/joint_states` (~125 Hz) and
   `/tcp_pose_broadcaster/pose` (~500 Hz) for the configured duration.

In the S24 baseline configuration, URSim does not move when the
script arrives. Investigation (S24-04-27) traced the cause to a
Universal Robots driver workflow requirement: the driver delivers
URScript through the **External Control URCap**, which must be
installed inside PolyScope and the corresponding `external_control.urp`
program must be loaded and in the PLAYING state. Vanilla
`universalrobots/ursim_e-series:5.12` Docker image ships without this
URCap, and PolyScope does not auto-install URCaps from a mounted
`/ursim/.urcaps/` directory at boot. URCap installation appears to
require interaction with the PolyScope web UI rather than being
scriptable through the Dashboard interface.

Mount paths tested without success:

- `/tmp/.../urcaps:/urcaps`
- `/tmp/.../urcaps:/ursim/GUI/bundle/urcaps`
- `/tmp/.../urcaps:/ursim/.urcaps`

In each case `externalcontrol-1.0.5.urcap` (35287 bytes, sourced from
`/opt/ros/humble/share/ur_robot_driver/resources/`) was visible inside
the container but the Felix OSGi bundle cache showed no install
artifacts and no `external_control.urp` template was generated.

This is a known limitation of vanilla URSim and is **not specific to
ENFIELD**. UR's official `start_ursim.sh` (in
`ur_client_library` package) downloads the URCap from GitHub and
mounts it the same way; from inspection of the script there is no
explicit auto-install hook beyond the volume mount, suggesting that
production users complete URCap installation interactively through
the PolyScope UI.

### Resolution scope

Resolving this requires one of:

1. Switching to a custom URSim image that ships with the URCap
   pre-installed (loses the upstream digest pin and reproducibility
   guarantees in their current form).
2. Driving the PolyScope web UI programmatically (Selenium / VNC
   automation) to install the URCap and create the program — high
   complexity, fragile across PolyScope versions.
3. Using the UR Primary interface (port 30002) directly with full
   `def...end` programs and bypassing the URCap path. Tested in S24
   and observed to send the script but not start program execution
   without an active program slot, suggesting URSim e-Series 5.12
   requires PolyScope to be in a runnable program state regardless
   of script delivery channel.

Resolution is deferred to the NTNU exchange visit (2026-06-01 to
2026-07-31) where the URCap installation workflow can be solved
collaboratively with the host supervisor.

### Current S24 deliverable

What is delivered and reproducible at HEAD:

- The full software pipeline from Task IR through translator,
  publisher, driver, and telemetry recorder.
- A 59012-row CSV captured during a 60 s window confirming that
  `/joint_states` and `/tcp_pose_broadcaster/pose` are streamed and
  recorded with stable schema and matched QoS settings (the joints
  remain at the home pose throughout because URSim never enters
  PLAYING state, but the recorder behaviour is correct).
- All 8 offline tests pass on every push.
- An idempotent smoke wrapper that brings up URSim, the driver,
  and the telemetry pipeline reliably.

The paper Section V.G "Execution Setup" remains accurate as a setup
declaration; the URCap-blocked motion outcome is acknowledged as
future work.

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
