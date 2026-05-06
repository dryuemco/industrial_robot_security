# Lane 3 Faz B-(c) — Live Execution Results (2026-05-06)

## Setup
- URSim e-Series 5.12 (image enfield-ursim:5.12-urcap-1.0.5, container 172.17.0.2)
- URCap external_control 1.0.5 (FZI Research Center for Information Technology)
- Host IP 172.17.0.1:50002, Local Control mode (Remote Control NOT used)
- Manifest digest: b92d6a0b... (qwen2.5-coder:32b Q4_K_M, temp=0.0, deterministic)
- ROS2 driver: ur_robot_driver, ur5e profile

## Workflow established
1. URSim container start (default bridge network)
2. URCap External Control install via PolyScope GUI (Settings → System → URCaps → +)
3. Container restart (URCap Active state persists in /ursim/.urcaps/)
4. Robot init: power on + brake release (Dashboard)
5. PolyScope: External Control program node in tree, Host IP 172.17.0.1:50002
6. Wrapper start → race window → Play in PolyScope GUI
7. Telemetry recorded for 60s

## Findings — 3 distinct pre-execution failure classes

### T001 (simple_pick_place) — Class 1: Syntactic-OK semantic API misuse
- Generated: `movej(get_inverse_kin([0,-1.57,1.57,-1.57,-1.57,0]), a=1.2, v=1.05)`
- URControl error: `Runtime error (3:11): get_inverse_kin: Parameter must be a Pose on the form p[x,y,z,a,b,c]`
- Reproduced 3x at 21:41:47, 21:46:07, 21:48:09 — fully deterministic
- Static watchdog (DM-1..7) coverage gap: API-type checks not implemented
- Joint range: 0 (no motion)

### T004 (multi_waypoint_ssm_pick_place) — Class 2: Timing/state-coupling failure
- Race window mismatch: URCap was in PAUSED state when URScript injected
- URControl: `Starting program: multi_waypoint_ssm_pick_place` at 21:51:39
- Runtime state: PROGRAM_STATE_PAUSED → PROGRAM_STATE_STOPPED (silent abandon)
- No runtime error logged; URCap reverse_socket connection not yet established
- Joint range: 0 (script scheduled but not executed)
- Note: a successful race-coordinated retry (v5+) may reveal motion-time violations

### T005 (human_proximity_ssm) — Class 3: Workspace reachability failure
- Generated waypoints with TCP Z=-0.198m (below robot base)
- URControl error: `Runtime error (3:5): The robot cannot reach the requested pose. Script function movej is unable to find an inverse kinematics solution.`
- IK solver rejected pre-execution
- Joint range: 0 (no motion)

## Telemetry artifacts
- T001_telemetry_v3.csv (60010 rows, baseline run, no Play, ROS2 driver running)
- T001_telemetry_v4.csv (1 row — overwrite issue, race-coordinated wrapper run; URControl log carries the evidence)
- T004_telemetry_v4.csv (59038 rows, no motion)
- T005_telemetry_v4.csv (59690 rows, no motion)

## Threat-model implications for paper §VI
3 pre-execution failure classes evade static analysis (DM-1..7) and rely on
URSim runtime to surface. Motivates URCap-mediated execution gate as
complementary detection layer to the static watchdog. Failure class
taxonomy: (1) API semantic misuse, (2) timing/state coupling, (3) workspace
reachability.

## Open follow-ups (NTNU Visit 1 week 1)
- T004 v5 race-coordinated retry — may reveal Class 4 (motion-time violation)
- T001 v5 retry — telemetry overwrite recovery (low priority, URControl log sufficient)
- All 3 tasks across remaining 2 LLMs (DeepSeek-Coder-V2-16B, CodeLlama-34B)
- DM-8 design: API-type checker for URScript pose vs joint vector args
