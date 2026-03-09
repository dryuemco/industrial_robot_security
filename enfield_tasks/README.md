# enfield_tasks

Vendor-neutral Task Intermediate Representation (IR) for the ENFIELD adversarial testing framework.

## Overview

Tasks are defined as JSON files conforming to `task_ir_v1.schema.json`. The IR is deliberately
robot-agnostic: it captures **what** the robot should do (motion sequence, speeds, zones, tool
operations) without encoding **how** (no URScript, RAPID, or KRL). Vendor-specific code generation
is handled by separate translator modules.

## Schema Location

```
enfield_tasks/
├── ir/
│   ├── schema/
│   │   └── task_ir_v1.schema.json   ← JSON Schema (Draft 2020-12)
│   └── tasks/
│       ├── T001_pick_place_collab.json
│       ├── T002_linear_weld_fenced.json
│       ├── T003_palletize_hybrid.json
│       ├── T004_multi_wp_obstacle_collab.json
│       └── T005_inspection_scan_collab.json
├── enfield_tasks/
│   ├── __init__.py
│   └── validators/
│       ├── __init__.py
│       └── schema_validator.py       ← JSON Schema + semantic checks
└── test/
    └── test_schema_validation.py     ← 75 tests
```

## Schema Design

The schema has five top-level required sections plus four optional safety sections:

**`task`** — Metadata: unique ID (`T001`–`T999`), category (pick_place / welding / palletizing / inspection),
and operating mode (collaborative / fenced / hybrid). The operating mode maps directly to
ISO 10218 speed limits.

**`robot`** — Which adapter package and model this task targets. Watchdog cross-references
this with `kinematic_limits.yaml` from the robot adapter.

**`environment`** — Coordinate frames and safety zones. Frames form a tree rooted at `world`.
Safety zones are primitives (box/sphere/cylinder) with exclusion/confinement semantics.

**`tool`** — End-effector type and parameters (gripper force, weld current, etc.).
The watchdog validates these against per-tool lookup tables.

**`motion_sequence`** — Ordered list of commands: `move_linear`, `move_joint`, `move_circular`,
`set_tool`, `set_digital_output`, `wait`, `estop_check`, `comment`.

**`safety_requirements`** — Speed/accel limits, safeguarded space (halfspaces), forbidden
orientation cones, tool payload bounds.

**`safety_logic_requirements`** — Required AST node patterns (A5).

**`work_object_specification`** — Expected work object frame for frame confusion detection (A6).

**`prompt_security`** — LLM prompt monitoring parameters (A8).

## Attack Surface Coverage

Every field that an A1–A8 attack could manipulate is explicitly modelled:

| Attack | Name | IR Field(s) | Detection Strategy |
|--------|------|-------------|-------------------|
| A1 | Speed Injection | `command.speed_mm_s`, `safety_requirements.max_tcp_speed_mm_s` | Value range check |
| A2 | Zone Penetration | `command.target_pose`, `safety_requirements.safeguarded_space` | Halfspace intersection |
| A3 | Orientation Anomaly | `command.target_pose.orientation`, `safety_requirements.forbidden_orientation_cones` | Cone intersection |
| A4 | Payload Misconfiguration | `robot.payload_kg`, `safety_requirements.tool_payload` | Bounds check |
| A5 | E-Stop / Logic Bypass | `motion_sequence` (estop_check commands), `safety_logic_requirements` | AST pattern + CFG |
| A6 | Frame Confusion | `command.frame_id`, `environment.frames`, `work_object_specification` | Frame registry |
| A7 | Tool Misuse | `tool.type`, `tool.activation_constraints` | State machine |
| A8 | Prompt Injection | `prompt_security` | Prompt analysis + A1–A7 |

See `docs/attack_definitions_A1_A4.md` and `docs/attack_definitions_A5_A8.md` for formal definitions.

## Validation

```python
from enfield_tasks.validators import validate_task_ir, validate_file

# Validate a parsed dict
result = validate_task_ir(task_data)
assert result.valid
print(result.summary())

# Validate from file path
result = validate_file("ir/tasks/T001_pick_place_collab.json")
```

The validator performs JSON Schema validation (Draft 2020-12, strict `additionalProperties: false`)
plus semantic checks for A1 (speed vs mode), A2 (waypoint vs halfspace), A4 (payload bounds),
A5 (estop command presence), and A6 (frame reference consistency).

## Units Convention

All units are explicit and consistent:

| Quantity | Unit | Notes |
|----------|------|-------|
| Position | mm | Cartesian x/y/z |
| Orientation | quaternion (qx,qy,qz,qw) | Unit quaternion |
| Joint angle | rad | Base-to-tip order |
| Speed | mm/s | TCP linear speed |
| Acceleration | mm/s² | TCP linear acceleration |
| Force | N | Gripper/tool force |
| Mass | kg | Payload |
| Time | s | Wait durations |
| Latency | ms | E-Stop response |

## Baseline Tasks (PR-F)

| ID | Category | Mode | Speed Limit | Payload | Key A-fields |
|----|----------|------|-------------|---------|-------------|
| T001 | pick_place | collaborative | 250 mm/s | 0.5 kg | A1–A8 full coverage |
| T002 | welding | fenced | 500 mm/s | 3.0 kg | A3 (2 cones), A7 (arc sequence) |
| T003 | palletizing | hybrid | 300 mm/s | 4.5 kg | A2 (tight space), A4 (heavy payload) |
| T004 | pick_place | collaborative | 250 mm/s | 1.5 kg | A5 (4 required nodes), A6 (dual frames) |
| T005 | inspection | collaborative | 150 mm/s | 0.3 kg | A1 (tight limit), A3 (2 cones) |

## Roadmap

- **PR-E** ✅ JSON Schema definition (34 tests)
- **PR-F** ✅ 5 baseline tasks + Python validator (75 tests)
- **PR-G** ✅ Attack variant generator — 40 adversarial variants (69 tests)
- **PR-H**: Vendor translators (IR → URScript)
