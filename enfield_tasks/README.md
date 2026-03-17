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
│       ├── T005_inspection_scan_collab.json
│       ├── T006_bin_pick_suction_fenced.json
│       ├── T007_curved_weld_hybrid.json
│       ├── T008_heavy_depal_fenced.json
│       ├── T009_laser_scan_fenced.json
│       ├── T010_screw_assembly_hybrid.json
│       ├── T011_dual_zone_pick_hybrid.json
│       ├── T012_spot_weld_collab.json
│       ├── T013_collab_suction_pallet.json
│       ├── T014_hybrid_camera_inspect.json
│       └── T015_laser_marking_fenced.json
├── enfield_tasks/
│   ├── __init__.py
│   └── validators/
│       ├── __init__.py
│       └── schema_validator.py       ← JSON Schema + semantic checks
└── test/
    └── test_schema_validation.py     ← 169 tests (79 static + 90 parametrized)
```

## Schema Design

The schema has five top-level required sections plus four optional safety sections:

**`task`** — Metadata: unique ID (`T001`–`T999`), category (pick_place / welding / palletizing /
inspection / custom), and operating mode (collaborative / fenced / hybrid). The operating mode
maps directly to ISO 10218 speed limits.

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

## Baseline Tasks (15)

| ID | Category | Mode | Speed Limit | Payload | Key A-fields |
|----|----------|------|-------------|---------|-------------|
| T001 | pick_place | collaborative | 250 mm/s | 0.5 kg | A1–A8 full coverage |
| T002 | welding | fenced | 500 mm/s | 3.0 kg | A3 (2 cones), A7 (arc sequence) |
| T003 | palletizing | hybrid | 300 mm/s | 4.5 kg | A2 (tight space), A4 (heavy payload) |
| T004 | pick_place | collaborative | 250 mm/s | 1.5 kg | A5 (4 required nodes), A6 (dual frames) |
| T005 | inspection | collaborative | 150 mm/s | 0.3 kg | A1 (tight limit), A3 (2 cones) |
| T006 | pick_place | fenced | 400 mm/s | 2.0 kg | A1 (higher fenced limit), A7 (suction) |
| T007 | welding | hybrid | 300 mm/s | 3.5 kg | A3 (strict torch-down), A7 (arc sequence) |
| T008 | palletizing | fenced | 450 mm/s | 5.0 kg | A4 (max payload boundary), A2 (tight space) |
| T009 | inspection | fenced | 200 mm/s | 0.5 kg | A3 (3 laser cones), A7 (laser activation) |
| T010 | custom | hybrid | 250 mm/s | 1.0 kg | A7 (screwdriver), A5 (torque monitor) |
| T011 | pick_place | hybrid | 300 mm/s | 1.2 kg | A6 (dual-zone frame), mode transition |
| T012 | welding | collaborative | 250 mm/s | 2.5 kg | A1 (collab+weld), A3 (3 cones) |
| T013 | palletizing | collaborative | 250 mm/s | 0.8 kg | A4 (light payload near min), A7 (suction) |
| T014 | inspection | hybrid | 200 mm/s | 0.4 kg | A3 (varied orientations), A6 (fixture frame) |
| T015 | custom | fenced | 400 mm/s | 0.6 kg | A7 (laser), A3 (4 cones), A6 (marking fixture) |

### Task Coverage Matrix (Category × Mode)

|  | collaborative | fenced | hybrid |
|--|--------------|--------|--------|
| pick_place | T001, T004 | T006 | T011 |
| welding | T012 | T002 | T007 |
| palletizing | T013 | T008 | T003 |
| inspection | T005 | T009 | T014 |
| custom | — | T015 | T010 |

## Test Structure (169 tests)

The test file uses **dynamic glob** — no task IDs are hardcoded. Adding T016 requires zero
changes to any test file.

| Class | Tests | Scope |
|-------|-------|-------|
| `TestSchemaIntegrity` | 5 | Schema file self-validation |
| `TestT001Validation` | 11 | T001 detailed checks |
| `TestSchemaRejection` | 12 | Negative tests (invalid IR rejected) |
| `TestSemanticWarnings` | 5 | Semantic checker triggers |
| `TestFileValidation` | 1 | File I/O edge cases |
| `TestT002Validation` | 9 | T002 welding-specific checks |
| `TestT003Validation` | 8 | T003 palletizing-specific checks |
| `TestT004Validation` | 8 | T004 obstacle avoidance checks |
| `TestT005Validation` | 7 | T005 inspection-specific checks |
| `TestAllTasksSchemaValid` | **90** | 6 checks × 15 tasks (parametrized) |
| `TestTaskSuiteCoverage` | 13 | Suite-level coverage matrix |
| **Total** | **169** | |

```bash
# Run all enfield_tasks tests
PYTHONPATH=enfield_tasks python3 -m pytest enfield_tasks/test/ -v

# Run only the parametrized 15-task coverage
PYTHONPATH=enfield_tasks python3 -m pytest \
  enfield_tasks/test/test_schema_validation.py::TestAllTasksSchemaValid -v
```

## Roadmap

- **PR-E** ✅ JSON Schema definition (34 tests)
- **PR-F** ✅ 5 baseline tasks + Python validator (75 tests)
- **PR-G** ✅ Attack variant generator — 120 adversarial variants (69 tests)
- **PR-H** ✅ Vendor translators (IR → URScript, 15 tasks)
- **PR-K** ✅ T006–T015 baseline tasks + dynamic schema tests (169 tests total)
