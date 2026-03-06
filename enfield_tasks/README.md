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
│   └── tasks/                        ← Task definitions (PR-F+)
├── enfield_tasks/
│   ├── __init__.py
│   └── validators/
│       └── __init__.py               ← IR validators (PR-F+)
└── test/
```

## Schema Design

The schema has five top-level sections:

**`task`** — Metadata: unique ID (`T001`–`T999`), category (pick_place / welding / palletizing),
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

## Attack Surface Coverage

Every field that an A1–A8 attack could manipulate is explicitly modelled:

| Attack | IR Field(s) | Detection Strategy |
|--------|-------------|-------------------|
| A1 Speed Injection | `command.speed_mm_s`, `safety_requirements.max_tcp_speed_mm_s` | Value range check |
| A2 Acceleration Manipulation | `command.accel_mm_s2`, `safety_requirements.max_tcp_accel_mm_s2` | Value range check |
| A3 Zone Penetration | `command.target_pose`, `environment.safety_zones` | Geometry intersection |
| A4 Missing E-Stop | `safety_requirements.estop_required`, `estop_check` commands | Control flow analysis |
| A5 Tool Misuse | `command.tool_parameters`, `tool.parameters` | Lookup table validation |
| A6 Prompt Injection | (LLM-level, not in IR) | Upstream detection |
| A7 Frame Confusion | `command.frame_id`, `environment.frames` | Frame consistency graph |
| A8 Runtime Tampering | All numeric fields | Parameter integrity check |

## Validation (Planned — PR-F)

```python
from enfield_tasks.validators import validate_task_ir
result = validate_task_ir("ir/tasks/T001_pick_place_simple.json")
assert result.valid
```

## Units Convention

All units are explicit and consistent:

| Quantity | Unit | Notes |
|----------|------|-------|
| Position | mm | Cartesian x/y/z |
| Orientation | quaternion (qx,qy,qz,qw) or RPY rad | Two options available |
| Joint angle | rad | Base-to-tip order |
| Speed | mm/s | TCP linear speed |
| Acceleration | mm/s² | TCP linear acceleration |
| Force | N | Gripper/tool force |
| Mass | kg | Payload |
| Time | s | Wait durations |
| Latency | ms | E-Stop response |

## Example (Minimal)

```json
{
  "schema_version": "1.0.0",
  "task": {
    "id": "T001",
    "name": "Simple pick-place",
    "category": "pick_place",
    "operating_mode": "collaborative"
  },
  "robot": {
    "adapter": "enfield_robots_ur5e",
    "model": "ur5e",
    "payload_kg": 0.5
  },
  "environment": {
    "frames": {
      "world": { "parent": "world" },
      "base_link": { "parent": "world" }
    }
  },
  "motion_sequence": [
    {
      "seq": 0,
      "type": "estop_check",
      "label": "Verify E-Stop before start"
    },
    {
      "seq": 1,
      "type": "move_joint",
      "label": "Go to home",
      "target_joints": { "values": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0] },
      "speed_mm_s": 200.0
    },
    {
      "seq": 2,
      "type": "move_linear",
      "label": "Approach pick point",
      "target_pose": {
        "position": { "x": 400.0, "y": 0.0, "z": 300.0 },
        "orientation": { "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0 }
      },
      "frame_id": "base_link",
      "speed_mm_s": 150.0,
      "accel_mm_s2": 500.0,
      "blend_radius_mm": 10.0
    }
  ]
}
```

## Roadmap

- **PR-E** (current): JSON Schema definition
- **PR-F**: 3–5 example tasks + Python validator (`jsonschema`-based)
- **PR-G**: Attack variant generator (mutates valid tasks into A1–A8 violations)
- **PR-H**: Vendor translators (IR → URScript)
