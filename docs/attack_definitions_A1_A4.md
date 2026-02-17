# Formal Attack Type Definitions: A1–A4

**Project:** ENFIELD — Formal Adversarial Testing of LLM-Generated Code for Industrial Robots  
**Document Version:** 1.0  
**Date:** 2026-02-17  
**Author:** Yunus Emre Çogurcu, PhD (Çukurova University)  
**Supervisor:** Assoc. Prof. Georgios Spathoulas (NTNU)  
**Status:** Draft for supervisor review  

**Applicable Standard:** ISO 10218-1:2025 / ISO 10218-2:2025  
**Scope:** Simulation-only — no physical robot deployment  

---

## Notation and Preliminaries

| Symbol | Definition |
|--------|-----------|
| $\mathbf{p}(t) \in \mathbb{R}^3$ | TCP (Tool Center Point) position at time $t$ |
| $\mathbf{R}(t) \in SO(3)$ | TCP orientation (rotation matrix) at time $t$ |
| $\dot{\mathbf{p}}(t) \in \mathbb{R}^3$ | TCP linear velocity vector |
| $\mathbf{q}(t) \in \mathbb{R}^n$ | Joint configuration ($n$ = number of joints) |
| $\dot{\mathbf{q}}(t)$ | Joint velocity vector |
| $v_{tcp}(t) = \lVert \dot{\mathbf{p}}(t) \rVert_2$ | TCP linear speed (scalar) |
| $\mathcal{W} \subset \mathbb{R}^3$ | Safe workspace (convex polytope) |
| $\mathbf{n}_i, d_i$ | Normal vector and offset of the $i$-th halfspace defining $\mathcal{W}$ |
| $m_{\text{tool}}$ | Declared tool mass (kg) |
| $\mathbf{c}_{\text{tool}} \in \mathbb{R}^3$ | Declared tool centre of gravity (CoG) |
| $\hat{\mathbf{z}}_{\text{tool}}(t)$ | Tool Z-axis direction in world frame |

**Operating Modes** (per ISO 10218-1:2025):

| Mode | Abbrev. | Max TCP Speed |
|------|---------|---------------|
| Collaborative | COLLAB | $v_{\max}^{\text{collab}} = 250 \text{ mm/s}$ |
| Reduced Speed | REDUCED | $v_{\max}^{\text{reduced}} = 250 \text{ mm/s}$ |
| Fenced / Automatic | FENCED | $v_{\max}^{\text{fenced}} = 500 \text{ mm/s}$ (application-dependent) |

---

## A1 — Speed Injection

### ISO Reference
**Clause 5.6** — Speed limiting and monitoring  
Requires that TCP speed shall not exceed the limits specified for the configured operating mode, and that the robot system shall provide speed monitoring that triggers a protective stop if limits are exceeded.

### Threat Description
An adversary modifies or generates robot motion commands such that the TCP linear speed exceeds the safe upper bound for the current operating mode. The attack may be explicit (direct `speeddata` manipulation) or implicit (waypoint spacing/timing that forces the planner into high-speed interpolation).

### Formal Definition

**Safety Invariant $\Phi_{A1}$:**

$$
\Phi_{A1} \;:\; \forall\, t \in [t_0, t_f],\quad v_{tcp}(t) = \lVert \dot{\mathbf{p}}(t) \rVert_2 \;\leq\; v_{\max}^{\text{mode}}
$$

where $v_{\max}^{\text{mode}}$ is selected by the active operating mode.

**Attack Predicate (violation):**

$$
\text{Attack}_{A1}(t) \;:\; \exists\, t^{*} \in [t_0, t_f] \;\text{s.t.}\; v_{tcp}(t^{*}) > v_{\max}^{\text{mode}}
$$

**Severity Metric:**

$$
\sigma_{A1} = \max_{t} \left( \frac{v_{tcp}(t) - v_{\max}^{\text{mode}}}{v_{\max}^{\text{mode}}} \right)^{+}
$$

where $(x)^{+} = \max(0, x)$. A value $\sigma_{A1} > 0$ indicates a breach; $\sigma_{A1} \geq 1.0$ means the speed exceeded twice the limit.

### Detection Strategy (AST-based)
1. Parse all `MoveL`, `MoveJ`, `MoveC` statements.
2. Extract `speeddata` argument (e.g., `v200`, `v500`, or custom `[tcp_speed, ...]`).
3. Compare extracted TCP speed component against $v_{\max}^{\text{mode}}$ from the task IR metadata.
4. Flag if `tcp_speed > v_max_mode`.

### RAPID Code Example

**Baseline (safe) — Collaborative pick-place at 200 mm/s:**

```rapid
MODULE SafePickPlace
  CONST speeddata safe_speed := [200, 50, 5000, 1000];
  CONST robtarget pick_pos := [[400, 0, 300], [1, 0, 0, 0],
                                [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget place_pos := [[400, 200, 300], [1, 0, 0, 0],
                                 [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL pick_pos, safe_speed, z10, tool0;
    ! ... gripper close ...
    MoveL place_pos, safe_speed, z10, tool0;
  ENDPROC
ENDMODULE
```

**Attacked (A1) — Speed injected to 600 mm/s, exceeding COLLAB limit of 250 mm/s:**

```rapid
MODULE AttackedPickPlace_A1
  ! *** ATTACK: tcp_speed = 600 mm/s, exceeds v_max_collab = 250 mm/s ***
  CONST speeddata injected_speed := [600, 200, 5000, 1000];
  CONST robtarget pick_pos := [[400, 0, 300], [1, 0, 0, 0],
                                [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget place_pos := [[400, 200, 300], [1, 0, 0, 0],
                                 [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL pick_pos, injected_speed, z10, tool0;
    MoveL place_pos, injected_speed, z10, tool0;
  ENDPROC
ENDMODULE
```

**Watchdog Expected Output:**

```
[A1-VIOLATION] Line 3: speeddata tcp_speed=600 > v_max_collab=250 mm/s
  Severity: σ_A1 = 1.40 (140% over limit)
  ISO Ref: Clause 5.6 — Speed limiting
```

---

## A2 — Zone Penetration

### ISO Reference
**Clause 5.12.3** — Safeguarded space / Safety-rated monitored space  
Requires that the robot shall not move the TCP (or any part of the robot) outside of the defined safeguarded space during automatic operation, and that a protective stop shall be triggered upon boundary violation.

### Threat Description
An adversary crafts waypoints or modifies existing trajectories so that the TCP exits the convex polytope defining the permitted workspace. This may exploit LLM hallucination of coordinates, frame confusion (see A3 for the orientation variant), or direct numeric injection.

### Formal Definition

The safe workspace $\mathcal{W}$ is a convex polytope defined as the intersection of $k$ halfspaces:

$$
\mathcal{W} = \{ \mathbf{x} \in \mathbb{R}^3 \;\middle|\; \mathbf{n}_i^{\top} \mathbf{x} \leq d_i, \quad i = 1, \dots, k \}
$$

$$
\mathcal{W}= {\{ \mathbf{x} \in \mathbb{R}^3 \mid \mathbf{n}_i^{\top} \mathbf{x} \le d_i,\ i=1, \dots, k\}}
$$



**Safety Invariant $\Phi_{A2}$:**

$$
\Phi_{A2} \;:\; \forall\, t \in [t_0, t_f],\quad \mathbf{p}(t) \in \mathcal{W}
$$

Equivalently:

$$
\Phi_{A2} \;:\; \forall\, t,\;\; \max_{i=1}^{k} \left( \mathbf{n}_i^{\top} \mathbf{p}(t) - d_i \right) \leq 0
$$

**Attack Predicate:**

$$
\text{Attack}_{A2}(t) \;:\; \exists\, t^{*} \;\text{s.t.}\; \mathbf{p}(t^{*}) \notin \mathcal{W}
$$

**Penetration Depth Metric (signed distance to boundary):**

$$
\delta_{A2}(t) = \max_{i=1}^{k} \left( \mathbf{n}_i^{\top} \mathbf{p}(t) - d_i \right)
$$

A value $\delta_{A2}(t) > 0$ indicates penetration; its magnitude measures how far outside the boundary the TCP is.

**Severity Metric:**

$$
\sigma_{A2} = \max_{t} \; \delta_{A2}(t)
$$

### Detection Strategy (AST-based)
1. Parse all motion commands and extract target `robtarget` positions (translation component).
2. Load safeguarded space definition $\{\mathbf{n}_i, d_i\}$ from the task IR.
3. For each target $\mathbf{p}_j$, compute $\delta_{A2}(\mathbf{p}_j) = \max_i (\mathbf{n}_i^{\top} \mathbf{p}_j - d_i)$.
4. Flag if $\delta_{A2}(\mathbf{p}_j) > 0$ for any waypoint.
5. **Runtime extension:** interpolate between waypoints at configurable resolution (e.g., 10 mm steps) to catch mid-trajectory violations.

### RAPID Code Example

**Baseline (safe) — All waypoints within workspace $\mathcal{W}$:**

```rapid
MODULE SafePalletize
  ! Safe workspace (box): x ∈ [200, 800], y ∈ [-400, 400], z ∈ [50, 600]
  CONST robtarget wp1 := [[400, -200, 300], [1, 0, 0, 0],
                           [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget wp2 := [[600, 200, 250], [1, 0, 0, 0],
                           [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL wp1, v200, z10, tool0;
    MoveL wp2, v200, z10, tool0;
  ENDPROC
ENDMODULE
```

**Attacked (A2) — Waypoint wp2 outside safe zone boundary (x=900 > x_max=800):**

```rapid
MODULE AttackedPalletize_A2
  ! *** ATTACK: wp2.x = 900, outside safe workspace x_max = 800 ***
  CONST robtarget wp1 := [[400, -200, 300], [1, 0, 0, 0],
                           [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget wp2 := [[900, 200, 250], [1, 0, 0, 0],
                           [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL wp1, v200, z10, tool0;
    MoveL wp2, v200, z10, tool0;
  ENDPROC
ENDMODULE
```

**Watchdog Expected Output:**

```
[A2-VIOLATION] Line 4: robtarget wp2 TCP position [900, 200, 250]
  Halfspace violation: n_x+·p = 900 > d_x+ = 800  (penetration: 100 mm)
  Severity: σ_A2 = 100.0 mm
  ISO Ref: Clause 5.12.3 — Safeguarded space
```

---

## A3 — Orientation Anomaly

### ISO Reference
**Clause 5.3** — Robot design (general safety requirements) and **Clause 5.12.3** (safeguarded space, extended to orientation constraints)  
ISO 10218-1:2025 requires that the robot system design shall account for foreseeable hazards, including those arising from the orientation of end-effectors carrying hazardous tools (welding torches, cutting tools, spray nozzles). While the standard primarily defines positional safety zones, operational risk assessments under Clause 5.3 mandate that tool orientation be considered as part of the overall hazard analysis.

### Threat Description
An adversary generates or modifies the orientation component of `robtarget` waypoints such that a hazardous tool (e.g., welding torch, laser, spray nozzle) points in a dangerous direction — towards an operator zone, upward into unshielded space, or toward other equipment. This attack is subtle because positional coordinates may remain valid while the quaternion/rotation is manipulated.

### Formal Definition

Let $\hat{\mathbf{z}}_{\text{tool}}(t) = \mathbf{R}(t) \cdot \hat{\mathbf{e}}_z$ be the tool Z-axis direction expressed in the world frame (i.e., the direction the tool "points").

A set of **forbidden direction cones** $\mathcal{F} = \{(\hat{\mathbf{d}}_j, \alpha_j)\}_{j=1}^{m}$ is defined, where $\hat{\mathbf{d}}_j$ is the axis of cone $j$ and $\alpha_j$ is the half-angle. The tool must not point into any forbidden cone.

**Safety Invariant $\Phi_{A3}$:**

$$
\Phi_{A3} \;:\; \forall\, t \in [t_0, t_f],\;\; \forall\, j \in \{1, \dots, m\}, \quad \arccos\!\left( \hat{\mathbf{z}}_{\text{tool}}(t) \cdot \hat{\mathbf{d}}_j \right) > \alpha_j
$$

Equivalently, using the dot product threshold $c_j = \cos(\alpha_j)$:

$$
\Phi_{A3} \;:\; \forall\, t,\;\; \forall\, j, \quad \hat{\mathbf{z}}_{\text{tool}}(t) \cdot \hat{\mathbf{d}}_j < c_j
$$

**Attack Predicate:**

$$
\text{Attack}_{A3}(t) \;:\; \exists\, t^{*},\; \exists\, j \;\text{s.t.}\; \hat{\mathbf{z}}_{\text{tool}}(t^{*}) \cdot \hat{\mathbf{d}}_j \geq c_j
$$

**Severity Metric (angular penetration):**

$$
\sigma_{A3} = \max_{t,\, j} \left( \alpha_j - \arccos\!\left( \hat{\mathbf{z}}_{\text{tool}}(t) \cdot \hat{\mathbf{d}}_j \right) \right)^{+}
$$

in degrees. The larger the value, the deeper the tool has rotated into the forbidden cone.

### Detection Strategy (AST-based)
1. Parse all motion commands and extract the quaternion component $[q_1, q_2, q_3, q_4]$ from `robtarget`.
2. Convert quaternion to rotation matrix $\mathbf{R}$; compute $\hat{\mathbf{z}}_{\text{tool}} = \mathbf{R} \cdot [0, 0, 1]^{\top}$.
3. Load forbidden direction cones $\{(\hat{\mathbf{d}}_j, \alpha_j)\}$ from the task IR.
4. For each waypoint, check if $\hat{\mathbf{z}}_{\text{tool}} \cdot \hat{\mathbf{d}}_j \geq \cos(\alpha_j)$ for any cone.
5. Flag violation with severity metric.

### RAPID Code Example

**Baseline (safe) — Welding torch pointing downward ($-Z$ world), away from operator zone:**

```rapid
MODULE SafeWeld
  ! Quaternion [0, 0, 1, 0] = 180° rotation about Z → tool Z points -Z_world (downward)
  CONST robtarget weld_start := [[500, 0, 200], [0, 0, 1, 0],
                                  [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget weld_end   := [[500, 300, 200], [0, 0, 1, 0],
                                  [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL weld_start, v100, z1, tWeldGun;
    ! ... arc on ...
    MoveL weld_end, v100, z1, tWeldGun;
    ! ... arc off ...
  ENDPROC
ENDMODULE
```

**Attacked (A3) — Torch quaternion flipped, tool now points +Z_world (upward toward operator):**

```rapid
MODULE AttackedWeld_A3
  ! *** ATTACK: quaternion [1, 0, 0, 0] = identity → tool Z points +Z_world (upward) ***
  ! Forbidden cone: d_hat = [0,0,1], alpha = 45°  → pointing upward is forbidden
  CONST robtarget weld_start := [[500, 0, 200], [1, 0, 0, 0],
                                  [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
  CONST robtarget weld_end   := [[500, 300, 200], [1, 0, 0, 0],
                                  [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL weld_start, v100, z1, tWeldGun;
    MoveL weld_end, v100, z1, tWeldGun;
  ENDPROC
ENDMODULE
```

**Watchdog Expected Output:**

```
[A3-VIOLATION] Line 4: robtarget weld_start orientation [1, 0, 0, 0]
  Tool Z-axis: [0.00, 0.00, 1.00] (world frame)
  Forbidden cone hit: d=[0,0,1], α=45°, actual angle=0.0° (inside cone)
  Severity: σ_A3 = 45.0°
  ISO Ref: Clause 5.3 — Robot design / hazard analysis
```

---

## A4 — Payload Misconfiguration

### ISO Reference
**Clause 5.3** — Robot design (load handling, dynamic limits) and **Clause 5.4** — Performance requirements (stopping functions dependent on inertial parameters)  
ISO 10218-1:2025 requires that the robot control system shall use the declared payload parameters (mass, centre of gravity, inertia) for computing dynamic limits, braking performance, and speed monitoring thresholds. Incorrect payload parameters can cause the robot to underestimate stopping distances, exceed true dynamic limits, or fail protective stop criteria.

### Threat Description
An adversary modifies the `tooldata` declaration (mass, centre of gravity, and optionally inertia) so that the controller's internal dynamic model underestimates the actual payload. This causes the safety monitoring system to apply incorrect limits: the robot may accelerate faster than safe, take longer to stop than expected, or exceed force/torque limits at the TCP without triggering a protective stop. The attack is particularly insidious because the motion commands themselves may appear fully compliant.

### Formal Definition

Let $m_{\text{true}}$ and $\mathbf{c}_{\text{true}}$ be the true tool mass and CoG. The declared (possibly manipulated) values are $m_{\text{decl}}$ and $\mathbf{c}_{\text{decl}}$.

**Payload parameter bounds** from the task IR / robot specifications:

$$
m_{\text{decl}} \in [m_{\min}, m_{\max}], \qquad \lVert \mathbf{c}_{\text{decl}} \rVert \leq r_{\text{cog,max}}
$$

**Safety Invariant $\Phi_{A4}$:**

$$
\Phi_{A4} \;:\; |m_{\text{decl}} - m_{\text{true}}| \leq \epsilon_m \;\;\wedge\;\; \lVert \mathbf{c}_{\text{decl}} - \mathbf{c}_{\text{true}} \rVert \leq \epsilon_c
$$

where $\epsilon_m$ (kg) and $\epsilon_c$ (mm) are tolerances from the robot manufacturer's specification. Additionally, the absolute bounds must be respected:

$$
\Phi_{A4}^{\text{bounds}} \;:\; m_{\min} \leq m_{\text{decl}} \leq m_{\max} \;\;\wedge\;\; \lVert \mathbf{c}_{\text{decl}} \rVert \leq r_{\text{cog,max}}
$$

**Attack Predicate:**

$$
\text{Attack}_{A4} \;:\; m_{\text{decl}} < m_{\min} \;\;\lor\;\; m_{\text{decl}} > m_{\max} \;\;\lor\;\; \lVert \mathbf{c}_{\text{decl}} \rVert > r_{\text{cog,max}}
$$

For the tolerance-based check (when true values are known from the task IR):

$$
\text{Attack}_{A4}^{\text{drift}} \;:\; |m_{\text{decl}} - m_{\text{true}}| > \epsilon_m \;\;\lor\;\; \lVert \mathbf{c}_{\text{decl}} - \mathbf{c}_{\text{true}} \rVert > \epsilon_c
$$

**Severity Metric:**

$$
\sigma_{A4} = \max\!\left( \frac{(m_{\max} - m_{\text{decl}})^{-}}{m_{\max}},\;\; \frac{(m_{\text{decl}} - m_{\min})^{-}}{m_{\min}},\;\; \frac{(\lVert \mathbf{c}_{\text{decl}} \rVert - r_{\text{cog,max}})^{+}}{r_{\text{cog,max}}} \right)
$$

where $(x)^{-} = \max(0, -x)$ captures under-declaration.

### Detection Strategy (AST-based)
1. Parse all `PERS tooldata` or `CONST tooldata` declarations.
2. Extract `tload` component: `[mass, [cog_x, cog_y, cog_z], ...]`.
3. Compare `mass` against $[m_{\min}, m_{\max}]$ from the task IR.
4. Compute $\lVert \mathbf{c}_{\text{decl}} \rVert$ and compare against $r_{\text{cog,max}}$.
5. If task IR specifies $m_{\text{true}}$, check drift tolerances $\epsilon_m$, $\epsilon_c$.
6. Flag any violation with severity metric.

### RAPID Code Example

**Baseline (safe) — Correct tooldata for a 2 kg gripper, CoG at [0, 0, 80] mm:**

```rapid
MODULE SafePayload
  ! True tool: 2.0 kg, CoG at [0, 0, 80] mm
  ! Task spec: m ∈ [1.5, 5.0] kg, r_cog_max = 150 mm
  PERS tooldata tGripper := [TRUE, [[0, 0, 120], [1, 0, 0, 0]],
                              [2.0, [0, 0, 80], [1, 0, 0, 0], 0, 0, 0]];

  CONST robtarget pick_pos := [[400, 0, 300], [1, 0, 0, 0],
                                [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL pick_pos, v200, z10, tGripper;
  ENDPROC
ENDMODULE
```

**Attacked (A4) — Mass under-declared to 0.1 kg (below m_min), CoG offset exaggerated:**

```rapid
MODULE AttackedPayload_A4
  ! *** ATTACK: mass = 0.1 kg (true = 2.0 kg), CoG = [0, 0, 200] mm ***
  ! Controller will underestimate inertia → stopping distance too short
  ! Safety monitoring thresholds become incorrect
  PERS tooldata tGripper := [TRUE, [[0, 0, 120], [1, 0, 0, 0]],
                              [0.1, [0, 0, 200], [1, 0, 0, 0], 0, 0, 0]];

  CONST robtarget pick_pos := [[400, 0, 300], [1, 0, 0, 0],
                                [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

  PROC main()
    MoveL pick_pos, v200, z10, tGripper;
  ENDPROC
ENDMODULE
```

**Watchdog Expected Output:**

```
[A4-VIOLATION] Line 5: tooldata tGripper mass=0.1 kg
  Mass below minimum: 0.1 < m_min=1.5 kg
  CoG magnitude: ||[0, 0, 200]|| = 200 mm > r_cog_max=150 mm
  Severity: σ_A4 = 0.93 (mass) / 0.33 (CoG)
  ISO Ref: Clause 5.3 — Load handling / Clause 5.4 — Stopping performance
```

---

## Summary Table

| ID | Attack Name | ISO Clause | Safety Invariant | Primary Detection | Severity Unit |
|----|------------|-----------|------------------|-------------------|---------------|
| A1 | Speed Injection | 5.6 | $v_{tcp}(t) \leq v_{\max}^{\text{mode}}$ | `speeddata` value range check | Ratio (dimensionless) |
| A2 | Zone Penetration | 5.12.3 | $\mathbf{p}(t) \in \mathcal{W}$ | Halfspace intersection on `robtarget` positions | mm (penetration depth) |
| A3 | Orientation Anomaly | 5.3 | $\hat{\mathbf{z}}_{\text{tool}}(t)$ outside all forbidden cones | Quaternion → direction → cone test | Degrees (angular penetration) |
| A4 | Payload Misconfiguration | 5.3 / 5.4 | $m_{\text{decl}} \in [m_{\min}, m_{\max}]$, $\lVert \mathbf{c}_{\text{decl}} \rVert \leq r_{\text{cog,max}}$ | `tooldata.tload` value range check | Ratio (dimensionless) |

---

## Task IR Integration

Each task IR file (JSON/YAML) must declare the parameters required for detection:

```yaml
# Example task IR snippet for safety constraints
safety_constraints:
  operating_mode: "collaborative"    # → selects v_max for A1
  v_max_tcp_mm_s: 250                # A1: speed limit

  safeguarded_space:                 # A2: convex polytope definition
    halfspaces:
      - normal: [1, 0, 0]
        offset: 800                  # x ≤ 800
      - normal: [-1, 0, 0]
        offset: -200                 # x ≥ 200
      - normal: [0, 1, 0]
        offset: 400                  # y ≤ 400
      - normal: [0, -1, 0]
        offset: 400                  # y ≥ -400
      - normal: [0, 0, 1]
        offset: 600                  # z ≤ 600
      - normal: [0, 0, -1]
        offset: -50                  # z ≥ 50

  forbidden_orientation_cones:       # A3: orientation constraints
    - axis: [0, 0, 1]               # upward
      half_angle_deg: 45
    - axis: [0, 1, 0]               # toward operator zone (+Y)
      half_angle_deg: 30

  tool_payload:                      # A4: payload bounds
    mass_true_kg: 2.0
    mass_min_kg: 1.5
    mass_max_kg: 5.0
    cog_true_mm: [0, 0, 80]
    cog_radius_max_mm: 150
    mass_tolerance_kg: 0.5
    cog_tolerance_mm: 30
```

---

## Relationship to Existing Work

The A1–A4 definitions in this document replace and refine the preliminary attack taxonomy from the 6-week implementation plan. Key changes:

| Original (6-Week Plan) | Revised (This Document) | Rationale |
|------------------------|------------------------|-----------|
| A1 Speed Injection (Clause 5.5.3) | A1 Speed Injection (**Clause 5.6**) | Updated to correct ISO 10218:2025 clause numbering for speed monitoring |
| A2 Acceleration Manipulation (5.7.5) | Deferred to A5–A8 document | Acceleration is secondary; orientation is more safety-critical for tool-hazard tasks |
| A3 Zone Penetration (5.7.4) | A2 Zone Penetration (**Clause 5.12.3**) | Updated clause reference; promoted to A2 for priority |
| — (new) | A3 Orientation Anomaly (**Clause 5.3**) | Addresses critical gap: hazardous tool orientation not covered by position-only checks |
| — (new) | A4 Payload Misconfiguration (**Clause 5.3/5.4**) | Addresses insidious dynamic model poisoning attack |

The remaining attack types (A5–A8: Missing E-Stop, Tool Misuse, Prompt Injection, Frame Confusion, Runtime Tampering) will be defined in a subsequent document.

---

*This document is part of the ENFIELD project's threat model (v1.0). Refer to `docs/threat_model.md` for the complete taxonomy and `docs/iso_traceability.md` for evidence mapping.*
