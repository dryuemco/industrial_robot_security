# enfield_robots_ur5e

UR5e robot adapter for the ENFIELD adversarial testing framework.

## Robot Adapter Contract

This package fulfils the 5-point adapter contract:

| # | Component | Status |
|---|-----------|--------|
| 1 | URDF/Xacro + meshes | ✅ Wrapper xacro including `ur_description` |
| 2 | SRDF + MoveIt2 config | ✅ Planning group, joint limits, OMPL, kinematics |
| 3 | ros2_control config | ✅ JointTrajectoryController @ 500 Hz |
| 4 | Simulation launch | ✅ Gazebo Ignition + ros_gz_bridge |
| 5 | Kinematic limits (ISO) | ✅ `config/kinematic_limits.yaml` |

## File Structure

```
enfield_robots_ur5e/
├── urdf/
│   └── ur5e_enfield.urdf.xacro    # Wrapper: ur_description + gz_ros2_control
├── srdf/
│   └── ur5e.srdf                   # MoveIt2 semantic description
├── config/
│   ├── kinematic_limits.yaml       # ISO 10218 mapped limits (PR-C)
│   ├── ur5e_controllers.yaml       # ros2_control: JTC + JSB
│   ├── initial_positions.yaml      # Home configuration
│   ├── gz_bridge.yaml              # Ignition ↔ ROS2 topic bridge
│   └── moveit/
│       ├── joint_limits.yaml       # MoveIt2 safety-constrained limits
│       ├── kinematics.yaml         # KDL solver
│       ├── ompl_planning.yaml      # OMPL planner pipelines
│       └── moveit_controllers.yaml # FollowJointTrajectory mapping
├── launch/
│   ├── ur5e_sim.launch.py          # Gazebo + robot + controllers
│   └── ur5e_moveit.launch.py       # MoveIt2 move_group node
├── meshes/                         # Visual/collision meshes (from ur_description)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Quick Start

```bash
# 1. Launch Gazebo simulation with UR5e
ros2 launch enfield_robots_ur5e ur5e_sim.launch.py

# 2. In a second terminal, launch MoveIt2
ros2 launch enfield_robots_ur5e ur5e_moveit.launch.py

# 3. Verify joint states are published
ros2 topic echo /joint_states --once
```

## Dependencies

- `ur_description` — upstream UR URDF/meshes
- `gz_ros2_control` — Gazebo Ignition hardware interface
- `ros_gz_sim`, `ros_gz_bridge` — Gazebo-ROS2 integration
- `moveit_ros_move_group`, `moveit_planners_ompl` — MoveIt2 planning

## ISO 10218 Safety Limits

Collaborative mode defaults are applied in `config/moveit/joint_limits.yaml`.
See `config/kinematic_limits.yaml` for the full ISO clause mapping.

| Parameter | Collaborative | Fenced | ISO Clause |
|-----------|--------------|--------|------------|
| TCP speed | ≤250 mm/s | ≤500 mm/s | 5.6 |
| Payload | ≤5.0 kg | ≤5.0 kg | 5.3 |
| E-Stop latency | ≤50 ms | ≤50 ms | 5.4 |
