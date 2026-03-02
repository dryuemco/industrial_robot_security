# enfield_robots_ur5e

UR5e robot adapter for the ENFIELD framework.

## Contents

- `urdf/` — UR5e URDF/Xacro (to be added from ur_description)
- `srdf/` — MoveIt2 semantic description
- `config/moveit/` — MoveIt2 planning configuration
- `config/kinematic_limits.yaml` — ISO 10218 mapped speed/payload limits
- `meshes/` — Visual/collision meshes
- `launch/` — Gazebo and MoveIt2 launch files

## Robot Adapter Contract

Each robot adapter must provide:
1. URDF/Xacro + meshes
2. SRDF + MoveIt2 config (planning pipelines, controllers)
3. ros2_control controller config (joint_trajectory_controller etc.)
4. Simulation launch (Gazebo/ros_gz bridge)
5. Kinematic limits table (ISO clause mapping for evidence generation)
