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

### 1. Install Dependencies

```bash
# ROS2 Humble core + UR packages
sudo apt install ros-humble-desktop ros-humble-ur-description

# Gazebo Ignition integration
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control

# MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-planners-ompl \
  ros-humble-moveit-simple-controller-manager ros-humble-moveit-ros-visualization

# ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# CLI tools (topic, action, etc.)
sudo apt install ros-humble-ros2topic ros-humble-ros2action ros-humble-ros2launch

# Build/test dependencies
sudo apt install ros-humble-xacro ros-humble-ament-lint-auto ros-humble-ament-lint-common
```

### 2. Environment Setup

Add the following to `~/.bashrc` (required once):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib" >> ~/.bashrc
```

The `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` variable is required for Gazebo Ignition Fortress
to locate the `ign_ros2_control` plugin under `/opt/ros/humble/lib/`.

### 3. Build

```bash
cd ~/ros2_ws/src/industrial_robot_security
colcon build --symlink-install --packages-select enfield_robots_ur5e
source install/setup.bash
```

### 4. Launch Simulation

```bash
# Terminal 1: Gazebo + UR5e + controllers
ros2 launch enfield_robots_ur5e ur5e_sim.launch.py

# Terminal 2: MoveIt2 move_group
ros2 launch enfield_robots_ur5e ur5e_moveit.launch.py

# Terminal 3: Verify joint states
ros2 topic echo /joint_states --once
```

Expected home position: `shoulder_lift_joint ≈ -1.57`, `wrist_1_joint ≈ -1.57`, others ≈ 0.

## Dependencies

- `ur_description` — upstream UR URDF/meshes
- `ign_ros2_control` — Gazebo Ignition Fortress hardware interface
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

## Troubleshooting

### `Unable to parse the value of parameter robot_description as yaml`

Both launch files use `ParameterValue(value, value_type=str)` to wrap URDF/SRDF XML
strings. Without this wrapper, the launch system attempts to parse XML as YAML and fails.

### `Failed to load system plugin [ign_ros2_control-system]`

Gazebo Ignition cannot find the `libign_ros2_control-system.so` shared library. Set:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
```

### `parent link [world] of joint [base_joint] not found`

The `ur_description` macro creates a `base_joint` with parent `world`, but does not
define the `world` link itself. The wrapper xacro includes `<link name="world"/>` to
satisfy this requirement.

### `ros2: error: invalid choice: 'launch'` / `'topic'`

Missing CLI verb packages. Install:

```bash
sudo apt install ros-humble-ros2launch ros-humble-ros2topic ros-humble-ros2action
```

Also ensure every terminal sources ROS2: `source /opt/ros/humble/setup.bash`

### Controller spawner waiting indefinitely for `/controller_manager`

This is a symptom of the Ignition plugin not loading. Check the Gazebo terminal output
for `[Err] [SystemLoader.cc:...]` messages and verify `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`
is set correctly.
