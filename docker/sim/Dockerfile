# =============================================================================
# ENFIELD — Simulation/runtime image (ROS 2 Humble)
#
# Extends the repo's `enfield:dev` builder stage (see ../../Dockerfile) with the
# UR driver, Ignition Fortress bridge and MoveIt packages that enfield_bringup
# and enfield_robots_ur5e declare in their package.xml but that the base
# ros:humble image does not ship.
#
# Ignition (not gz) is deliberate: enfield_robots_ur5e/urdf/ur5e_enfield.urdf.xacro
# instantiates `ign_ros2_control::IgnitionROS2ControlPlugin` and
# config/gz_bridge.yaml declares `ignition.msgs.*` types — the Fortress-era API
# that ships with Humble.
#
# Build: docker build -t enfield:sim -f docker/sim/Dockerfile docker/sim/
# Run:   docker run --rm -it --net=host enfield:sim
# =============================================================================

FROM enfield:dev

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ur \
      ros-humble-ign-ros2-control \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      ros-humble-moveit \
      ros-humble-joint-state-publisher \
      ros-humble-xacro \
      ros-humble-rviz2 \
      netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
