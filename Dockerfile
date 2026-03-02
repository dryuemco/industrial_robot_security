# =============================================================================
# ENFIELD Project - Multi-Stage Dockerfile
# ROS2 Humble + Multi-Package Workspace
# =============================================================================

# STAGE 1: Build (compile all ROS2 packages)
FROM ros:humble AS builder

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy entire project as source
COPY . src/enfield

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Build all packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# STAGE 2: Runtime (minimal image)
FROM ros:humble-ros-base AS runtime

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy built workspace from builder
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Install Python runtime deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Source workspace on shell entry
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
