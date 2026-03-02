cmake_minimum_required(VERSION 3.8)
project(enfield_robots_ur5e)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------
# Install directories
# ---------------------------------------------------------------

# URDF / Xacro
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# SRDF (MoveIt2 semantic description)
install(DIRECTORY srdf/
  DESTINATION share/${PROJECT_NAME}/srdf
)

# Config: ros2_control, MoveIt2, Gazebo bridge, kinematic limits
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Mesh files (currently empty — populated when visual meshes added)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  OPTIONAL
)

# ---------------------------------------------------------------
# Testing
# ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
