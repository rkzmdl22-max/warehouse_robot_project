cmake_minimum_required(VERSION 3.8)
project(tiago_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------
# Install package-local directories into the ament share tree.
# Each directory is OPTIONAL so a missing folder does not break builds.
# --------------------------------------------------------------------
install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# --------------------------------------------------------------------
# Also install the workspace-level worlds/ directory so that
# get_package_share_directory("tiago_gazebo")/worlds resolves to the
# same .world files that are mounted in Docker at /worlds.
#
# The path below is relative to the colcon workspace root:
#   <ws>/worlds  →  install/tiago_gazebo/share/tiago_gazebo/worlds
#
# Adjust WORLD_DIR if your colcon workspace root differs.
# --------------------------------------------------------------------
set(WORKSPACE_WORLDS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../worlds")

if(EXISTS "${WORKSPACE_WORLDS_DIR}")
  install(
    DIRECTORY "${WORKSPACE_WORLDS_DIR}/"
    DESTINATION share/${PROJECT_NAME}/worlds
  )
else()
  message(WARNING
    "tiago_gazebo: workspace worlds/ directory not found at "
    "${WORKSPACE_WORLDS_DIR}. "
    "Place .world files in ros_ws/src/tiago_gazebo/worlds/ instead."
  )
endif()

# --------------------------------------------------------------------
# Testing
# --------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
endif()

ament_package()
