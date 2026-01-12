cmake_minimum_required(VERSION 3.12)
project(rbpodo_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# ===============================
# Resource files (URDF, meshes, worlds, launch, rviz, config)
# ===============================
install(
  DIRECTORY
    robots
    meshes
    launch
    rviz
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

# ===============================
# Python scripts (ros2 run 가능)
# ===============================
install(
  PROGRAMS
    scripts/spawn_and_attach_tof.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

