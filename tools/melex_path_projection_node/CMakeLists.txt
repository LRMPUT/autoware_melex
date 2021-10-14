cmake_minimum_required(VERSION 3.5)
project(melex_path_projection_node)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  PROGRAMS
    nodes/melex_path_projection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
    launch/
)
