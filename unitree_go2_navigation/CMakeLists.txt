cmake_minimum_required(VERSION 3.8)
project(unitree_go2_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/go2_nav_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
