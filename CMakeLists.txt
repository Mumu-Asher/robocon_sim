cmake_minimum_required(VERSION 3.8)
project(robocon_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf config maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_package()
