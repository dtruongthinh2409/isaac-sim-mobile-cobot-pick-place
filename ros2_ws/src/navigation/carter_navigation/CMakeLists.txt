cmake_minimum_required(VERSION 3.5)
project(carter_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz2
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/scan_qos_relay.py
  scripts/tf_static_relay.py
  scripts/generate_initial_pose.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
