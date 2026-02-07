cmake_minimum_required(VERSION 3.8)
project(mobile_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# Generate service interfaces (GoToNamedTarget.srv, GoToPose.srv)
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToNamedTarget.srv"
  "srv/GoToPose.srv"
  DEPENDENCIES geometry_msgs
)

# ==============================================================================
# C++ executable: manipulation_server
# ==============================================================================
add_executable(manipulation_server src/manipulation_server.cpp)
ament_target_dependencies(manipulation_server
  rclcpp
  std_srvs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

# Link generated service interfaces with C++ target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(manipulation_server "${cpp_typesupport_target}")

install(TARGETS
  manipulation_server
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Python scripts
# ==============================================================================
install(PROGRAMS
  scripts/task_executor.py
  scripts/navigation_client.py
  scripts/manipulation_client.py
  scripts/test_arm_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Launch and config files
# ==============================================================================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
