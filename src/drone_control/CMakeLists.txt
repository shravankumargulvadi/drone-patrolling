cmake_minimum_required(VERSION 3.8)
project(drone_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compile options, if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneStatus.msg"
  "msg/PathPlannerUpdates.msg"
  "srv/PathPlanner.srv"
  DEPENDENCIES action_msgs geometry_msgs
)

add_library(path_planning_utils src/path_planning_utils.cpp)

ament_target_dependencies(path_planning_utils
    geometry_msgs
    rclcpp
)

# Add this right after the add_library command
target_include_directories(path_planning_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Add your executable
add_executable(drone_manager src/drone_manager.cpp)
add_executable(ground_control_node src/ground_control_node.cpp)

# Specify external dependencies for your executable
ament_target_dependencies(drone_manager
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
)

# Only for external dependencies.
ament_target_dependencies(ground_control_node 
  geodesy
  geographic_msgs
  geometry_msgs
  rclcpp 
  std_msgs 
)

# Get the C++ typesupport target for interfaces in this project
rosidl_get_typesupport_target(drone_control_interface ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Link ground_control_node against the drone_control interfaces defined in the srv file. 
target_link_libraries(ground_control_node 
  ${drone_control_interface}
  path_planning_utils
)

# Link ground_control_node against the drone_control interfaces defined in the srv file. 
target_link_libraries(drone_manager 
  ${drone_control_interface}
)

install(TARGETS path_planning_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install the executable
install(TARGETS
  drone_manager
  ground_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "srv"
  DESTINATION "share/${PROJECT_NAME}/srv"
)

install(
  DIRECTORY "msg"
  DESTINATION "share/${PROJECT_NAME}/msg"
)

# Install the headers.
install(
  DIRECTORY include/
  DESTINATION include
)

# Necessary for ROS2 package
ament_package()
