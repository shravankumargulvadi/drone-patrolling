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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add your executable
add_executable(drone_commander src/send_command_template.cpp)

# Specify dependencies for your executable
ament_target_dependencies(drone_commander
  rclcpp
  px4_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  drone_commander
  DESTINATION lib/${PROJECT_NAME}
)

# Necessary for ROS2 package
ament_package()
