cmake_minimum_required(VERSION 3.8)
project(disparity_extender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories into build
include_directories(include)

# Add disparity extender executable
add_executable(disparity_extender_node src/disparity_extender_node.cpp)
ament_target_dependencies(disparity_extender_node rclcpp ackermann_msgs sensor_msgs nav_msgs std_msgs)
target_link_libraries(disparity_extender_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  disparity_extender_node
  DESTINATION lib/${PROJECT_NAME})

# Add launch
install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
