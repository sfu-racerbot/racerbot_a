cmake_minimum_required(VERSION 3.8)
project(reactive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories into build
include_directories(include)

# Add gap message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gap.msg"
)

# Needed to reference the generated headers/typesupport within this same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add gap finder executable
add_executable(gap_finder_node src/gap_finder_node.cpp)
ament_target_dependencies(gap_finder_node rclcpp ackermann_msgs sensor_msgs nav_msgs)
target_link_libraries(gap_finder_node "${cpp_typesupport_target}")

# Add gap follower executable
add_executable(gap_follow_node src/gap_follow_node.cpp)
ament_target_dependencies(gap_follow_node rclcpp ackermann_msgs sensor_msgs nav_msgs)
target_link_libraries(gap_follow_node "${cpp_typesupport_target}")

# Safety node executable
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp ackermann_msgs sensor_msgs nav_msgs)

# Install executables
install(TARGETS
  gap_finder_node
  gap_follow_node
  safety_node
  DESTINATION lib/${PROJECT_NAME})

# add launch
install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
