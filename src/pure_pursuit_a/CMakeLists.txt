cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pure_pursuit_a_node src/pure_pursuit_a_node.cpp)
target_compile_features(pure_pursuit_a_node PRIVATE cxx_std_14)
target_include_directories(pure_pursuit_a_node PRIVATE include)
ament_target_dependencies(pure_pursuit_a_node rclcpp nav_msgs ackermann_msgs ament_index_cpp)

install(TARGETS pure_pursuit_a_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY waypoints DESTINATION share/${PROJECT_NAME})
install(
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
