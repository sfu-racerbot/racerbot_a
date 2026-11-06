cmake_minimum_required(VERSION 3.8)
project(sim_tests)

########
# test #
########
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  #ROS domain isolation provided for any added tests
  ament_add_pytest_test(
    simulation
    test/test_simulation.py
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    TIMEOUT 120
  )

endif()

ament_package()