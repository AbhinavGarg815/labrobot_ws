cmake_minimum_required(VERSION 3.8)
project(labrobot_follow)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The python library (tracker, identity, geometry, TensorRT runtime ...).
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/person_tracker.py
    scripts/person_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tracking test/test_tracking.py)
endif()

ament_package()
