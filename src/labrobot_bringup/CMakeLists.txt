cmake_minimum_required(VERSION 3.8)
project(labrobot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/drive_test.py
    scripts/compare_odom.py
    scripts/check_system.py
    scripts/check_map.py
    scripts/check_costmap.py
    scripts/localisation_check.py
    scripts/relocalise.py
    scripts/auto_localise.py
    scripts/nav_goal.py
    scripts/analyse_motion.py
    scripts/verify_wheels.py
    scripts/publish_cloud.py
    scripts/monitor_slam.py
    scripts/check_ground_plane.py
    scripts/check_imu.py
    scripts/analyse_floor.py
    scripts/imu_bias_compensator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
