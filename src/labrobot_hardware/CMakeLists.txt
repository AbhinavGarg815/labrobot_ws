cmake_minimum_required(VERSION 3.16)
project(labrobot_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The wire format is defined once, in the firmware tree, and compiled into both
# the Arduino sketch and this plugin. Duplicating it here would let the two ends
# of the same link drift apart silently.
set(FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/labrobot_firmware)
if(NOT EXISTS ${FIRMWARE_DIR}/labrobot_protocol.h)
  message(FATAL_ERROR
    "Shared protocol header not found at ${FIRMWARE_DIR}. "
    "labrobot_hardware must be built inside the workspace that contains firmware/.")
endif()

add_library(labrobot_hardware SHARED
  src/rmcs2303_system.cpp
  src/serial_port.cpp
  ${FIRMWARE_DIR}/labrobot_protocol.cpp
)

target_include_directories(labrobot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${FIRMWARE_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(labrobot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface labrobot_hardware.xml)

install(
  TARGETS labrobot_hardware
  EXPORT export_labrobot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(labrobot_hardware)
ament_export_targets(export_labrobot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
