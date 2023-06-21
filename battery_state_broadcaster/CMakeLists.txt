cmake_minimum_required(VERSION 3.8)
project(battery_state_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(battery_state_broadcaster_parameters
  src/battery_state_broadcaster_parameters.yaml
)

ament_auto_add_library(battery_state_broadcaster
  SHARED
  src/battery_state_broadcaster.cpp
)
target_link_libraries(battery_state_broadcaster
  battery_state_broadcaster_parameters
)

pluginlib_export_plugin_description_file(
  controller_interface battery_state_broadcaster.xml)

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_rostest_with_parameters_gmock(test_load_battery_state_broadcaster
    test/test_load_battery_state_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/battery_state_broadcaster_params.yaml)
  target_link_libraries(test_load_battery_state_broadcaster
  battery_state_broadcaster)
  ament_target_dependencies(test_load_battery_state_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets)

  ament_add_gmock(test_battery_state_broadcaster
    test/test_battery_state_broadcaster.cpp
  )
  target_link_libraries(test_battery_state_broadcaster
  battery_state_broadcaster
  )
  ament_target_dependencies(test_battery_state_broadcaster
    hardware_interface)
endif()

ament_auto_package()