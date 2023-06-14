cmake_minimum_required(VERSION 3.8)
project(image_sensor_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(image_sensor_broadcaster_parameters
  src/image_sensor_broadcaster_parameters.yaml
)

ament_auto_add_library(image_sensor_broadcaster
  SHARED
  src/image_sensor_broadcaster.cpp
)
target_link_libraries(image_sensor_broadcaster
  image_sensor_broadcaster_parameters
)

pluginlib_export_plugin_description_file(
  h6x_controller_interface image_sensor_broadcaster.xml)

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(h6x_controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_image_sensor_broadcaster
    test/test_load_image_sensor_broadcaster.cpp
  )
  target_link_libraries(test_load_image_sensor_broadcaster
    image_sensor_broadcaster
  )
  ament_target_dependencies(test_load_image_sensor_broadcaster
    h6x_controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_image_sensor_broadcaster
    test/test_image_sensor_broadcaster.cpp
  )
  target_link_libraries(test_image_sensor_broadcaster
    image_sensor_broadcaster
  )
  ament_target_dependencies(test_image_sensor_broadcaster
    hardware_interface
  )
endif()

ament_auto_package()