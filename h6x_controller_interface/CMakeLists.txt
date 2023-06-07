cmake_minimum_required(VERSION 3.8)
project(h6x_controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(h6x_controller_interface
  SHARED
  src/h6x_controller_interface.cpp
)

target_compile_definitions(h6x_controller_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(sensor_msgs REQUIRED)

  # ament_add_gmock(test_image_sensor test/test_image_sensor.cpp)
  # target_link_libraries(test_image_sensor
  #   controller_interface
  #   hardware_interface::hardware_interface
  # )
  # ament_target_dependencies(test_image_sensor
  #   sensor_msgs
  # )

endif()

ament_auto_package()
