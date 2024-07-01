cmake_minimum_required(VERSION 3.8)
project(robotic_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(
  robotic_hardware 
  SHARED 
  src/robotic_diffdrive.cpp 
  src/nucleo_comms.cpp
)

target_compile_features(robotic_hardware PUBLIC cxx_std_17)
target_include_directories(
  robotic_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/robotic_hardware>
)

ament_target_dependencies(
  robotic_hardware
  PUBLIC
  hardware_interface
  rclcpp
  pluginlib
  rclcpp_components
)

target_link_libraries(robotic_hardware PUBLIC serial)

target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOTIC_HARDWARE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  DIRECTORY controllers launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS robotic_hardware
  EXPORT export_robotic_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_robotic_hardware HAS_LIBRARY_TARGET)


ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  rclcpp_components
)



ament_package()

