cmake_minimum_required(VERSION 3.8)
project(rowboboat_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

## COMPILE
add_library(
  rowboboat_control
  SHARED
  src/paddle_wheel.cpp
)

target_compile_features(rowboboat_control PUBLIC cxx_std_17)
target_include_directories(rowboboat_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/rowboboat_control>
)

ament_target_dependencies(
  rowboboat_control PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rowboboat_control_plugins.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/rowboboat_control
)

install(TARGETS rowboboat_control
  EXPORT export_rowboboat_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()


## EXPORTS
ament_export_targets(export_rowboboat_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()