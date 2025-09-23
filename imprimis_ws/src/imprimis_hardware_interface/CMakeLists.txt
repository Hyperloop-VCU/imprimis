cmake_minimum_required(VERSION 3.8)
project(diff_drive_hw)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(diff_drive_system SHARED src/diff_drive_system.cpp)
target_include_directories(diff_drive_system PRIVATE include)
ament_target_dependencies(diff_drive_system
  rclcpp pluginlib hardware_interface
)
# Export as pluginlib
pluginlib_export_plugin_description_file(hardware_interface diff_drive_system_plugins.xml)

install(TARGETS diff_drive_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES diff_drive_system_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
