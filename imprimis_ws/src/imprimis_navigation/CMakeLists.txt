cmake_minimum_required(VERSION 3.8)
project(imprimis_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)


add_library(speed_critic SHARED
  src/speed_critic.cpp
)

target_include_directories(speed_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(speed_critic
  rclcpp
  pluginlib
  nav2_mppi_controller
  nav2_costmap_2d
)

pluginlib_export_plugin_description_file(nav2_mppi_controller speed_critic_plugin.xml)

# install
install(TARGETS speed_critic
  DESTINATION lib
)

install(FILES speed_critic_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch config scripts nav2_bt include
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch config scripts nav2_bt
  DESTINATION share/${PROJECT_NAME}
)
#add controller switching script
install(PROGRAMS
  scripts/controller_switcher.py
  scripts/velocity_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
