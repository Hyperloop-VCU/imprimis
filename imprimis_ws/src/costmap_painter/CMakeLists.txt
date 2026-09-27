cmake_minimum_required(VERSION 3.8)
project(costmap_painter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(library_name costmap_painter_core)   # must match <library path="..."> in the plugin XML
set(dependencies
  rclcpp
  nav2_costmap_2d
  pluginlib
)

# ---- Plugin library ----
add_library(${library_name} SHARED
  src/costmap_painter.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/costmap_painter>
)

ament_target_dependencies(${library_name} ${dependencies})

# Register the plugin with nav2_costmap_2d's pluginlib index
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_painter.xml)

# ---- Install ----
install(TARGETS ${library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES costmap_painter.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---- Exports ----
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()