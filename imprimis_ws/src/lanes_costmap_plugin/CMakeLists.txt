cmake_minimum_required(VERSION 3.5)
project(lanes_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)

set(dep_pkgs
    OpenCV
    rclcpp
    nav2_costmap_2d
    pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/lane_layer.cpp)
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===
pluginlib_export_plugin_description_file(nav2_costmap_2d lane_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
