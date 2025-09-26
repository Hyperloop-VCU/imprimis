cmake_minimum_required(VERSION 3.16)
project(cmd_vel_gui VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS setup
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Qt setup
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# Sources from Qt Creator project
set(PROJECT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/mainwindow.h
  src/mainwindow.ui
)

# Create target
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
  qt_add_executable(main
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
  )
else()
  add_executable(main
    ${PROJECT_SOURCES}
  )
endif()
target_compile_features(main PRIVATE cxx_std_17)

# ROS dependencies
ament_target_dependencies(main
  rclcpp
  geometry_msgs
)

# Qt dependency
target_link_libraries(main
  Qt${QT_VERSION_MAJOR}::Widgets
)

# Install
install(TARGETS main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Qt cleanup
if(QT_VERSION_MAJOR EQUAL 6)
  qt_finalize_executable(main)
endif()

# ROS cleanup
ament_package()
