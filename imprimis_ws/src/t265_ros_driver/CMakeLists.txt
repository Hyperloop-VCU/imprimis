cmake_minimum_required(VERSION 3.8)
project(t265_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# use ament macros
find_package(ament_cmake REQUIRED)

# set ROS dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  nav_msgs
)

# find ROS dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find realsense dependency
find_package(realsense2 REQUIRED)

# add target
add_executable(main src/main.cpp)

# target all local include directories
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target linking and include directories for ROS dependencies
ament_target_dependencies(
  main PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# target linking and include directories for realsense dependency
target_link_libraries(main PUBLIC ${realsense2_LIBRARY})
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include)


# install
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})



ament_package()
