cmake_minimum_required(VERSION 3.16)
project(imprimis_hardware_platform LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# declare all dependencies except serial
# the serial dependency isn't a standard ros package,
# so it must be handled differently.
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


# find all dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# find serial dependency
find_package(serial REQUIRED)


# add target
add_library(
  imprimis_hardware_platform
  SHARED
  hardware/diffbot_system.cpp
  hardware/serial_comms.cpp  
)


# target include dirs and compile features
target_compile_features(imprimis_hardware_platform PUBLIC cxx_std_17)
target_include_directories(imprimis_hardware_platform PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/imprimis_hardware_platform>
)


# add all dependencies except serial
ament_target_dependencies(
  imprimis_hardware_platform PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# add serial dependency
target_link_libraries(imprimis_hardware_platform PUBLIC ${serial_LIBRARIES})
target_include_directories(imprimis_hardware_platform PUBLIC ${serial_INCLUDE_DIRS})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface imprimis_hardware_platform.xml)


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/imprimis_hardware_platform
)

install(
  DIRECTORY launch config worlds meshes
  DESTINATION share/imprimis_hardware_platform
)

install(TARGETS imprimis_hardware_platform
  EXPORT export_imprimis_hardware_platform
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_imprimis_hardware_platform HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} serial)
ament_package()
