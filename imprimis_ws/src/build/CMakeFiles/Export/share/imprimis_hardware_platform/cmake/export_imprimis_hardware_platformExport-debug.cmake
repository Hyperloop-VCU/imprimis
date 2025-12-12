#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "imprimis_hardware_platform::imprimis_hardware_platform" for configuration "Debug"
set_property(TARGET imprimis_hardware_platform::imprimis_hardware_platform APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(imprimis_hardware_platform::imprimis_hardware_platform PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libimprimis_hardware_platform.so"
  IMPORTED_SONAME_DEBUG "libimprimis_hardware_platform.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS imprimis_hardware_platform::imprimis_hardware_platform )
list(APPEND _IMPORT_CHECK_FILES_FOR_imprimis_hardware_platform::imprimis_hardware_platform "${_IMPORT_PREFIX}/lib/libimprimis_hardware_platform.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
