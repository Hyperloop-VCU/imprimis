# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imprimis_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imprimis_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imprimis_FOUND FALSE)
  elseif(NOT imprimis_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imprimis_FOUND FALSE)
  endif()
  return()
endif()
set(_imprimis_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imprimis_FIND_QUIETLY)
  message(STATUS "Found imprimis: 0.0.0 (${imprimis_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imprimis' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imprimis_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imprimis_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imprimis_DIR}/${_extra}")
endforeach()
