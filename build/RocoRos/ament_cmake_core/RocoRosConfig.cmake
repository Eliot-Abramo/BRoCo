# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_RocoRos_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED RocoRos_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(RocoRos_FOUND FALSE)
  elseif(NOT RocoRos_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(RocoRos_FOUND FALSE)
  endif()
  return()
endif()
set(_RocoRos_CONFIG_INCLUDED TRUE)

# output package information
if(NOT RocoRos_FIND_QUIETLY)
  message(STATUS "Found RocoRos: 0.0.0 (${RocoRos_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'RocoRos' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${RocoRos_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(RocoRos_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${RocoRos_DIR}/${_extra}")
endforeach()
