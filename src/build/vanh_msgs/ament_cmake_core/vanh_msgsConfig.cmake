# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vanh_msgs_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vanh_msgs_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vanh_msgs_FOUND FALSE)
  elseif(NOT vanh_msgs_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vanh_msgs_FOUND FALSE)
  endif()
  return()
endif()
set(_vanh_msgs_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vanh_msgs_FIND_QUIETLY)
  message(STATUS "Found vanh_msgs: 0.0.0 (${vanh_msgs_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vanh_msgs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vanh_msgs_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vanh_msgs_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vanh_msgs_DIR}/${_extra}")
endforeach()
