# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dual_rviz_jazzy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dual_rviz_jazzy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dual_rviz_jazzy_FOUND FALSE)
  elseif(NOT dual_rviz_jazzy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dual_rviz_jazzy_FOUND FALSE)
  endif()
  return()
endif()
set(_dual_rviz_jazzy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dual_rviz_jazzy_FIND_QUIETLY)
  message(STATUS "Found dual_rviz_jazzy: 0.0.1 (${dual_rviz_jazzy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dual_rviz_jazzy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT dual_rviz_jazzy_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dual_rviz_jazzy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dual_rviz_jazzy_DIR}/${_extra}")
endforeach()
