# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mid70pp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mid70pp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mid70pp_FOUND FALSE)
  elseif(NOT mid70pp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mid70pp_FOUND FALSE)
  endif()
  return()
endif()
set(_mid70pp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mid70pp_FIND_QUIETLY)
  message(STATUS "Found mid70pp: 0.0.0 (${mid70pp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mid70pp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mid70pp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mid70pp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mid70pp_DIR}/${_extra}")
endforeach()
