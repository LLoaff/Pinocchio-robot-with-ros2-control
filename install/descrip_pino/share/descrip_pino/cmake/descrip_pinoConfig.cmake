# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_descrip_pino_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED descrip_pino_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(descrip_pino_FOUND FALSE)
  elseif(NOT descrip_pino_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(descrip_pino_FOUND FALSE)
  endif()
  return()
endif()
set(_descrip_pino_CONFIG_INCLUDED TRUE)

# output package information
if(NOT descrip_pino_FIND_QUIETLY)
  message(STATUS "Found descrip_pino: 0.0.0 (${descrip_pino_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'descrip_pino' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${descrip_pino_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(descrip_pino_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${descrip_pino_DIR}/${_extra}")
endforeach()
