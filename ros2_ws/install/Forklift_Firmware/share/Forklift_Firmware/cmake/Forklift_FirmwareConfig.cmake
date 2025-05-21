# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Forklift_Firmware_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Forklift_Firmware_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Forklift_Firmware_FOUND FALSE)
  elseif(NOT Forklift_Firmware_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Forklift_Firmware_FOUND FALSE)
  endif()
  return()
endif()
set(_Forklift_Firmware_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Forklift_Firmware_FIND_QUIETLY)
  message(STATUS "Found Forklift_Firmware: 0.0.0 (${Forklift_Firmware_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Forklift_Firmware' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Forklift_Firmware_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Forklift_Firmware_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Forklift_Firmware_DIR}/${_extra}")
endforeach()
