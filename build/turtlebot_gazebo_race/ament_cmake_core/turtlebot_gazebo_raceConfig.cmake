# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_turtlebot_gazebo_race_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED turtlebot_gazebo_race_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(turtlebot_gazebo_race_FOUND FALSE)
  elseif(NOT turtlebot_gazebo_race_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(turtlebot_gazebo_race_FOUND FALSE)
  endif()
  return()
endif()
set(_turtlebot_gazebo_race_CONFIG_INCLUDED TRUE)

# output package information
if(NOT turtlebot_gazebo_race_FIND_QUIETLY)
  message(STATUS "Found turtlebot_gazebo_race: 0.0.1 (${turtlebot_gazebo_race_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'turtlebot_gazebo_race' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT turtlebot_gazebo_race_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(turtlebot_gazebo_race_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${turtlebot_gazebo_race_DIR}/${_extra}")
endforeach()
