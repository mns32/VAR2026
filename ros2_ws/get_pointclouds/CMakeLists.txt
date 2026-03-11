cmake_minimum_required(VERSION 3.8)
project(get_pointclouds)

# Silence CMake dev warnings about *_ROOT variables used by find_package.
# Guarded so it still configures with older CMake versions.
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

add_executable(get_pointclouds_node src/node.cpp)
ament_target_dependencies(get_pointclouds_node rclcpp sensor_msgs pcl_conversions)

target_include_directories(get_pointclouds_node PUBLIC ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(get_pointclouds_node ${PCL_LIBRARIES})

install(TARGETS get_pointclouds_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
