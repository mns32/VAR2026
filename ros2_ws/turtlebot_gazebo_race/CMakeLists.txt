cmake_minimum_required(VERSION 3.8)
project(turtlebot_gazebo_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_description_topic_publisher src/robot_description_topic_publisher.cpp)
ament_target_dependencies(robot_description_topic_publisher rclcpp std_msgs)

add_executable(depth_camera_info_publisher src/depth_camera_info_publisher.cpp)
ament_target_dependencies(depth_camera_info_publisher rclcpp sensor_msgs)

# Instalar los directorios launch y worlds para que ROS los encuentre
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  robot_description_topic_publisher
  depth_camera_info_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()