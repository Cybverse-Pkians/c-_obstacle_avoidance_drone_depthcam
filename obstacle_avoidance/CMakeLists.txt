cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(avoidance_node
  src/avoidance_node.cpp
)

add_executable(fake_obstacle_node
  fake_obstacle_node.cpp
)

ament_target_dependencies(avoidance_node
  rclcpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
  cv_bridge
)

ament_target_dependencies(fake_obstacle_node
  rclcpp
  sensor_msgs
)

target_link_libraries(avoidance_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  avoidance_node
  fake_obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
