cmake_minimum_required(VERSION 3.5)
project(turtlebot_nav)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    src/trainer_node.py
    src/sensor_helpers.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
