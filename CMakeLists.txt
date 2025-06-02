cmake_minimum_required(VERSION 3.5)
project(ros2_maestro)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
install(
  DIRECTORY ros2_maestro
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    ros2_maestro/maestro_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
