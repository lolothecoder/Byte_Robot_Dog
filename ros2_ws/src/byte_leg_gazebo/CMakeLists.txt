cmake_minimum_required(VERSION 3.8)
project(byte_leg_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
