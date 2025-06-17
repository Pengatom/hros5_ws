cmake_minimum_required(VERSION 3.10)
project(hros5_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

