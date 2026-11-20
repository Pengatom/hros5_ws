cmake_minimum_required(VERSION 3.8)
project(hros5_gazebo)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)
install(
  PROGRAMS
    scripts/reset_world_service.py
    scripts/sim_dxl_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
