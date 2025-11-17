cmake_minimum_required(VERSION 3.10)
project(hros5_head_tracker_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# --- tracker node (OpenCV) ---
add_executable(tracker_node src/tracker_node.cpp)
ament_target_dependencies(tracker_node
  rclcpp sensor_msgs std_msgs image_transport cv_bridge)
target_link_libraries(tracker_node ${OpenCV_LIBS})

# --- Dynamixel node (C++) ---
add_executable(dxl_node src/dxl_node.cpp)
ament_target_dependencies(dxl_node rclcpp std_msgs)

# Dynamixel SDK (system-wide). If not found, vendor it and link manually.
find_library(DXL_LIB dynamixel_sdk)
if(DXL_LIB)
  message(STATUS "Found Dynamixel SDK library: ${DXL_LIB}")
  target_link_libraries(dxl_node ${DXL_LIB})
else()
  message(WARNING "Dynamixel SDK not found system-wide. "
                  "Install it (sudo make install) or vendor it and link manually.")
endif()

# --- install targets ---
install(TARGETS tracker_node dxl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config YAML
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
