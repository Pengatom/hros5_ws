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
find_package(dynamixel_sdk REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# --- tracker node (OpenCV) ---
add_executable(tracker_node src/tracker_node.cpp)
ament_target_dependencies(tracker_node
  rclcpp sensor_msgs std_msgs image_transport cv_bridge)
target_link_libraries(tracker_node ${OpenCV_LIBS})

# --- OpenCV window test ---
add_executable(opencv_window_test src/opencv_window_test.cpp)
ament_target_dependencies(opencv_window_test rclcpp OpenCV)

# --- Dynamixel node (C++) ---
add_executable(dxl_node src/dxl_node.cpp)
ament_target_dependencies(dxl_node rclcpp std_msgs dynamixel_sdk)

# --- install targets ---
install(TARGETS tracker_node dxl_node opencv_window_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config YAML
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
