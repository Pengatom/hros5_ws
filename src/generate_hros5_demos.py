# generate_hros5_demos.py
# Run this script from inside your ROS 2 workspace (e.g., ~/hros5_ws/src)

import os

print("📦 Creating a clean ROS 2 Jazzy hros5_demos package")

base_path = os.path.abspath("hros5_demos")
folders = ["launch", "config", "description"]

files = {
    "package.xml": """<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>hros5_demos</name>
  <version>0.1.0</version>
  <description>Test and prototype components for hros5 (ROS 2 Jazzy)</description>
  <maintainer email=\"you@example.com\">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>position_controllers</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>dynamixel_hardware</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
""",

    "CMakeLists.txt": """cmake_minimum_required(VERSION 3.10)
project(hros5_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
""",

    "launch/test_single_servo.launch.py": """from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hros5_demos')
    urdf_path = os.path.join(pkg_share, 'description', 'single_servo.urdf')
    yaml_path = os.path.join(pkg_share, 'config', 'servo_control.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                yaml_path
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint1_position_controller'],
        )
    ])
""",

    "config/servo_control.yaml": """controller_manager:
  ros__parameters:
    update_rate: 50

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint1_position_controller:
      type: position_controllers/JointPositionController
      joint: joint1

dynamixel_hardware:
  ros__parameters:
    use_dummy: false
    serial_port: \"/dev/ttyUSB0\"
    baudrate: 1000000
    joints:
      - id: 1
        name: joint1
        model: \"MX-28\"
""",

    "description/single_servo.urdf": """<robot name=\"single_dynamixel\">
  <link name=\"base_link\"/>
  <joint name=\"joint1\" type=\"revolute\">
    <parent link=\"base_link\"/>
    <child link=\"servo_link\"/>
    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
    <axis xyz=\"0 0 1\"/>
    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"1.0\" velocity=\"1.0\"/>
  </joint>
  <link name=\"servo_link\"/>
</robot>
"""
}

# Create folders
os.makedirs(base_path, exist_ok=True)
for f in folders:
    os.makedirs(os.path.join(base_path, f), exist_ok=True)

# Write files
for filepath, content in files.items():
    with open(os.path.join(base_path, filepath), "w") as f:
        f.write(content)

print("✅ Done: hros5_demos (ament_cmake, no catkin) created at", base_path)

