from launch import LaunchDescription
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
