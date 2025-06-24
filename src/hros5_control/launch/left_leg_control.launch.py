from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('hros5_control')
    config = os.path.join(pkg, 'config', 'left_leg_controller.yaml')
    dynamixel = os.path.join(pkg, 'config', 'dynamixel.yaml')  # optional

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config, dynamixel],
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
            arguments=['left_leg_controller'],
        )
    ])

