from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('hros5_control')
    config_dir = os.path.join(pkg_path, 'config')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(config_dir, 'left_leg_controller.yaml'),
                os.path.join(config_dir, 'right_leg_controller.yaml'),
                os.path.join(config_dir, 'dynamixel.yaml'),
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_leg_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_leg_controller'],
            output='screen'
        )
    ])

