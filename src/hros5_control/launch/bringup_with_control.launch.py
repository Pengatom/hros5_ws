from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_pkg = get_package_share_directory('hros5_description')
    control_pkg = get_package_share_directory('hros5_control')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_pkg, 'launch', 'display.launch.py')
            )
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(control_pkg, 'config', 'left_leg_controller.yaml'),
                os.path.join(control_pkg, 'config', 'dynamixel.yaml'),
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
            arguments=['left_leg_controller'],
        )
    ])

