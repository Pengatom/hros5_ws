from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('hros5_dynamixel_bridge'),
        'config',
        'left_leg_dxl.yaml'
    ])

    return LaunchDescription([
        Node(
            package='hros5_dynamixel_bridge',
            executable='left_leg_dxl_node',
            name='left_leg_dxl_node',
            output='screen',
            parameters=[config],
        ),
    ])
