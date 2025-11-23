from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('hros5_teleop'),
        'config',
        'ps4_hands_teleop.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 30.0,
        }]
    )

    hands_teleop = Node(
        package='hros5_teleop',
        executable='hands_teleop_node',
        name='hands_teleop_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        joy_node,
        hands_teleop,
    ])
