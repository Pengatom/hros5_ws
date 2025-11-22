from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('hros5_teleop'),
        'config',
        'ps4_head_teleop.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # you can tweak deadzone/autorepeat here too if you want
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    head_teleop = Node(
        package='hros5_teleop',
        executable='head_teleop_node',
        name='head_teleop_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        joy_node,
        head_teleop,
    ])
