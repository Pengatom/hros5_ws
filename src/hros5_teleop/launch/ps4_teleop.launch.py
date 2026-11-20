"""Whole-robot PS4 teleop: one joy_node + ps4_teleop_node (modes + dead-man).

  ros2 launch hros5_teleop ps4_teleop.launch.py

Drives whatever listens on /hros5/<limb>/target_angles_deg: the real *_dxl_node executables,
or hros5_gazebo's sim_dxl_bridge (see hros5_gazebo_teleop.launch.py).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_device_id', default_value='0',
                              description='Joystick index for joy_node (0 = /dev/input/js0)'),
        DeclareLaunchArgument('initial_mode', default_value='head',
                              description='Mode at start: head, left_arm, right_arm, hands, left_leg, right_leg, legs'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                'deadzone': 0.0,          # deadzones are applied in ps4_teleop_node (deadzones.yaml)
                'autorepeat_rate': 20.0,  # keeps /joy alive while a stick is held still (dead-man timeout)
            }],
        ),
        Node(
            package='hros5_teleop',
            executable='ps4_teleop_node',
            name='ps4_teleop_node',
            output='screen',
            parameters=[{'initial_mode': LaunchConfiguration('initial_mode')}],
        ),
    ])
