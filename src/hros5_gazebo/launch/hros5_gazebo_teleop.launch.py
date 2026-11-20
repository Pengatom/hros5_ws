"""Gazebo + PS4 teleop: the same hros5_teleop nodes as on hardware, driving the sim robot.

  ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py limb:=left_arm

The real *_dxl_node executables are replaced by sim_dxl_bridge.py, which listens on the same
/hros5/<limb>/target_angles_deg topics. All teleop nodes use the same sticks, so (as on
hardware) one limb is driven at a time. Legs default to fixed_base:=true (robot on a stand),
because moving one leg while standing tips the robot over.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# limb argument -> (teleop executable/node name, config file in hros5_teleop/config)
TELEOP = {
    'head': ('head_teleop_node', 'ps4_head_teleop.yaml'),
    'left_arm': ('left_arm_teleop_node', 'ps4_left_arm_teleop.yaml'),
    'right_arm': ('right_arm_teleop_node', 'ps4_right_arm_teleop.yaml'),
    'left_leg': ('left_leg_teleop_node', 'ps4_left_leg_teleop.yaml'),
    'right_leg': ('right_leg_teleop_node', 'ps4_right_leg_teleop.yaml'),
    'hands': ('hands_teleop_node', 'ps4_hands_teleop.yaml'),
}


def generate_launch_description():
    limb = LaunchConfiguration('limb')
    fixed_base_arg = LaunchConfiguration('fixed_base')
    spawn_z_arg = LaunchConfiguration('spawn_z')
    headless = LaunchConfiguration('headless')
    joy_dev = LaunchConfiguration('joy_device_id')

    fixed_base = PythonExpression([
        "'true' if ('", fixed_base_arg, "' == 'true' or ('", fixed_base_arg, "' == 'auto' and '",
        limb, "' in ('left_leg', 'right_leg'))) else 'false'"])
    spawn_z = PythonExpression([
        "'", spawn_z_arg, "' if '", spawn_z_arg, "' != 'auto' else ('0.6' if '", fixed_base,
        "' == 'true' else '0.40')"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hros5_gazebo'), 'launch', 'hros5_gazebo.launch.py')),
        launch_arguments={
            'fixed_base': fixed_base,
            'spawn_z': spawn_z,
            'headless': headless,
        }.items())

    teleop_nodes = [
        Node(
            condition=IfCondition(PythonExpression(["'", limb, "' == '", key, "'"])),
            package='hros5_teleop',
            executable=exe,
            name=exe,
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('hros5_teleop'), 'config', cfg])],
        )
        for key, (exe, cfg) in TELEOP.items()
    ]

    return LaunchDescription([
        DeclareLaunchArgument('limb', default_value='head',
                              description='Which teleop to run: ' + ', '.join(TELEOP)),
        DeclareLaunchArgument('fixed_base', default_value='auto',
                              description='true/false, or auto = true for legs, false otherwise'),
        DeclareLaunchArgument('spawn_z', default_value='auto',
                              description='auto = 0.6 on the stand, 0.40 standing'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo server only, no GUI'),
        DeclareLaunchArgument('joy_device_id', default_value='0',
                              description='Joystick index for joy_node (0 = /dev/input/js0)'),
        gazebo,
        # One joy_node for everything (the per-limb teleop launch files each start their own).
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': joy_dev, 'deadzone': 0.05, 'autorepeat_rate': 30.0}],
        ),
        Node(
            package='hros5_gazebo',
            executable='sim_dxl_bridge.py',
            name='sim_dxl_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        *teleop_nodes,
    ])
