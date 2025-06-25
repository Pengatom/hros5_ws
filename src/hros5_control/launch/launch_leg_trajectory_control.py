from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_pkg = get_package_share_directory('hros5_description')
    control_pkg = get_package_share_directory('hros5_control')

    xacro_file = os.path.join(description_pkg, 'urdf', 'hros5.xacro')
    default_meshes_path = os.path.join(description_pkg, 'urdf', 'hros5_visuals_collisions_endoskeleton.xacro')
    config_dir = os.path.join(control_pkg, 'config')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' meshes_xacro_filename:=', default_meshes_path
        ]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(config_dir, 'dynamixel.yaml'),  # ← Add this line
                os.path.join(config_dir, 'left_leg_controller.yaml'),
                os.path.join(config_dir, 'right_leg_controller.yaml'),
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

