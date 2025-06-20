from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('hros5_description')

    # Launch arguments
    meshes_xacro = LaunchConfiguration('meshes_xacro')
    use_gui = LaunchConfiguration('gui')

    default_meshes_path = os.path.join(pkg_share, 'urdf', 'hros5_visuals_collisions_endoskeleton.xacro')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hros5.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'hros5.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'meshes_xacro',
            default_value=default_meshes_path,
            description='Path to mesh xacro file'
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch joint_state_publisher GUI'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ', xacro_file,
                        ' meshes_xacro_filename:=', meshes_xacro
                    ]),
                    value_type=str
                )
            }]
        ),

        Node(
            package='joint_state_publisher_gui' if use_gui == 'true' else 'joint_state_publisher',
            executable='joint_state_publisher_gui' if use_gui == 'true' else 'joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])

