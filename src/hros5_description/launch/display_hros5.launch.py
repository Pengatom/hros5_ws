from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('hros5_description')

    # Launch arguments
    use_dae_meshes = LaunchConfiguration('use_dae_meshes')
    use_gui = 'false' #LaunchConfiguration('gui') # fix this to use joint_state_publisher_gui

    meshes_xacro_file = os.path.join(pkg_share, 'urdf', 'hros5_visuals_collisions_endoskeleton.xacro')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hros5.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'hros5.rviz')
    visual_mesh_ext = PythonExpression([
        "'dae' if '", use_dae_meshes, "' == 'true' else 'stl'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_dae_meshes',
            default_value='true',
            description='Use DAE visuals in RViz (falls back to STL if set false)'
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
                        ' meshes_xacro_filename:=', meshes_xacro_file,
                        ' visual_mesh_ext:=', visual_mesh_ext
                    ]),
                    value_type=str
                )
            }]
        ),

        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        ),

        Node(
            condition=UnlessCondition(use_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
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
