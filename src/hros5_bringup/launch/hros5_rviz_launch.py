
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the 'gui' argument
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher GUI'
    )

    # Paths
    hros5_description_path = get_package_share_directory('hros5_description')
    urdf_launch_path = os.path.join(hros5_description_path, 'launch', 'hros5_urdf.launch.py')
    rviz_config_path = os.path.join(hros5_description_path, 'config', 'hros5.rviz')

    # Include URDF launch file
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch_path)
    )

    # Nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': LaunchConfiguration('gui')}],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        urdf_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

