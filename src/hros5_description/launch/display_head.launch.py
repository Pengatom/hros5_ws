from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare('hros5_description').find('hros5_description')
    
    # ✅ This defines the correct path!
    xacro_file = os.path.join(pkg_share, 'urdf', 'display_head.urdf.xacro')

    # ✅ Print to confirm
    print("Using xacro file:", xacro_file)

    # ✅ Load the xacro
    robot_description_content = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])

