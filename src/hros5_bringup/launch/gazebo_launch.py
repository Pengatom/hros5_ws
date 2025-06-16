from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    hros5_description_path = get_package_share_directory('hros5_description')
    urdf_path = os.path.join(hros5_description_path, 'urdf', 'hros5.urdf')

    return LaunchDescription([
        # Launch gz sim
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '--verbose'],
            output='screen'
        ),

        # Spawn robot using gz service
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'hros5',
                '-file', urdf_path,
                '-x', '0', '-y', '0', '-z', '1'
            ],
            output='screen'
        )
    ])

