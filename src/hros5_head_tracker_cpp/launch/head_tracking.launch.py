import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('hros5_head_tracker_cpp'),
        'config',
        'head_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense',
            parameters=[{
                'camera_name': 'realsense',     # topics will be /camera/realsense/color/image_raw
                'rgb_camera.profile': '640x480x30',
                'enable_depth': False,
            }]
        ),
        Node(
            package='hros5_head_tracker_cpp',
            executable='tracker_node',
            output='screen',
            parameters=[cfg]
        ),
        Node(
            package='hros5_dynamixel_bridge',
            executable='head_dxl_node',
            name='head_dxl_node',
            output='screen',
            parameters=[cfg]
        ),
    ])
