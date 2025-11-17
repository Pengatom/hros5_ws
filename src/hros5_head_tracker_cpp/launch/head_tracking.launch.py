from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',                 # <— adds /camera prefix
            name='realsense',
            parameters=[{
                'camera_name': 'camera',        # <— ensures /camera/color/image_raw
                'rgb_camera.profile': '640x480x30',
                'enable_depth': False,
            }]
        ),
        Node(
            package='hros5_head_tracker_cpp',
            executable='tracker_node',
            name='tracker',
            output='screen',
            parameters=['config/head_params.yaml']
        ),
        Node(
            package='hros5_head_tracker_cpp',
            executable='dxl_node',
            name='dxl',
            output='screen',
            parameters=['config/head_params.yaml']
        ),
    ])