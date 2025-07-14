from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Update these as needed
    hros5_description_pkg = get_package_share_directory('hros5_description')
    hros5_gazebo_pkg = get_package_share_directory('hros5_gazebo')    

    robot_description_file = os.path.join(hros5_gazebo_pkg, 'urdf/hands/left_hand_gazebo.xacro')
    controller_yaml = os.path.join(hros5_gazebo_pkg, 'config', 'left_hand_controllers.yaml')
    world_path = os.path.join(hros5_gazebo_pkg, 'worlds', 'empty.sdf')

    # Spawn with ROS 2 Harmonic integration (ros_gz_sim)
    return LaunchDescription([
        # Launch Gazebo Harmonic with a world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(robot_description_file).read()}],
            output='screen'
        ),
        # ros2_control node (robot controllers)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': open(robot_description_file).read()},
                        controller_yaml],
            output='screen'
        ),
        # Controller spawners
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen'
        ),
        # Optionally: Spawn entity into simulation using ros_gz
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'left_hand',
                '-file', robot_description_file
            ],
            output='screen'
        ),
    ])
