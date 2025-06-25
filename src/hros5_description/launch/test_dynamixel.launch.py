from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare("hros5_description").find("hros5_description")

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_file",
            default_value=os.path.join(pkg_share, "urdf", "head", "display_head.urdf.xacro"),
            description="Path to URDF/Xacro file",
        ),
        DeclareLaunchArgument(
            "dynamixel_config",
            default_value=os.path.join(pkg_share, "config", "dynamixel.yaml"),
            description="YAML config for dynamixel_hardware_interface",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                LaunchConfiguration("urdf_file"),
                LaunchConfiguration("dynamixel_config")
            ],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )
    ])

