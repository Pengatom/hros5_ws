from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare("hros5_description").find("hros5_description")

    # Expand the minimal hands-only xacro
    robot_description_content = Command([
        "xacro ",
        os.path.join(pkg_share, "urdf", "display_hands.urdf.xacro")
    ])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument(
            "dynamixel_config",
            default_value=os.path.join(
                FindPackageShare("hros5_control").find("hros5_control"),
                "config", "dynamixel.yaml"
            ),
            description="YAML config for dynamixel_hardware_interface",
        ),

        # Publish robot description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # Launch ros2_control with just the YAML
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[LaunchConfiguration("dynamixel_config")],
            output="screen"
        ),

        # Spawn joint_state_broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )
    ])
