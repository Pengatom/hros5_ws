from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("hros5_demos"),
            "urdf",
            "test_urdf.xacro",
        ])
    ])
    print("robot_description_command:", robot_description_content)
    robot_description = {"robot_description": robot_description_content}

    yaml_file = PathJoinSubstitution([
        FindPackageShare("hros5_demos"),
        "config",
        "two_mx28.yaml",
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("hros5_demos"),
        "config",
        "dynamixel_controllers.yaml",
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, yaml_file, controllers_yaml],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dynamixel_controller"],
        )
    ])
