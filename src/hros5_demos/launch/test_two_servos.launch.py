from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("hros5_demos"),
            "urdf",
            "test_urdf.xacro",
        ])
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Load controllers YAML
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("hros5_demos"),
        "config",
        "dynamixel_controllers.yaml",
    ])

    return LaunchDescription([
        # Robot State Publisher (publishes /robot_description topic)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        # ros2_control node with hardware plugin (hardware params come from URDF)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_yaml],
            output="screen"
        ),
        # Spawner nodes
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
