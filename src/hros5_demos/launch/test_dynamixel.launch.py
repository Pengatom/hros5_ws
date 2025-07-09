from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare("hros5_demos")
    urdf_file = PathJoinSubstitution([pkg, "urdf", "test_robot.urdf.xacro"])
    yaml_file = PathJoinSubstitution([pkg, "config", "dynamixel_test_controllers.yaml"])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_file])
            }]
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": Command(["xacro ", urdf_file])},
                yaml_file
            ],
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
            arguments=["position_controller"],
        )
    ])
