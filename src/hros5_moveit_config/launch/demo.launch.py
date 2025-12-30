from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "hros5", package_name="hros5_moveit_config"
    ).to_moveit_configs()

    launch_pkg_path = moveit_config.package_path / "launch"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "db",
                default_value="false",
                description="Start the MongoDB warehouse (disabled by default)",
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="Enable debug mode for MoveIt nodes",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt config",
            ),
            # Static transforms for any virtual joints
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_pkg_path / "static_virtual_joint_tfs.launch.py")
                )
            ),
            # Robot description publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_pkg_path / "rsp.launch.py"))
            ),
            # ros2_control (fake system) before move_group so controllers are known
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml")],
                remappings=[("/controller_manager/robot_description", "/robot_description")],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_pkg_path / "spawn_controllers.launch.py"))
            ),
            # MoveGroup (planning + execution)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_pkg_path / "move_group.launch.py"))
            ),
            # RViz with MoveIt plugin
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_pkg_path / "moveit_rviz.launch.py")),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            # Optional warehouse DB
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_pkg_path / "warehouse_db.launch.py")),
                condition=IfCondition(LaunchConfiguration("db")),
            ),
        ]
    )
