import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_camera = LaunchConfiguration("start_camera")
    start_dxl = LaunchConfiguration("start_dxl")
    start_watchdog = LaunchConfiguration("start_watchdog")
    start_minimal_teleop = LaunchConfiguration("start_minimal_teleop")

    description_share = FindPackageShare("hros5_description").find("hros5_description")
    control_share = FindPackageShare("hros5_control").find("hros5_control")
    dxl_share = FindPackageShare("hros5_dynamixel_bridge").find("hros5_dynamixel_bridge")
    teleop_share = FindPackageShare("hros5_teleop").find("hros5_teleop")

    robot_description = {
        "robot_description": xacro.process_file(
            os.path.join(description_share, "urdf", "hros5.xacro")
        ).toxml()
    }

    controllers_config = os.path.join(control_share, "config", "hardware_controllers.yaml")
    joint_limits_config = os.path.join(control_share, "config", "joint_limits.yaml")
    watchdog_config = os.path.join(control_share, "config", "safety_watchdog.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_config,
            joint_limits_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "60",
            ],
            output="screen",
        )
        for controller in [
            "joint_state_broadcaster",
            "head_controller",
            "left_arm_controller",
            "right_arm_controller",
            "left_hand_controller",
            "right_hand_controller",
            "left_leg_controller",
            "right_leg_controller",
        ]
    ]

    dxl_nodes = GroupAction(
        actions=[
            Node(
                package="hros5_dynamixel_bridge",
                executable="head_dxl_node",
                name="head_dxl",
                output="screen",
                parameters=[os.path.join(dxl_share, "config", "head_dxl.yaml")],
            ),
            Node(
                package="hros5_dynamixel_bridge",
                executable="left_arm_dxl_node",
                name="left_arm_dxl",
                output="screen",
                parameters=[os.path.join(dxl_share, "config", "left_arm_dxl.yaml")],
            ),
            Node(
                package="hros5_dynamixel_bridge",
                executable="right_arm_dxl_node",
                name="right_arm_dxl",
                output="screen",
                parameters=[os.path.join(dxl_share, "config", "right_arm_dxl.yaml")],
            ),
            Node(
                package="hros5_dynamixel_bridge",
                executable="left_leg_dxl_node",
                name="left_leg_dxl",
                output="screen",
                parameters=[os.path.join(dxl_share, "config", "left_leg_dxl.yaml")],
            ),
            Node(
                package="hros5_dynamixel_bridge",
                executable="right_leg_dxl_node",
                name="right_leg_dxl",
                output="screen",
                parameters=[os.path.join(dxl_share, "config", "right_leg_dxl.yaml")],
            ),
        ],
        condition=IfCondition(start_dxl),
    )

    sensors = GroupAction(
        actions=[
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense",
                output="screen",
                parameters=[
                    {
                        "camera_name": "realsense",
                        "rgb_camera.profile": "640x480x30",
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "enable_gyro": True,
                        "enable_accel": True,
                    }
                ],
            )
        ],
        condition=IfCondition(start_camera),
    )

    logger = get_logger("hros5_hardware_launch")

    def package_available(package_name: str) -> bool:
        try:
            get_package_share_directory(package_name)
            return True
        except PackageNotFoundError:
            return False

    def maybe_launch_watchdog(context):
        """Start watchdog only if requested and available to avoid launch failure when missing."""
        start = start_watchdog.perform(context).lower()
        if start not in ("true", "1", "yes"):
            return []
        if not package_available("diagnostic_aggregator"):
            logger.warning("diagnostic_aggregator not installed; skipping watchdog/diagnostics.")
            return []
        return [
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="hros5_watchdog",
                output="screen",
                parameters=[watchdog_config],
            )
        ]

    minimal_teleop = GroupAction(
        actions=[
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
            ),
            Node(
                package="hros5_teleop",
                executable="head_teleop_node",
                name="head_teleop_node",
                output="screen",
                parameters=[os.path.join(teleop_share, "config", "ps4_head_teleop.yaml")],
            ),
        ],
        condition=IfCondition(start_minimal_teleop),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (off for hardware).",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Start onboard RealSense (RGB-D + IMU).",
            ),
            DeclareLaunchArgument(
                "start_dxl",
                default_value="true",
                description="Start all Dynamixel bridge nodes.",
            ),
            DeclareLaunchArgument(
                "start_watchdog",
                default_value="true",
                description="Start diagnostic watchdog/aggregator.",
            ),
            DeclareLaunchArgument(
                "start_minimal_teleop",
                default_value="false",
                description="Enable bench/bring-up teleop (joy + head).",
            ),
            robot_state_publisher,
            ros2_control_node,
            *controller_spawners,
            dxl_nodes,
            sensors,
            OpaqueFunction(function=maybe_launch_watchdog),
            minimal_teleop,
        ]
    )
