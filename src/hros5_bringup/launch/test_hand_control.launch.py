from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare("hros5_description"),
            "urdf",
            "test_hand_control.urdf.xacro"
        ])
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("hros5_description"),
        "config",
        "hand_test_controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_content},
                controller_config
            ],
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("hros5_description"),
                "rviz",
                "hros5_hand.rviz"
            ])]
        ),
        Node(
            package="rqt_joint_trajectory_controller",
            executable="rqt_joint_trajectory_controller",
            name="rqt_slider",
            output="screen"
        )
    ])
