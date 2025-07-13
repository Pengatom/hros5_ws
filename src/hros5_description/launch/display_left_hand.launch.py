from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_meshes_arg = DeclareLaunchArgument(
        name="use_meshes",
        default_value="true",
        description="Whether to include visual/collision meshes"
    )

    meshes_xacro_arg = DeclareLaunchArgument(
        name="meshes_xacro_filename",
        default_value=PathJoinSubstitution([
            FindPackageShare("hros5_description"),
            "urdf",
            "hros5_visuals_collisions_endoskeleton.xacro"
        ])
    )

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("hros5_description"),
                "urdf",
                "hands",
                "left_hand_test.xacro"
            ]),
            " ",
            "use_meshes:=true ",
            "meshes_xacro_filename:=",
            LaunchConfiguration("meshes_xacro_filename")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("hros5_description"),
            "rviz",
            "left_hand.rviz"
        ])]
    )

    return LaunchDescription([
        use_meshes_arg,
        meshes_xacro_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
