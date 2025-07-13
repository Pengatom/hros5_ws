from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("hros5_dynamixel_bridge"),
        "config",
        "joints.yaml"
    ])
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        PathJoinSubstitution([
                            FindPackageShare("hros5_description"),
                            "urdf",
                            "hands",
                            "left_hand_test.xacro"
                        ]),
                        " use_meshes:=true",
                        " meshes_xacro_filename:=",
                        PathJoinSubstitution([
                            FindPackageShare("hros5_description"),
                            "urdf",
                            "hros5_visuals_collisions_endoskeleton.xacro"
                        ])
                    ]),
                    value_type=str
                )
            }]
        ),
        Node(
            package="hros5_dynamixel_bridge",
            executable="dynamixel_bridge_node",
            output="screen",
            parameters=[
                {"config_file": config_file},
                {"port": "/dev/ttyUSB0"},
                {"baudrate": 1000000}
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("hros5_description"),
                "rviz",
                "left_hand.rviz"
            ])]
        )        
    ])
# This launch file starts the robot state publisher and the dynamixel bridge node.
# It uses a YAML configuration file for joint settings and specifies the serial port and baud rate.
# The robot state publisher will publish the robot's state based on the URDF description.
# Ensure that the config file path and serial port are correct for your system.
# The baud rate is set to 1000000, which is common for Dynamixel servos.
# This launch file is essential for initializing the dynamixel bridge in a ROS2 environment.
# It allows for real-time control and monitoring of the robot's joints through the Dynamixel SDK.
# The node will handle communication with the Dynamixel servos, converting between ROS messages and
# the Dynamixel protocol. It will also manage the state of each joint, including position and
# velocity, and publish this information to the ROS2 ecosystem.
# The robot state publisher will also ensure that the robot's kinematic state is accurately represented
# in the ROS2 ecosystem, allowing for visualization and control in tools like RViz2 and
# MoveIt2. This setup is crucial for simulating and controlling the robot effectively.