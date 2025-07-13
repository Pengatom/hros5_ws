import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('hros5_dynamixel_bridge'),
        'config',
        'joints.yaml'
    )
    return LaunchDescription([
        Node(
            package='hros5_dynamixel_bridge',
            executable='dynamixel_bridge_node',
            output='screen',
            parameters=[
                {'config_file': config_file},
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 1000000}
            ]
        )
    ])
# This launch file starts the hros5_dynamixel_bridge node with specified parameters.
# It uses a YAML configuration file for joint settings and specifies the serial port and baud rate.
# Make sure to adjust the parameters according to your setup.
# The node will publish JointState messages and subscribe to JointTrajectory messages.
# It is designed to work with MoveIt2, RViz2, and Gazebo for simulation and control of the robot.
# Ensure that the config file path and serial port are correct for your system.
# The baud rate is set to 1000000, which is common for Dynamixel servos.
# This launch file is essential for initializing the dynamixel bridge in a ROS2 environment.
# It allows for real-time control and monitoring of the robot's joints through the Dynamixel SDK.
# The node will handle communication with the Dynamixel servos, converting between ROS messages and
# the Dynamixel protocol. It will also manage the state of each joint, including position and
# velocity, and publish this information to the ROS2 ecosystem.