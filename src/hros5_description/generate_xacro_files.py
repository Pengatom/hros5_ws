#!/usr/bin/env python3
"""
Script to generate the ROS 2 Jazzy package structure for hros5_description.
"""
import os

def main():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))

    dirs = ['urdf/common', 'urdf/base', 'urdf/head', 'urdf/arms', 'urdf/legs', 'launch', 'rviz']
    for d in dirs:
        os.makedirs(os.path.join(pkg_dir, d), exist_ok=True)

    control_path = os.path.join(pkg_dir, 'urdf', 'hros5_control.xacro')
    control_content = ''
    if os.path.isfile(control_path):
        with open(control_path, 'r') as f:
            control_content = f.read()

    def extract_macro(name):
        start = control_content.find(f'<xacro:macro name="{name}"')
        if start < 0:
            return ''
        end = control_content.find('</xacro:macro>', start) + len('</xacro:macro>')
        return control_content[start:end]

    head_macro = extract_macro('head_trasmissions')
    arm_macro = extract_macro('arm_joints_transmissions')
    leg_macro = extract_macro('leg_joints_transmissions')

    files = {
        'launch/display.launch.py': '''from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('hros5_description')
    robot_desc = Command(['xacro', PathJoinSubstitution([pkg_share, 'urdf', 'hros5.urdf.xacro'])])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'hros5.rviz'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}], output='screen'
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen'
        ),
    ])'''
    }

    for rel_path, content in files.items():
        abs_path = os.path.join(pkg_dir, rel_path)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
        with open(abs_path, 'w') as f:
            f.write(content)
        print(f"Generated {rel_path}")

if __name__ == '__main__':
    main()

