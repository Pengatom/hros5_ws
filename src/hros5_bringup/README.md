# hros5_bringup

High-level bringup package for the HR-OS5 robot.

This package provides combined launch files for quick testing, visualization,
and partial robot bringup in RViz or Gazebo.

## Features

- RViz bringup for visualizing URDF models
- Gazebo bringup for hand/arm tests
- Combined launch files for testing teleop, description, and control

## Directory structure

```text
hros5_bringup/
├── CMakeLists.txt
├── launch
│   ├── gazebo_launch.py
│   ├── hros5_rviz_launch.py
│   ├── rviz_launch.py
│   └── test_hand_control.launch.py
├── package.xml
└── src
    └── hros5_bringup_node.cpp
Status
This package is evolving toward a full hardware + simulation bringup
(P1–P3 in the capability roadmap).
