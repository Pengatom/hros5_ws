# hros5_head_tracker_cpp

Camera-based head tracking for HR-OS5 using OpenCV.

Allows selecting an ROI and commanding the pan/tilt servos to follow a target.

## Directory structure

```text
hros5_head_tracker_cpp/
├── CMakeLists.txt
├── config
│   └── head_params.yaml
├── launch
│   └── head_tracking.launch.py
├── package.xml
└── src
    ├── opencv_window_test.cpp
    └── tracker_node.cpp
Status
Active (P1–P2).
Used for early perception experiments and head tracking.