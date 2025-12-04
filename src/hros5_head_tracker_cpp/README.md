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
```

## Configuration

`config/head_params.yaml` is the single source for head tracking parameters. It references shared configs from `hros5_control`:
- `config_package: hros5_control`
- `config_file: config/joints/head.yaml` (HeadYaw/HeadPitch IDs, limits, offsets)
- `bus_config: config/dxl_bus.yaml` (DXL device, baud, protocol)

Key tracker parameters:
- `camera_topic` (default `/camera/realsense/color/image_raw`)
- `fov_h_deg`, `fov_v_deg`, `kp`
- `pan_*` / `tilt_*` limits and offsets
- DXL step size and invert flags (`max_step_deg`, `invert_pan`, `invert_tilt`)

## Running

```
ros2 launch hros5_head_tracker_cpp head_tracking.launch.py
```

This starts:
- `realsense2_camera_node` with `camera_name=realsense` (topics under `/camera/realsense`)
- `tracker_node` consuming the RGB topic from `head_params.yaml`
- `head_dxl_node` (from `hros5_dynamixel_bridge`) using the shared joint and bus configs

## Troubleshooting

- If you see `No images received`, run `ros2 topic list` and align `camera_topic` in `config/head_params.yaml`.
- To confirm parameters are loaded: `ros2 param dump /tracker_node` and `ros2 param dump /head_dxl_node`.
Status
Active (P1–P2).
Used for early perception experiments and head tracking.
