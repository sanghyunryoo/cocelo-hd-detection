# Weldline 3D Localization (ROS 2 / C++)

Production-oriented RGB-D weld-line localization package. The detector is a native C++17 ROS 2 node using OpenCV DNN for ONNX inference; no Python runtime is part of the new YOLO 3D deployment path. Its bringup launches the official Intel RealSense ROS 2 driver (`realsense2_camera`) alongside the detector.

## Runtime contract

The node uses synchronized RGB, aligned depth, and camera intrinsics as an input bundle. A bounded latest-frame buffer is processed by a 30 Hz timer, preventing a slow inference cycle from accumulating camera latency.

| Interface | Type | Default | Purpose |
| --- | --- | --- | --- |
| Input RGB | `sensor_msgs/Image` | `/camera/camera/color/image_raw` | YOLO inference |
| Input depth | `sensor_msgs/Image` | `/camera/camera/aligned_depth_to_color/image_raw` | 3D projection (`16UC1` mm or `32FC1` m) |
| Input intrinsics | `sensor_msgs/CameraInfo` | `/camera/camera/color/camera_info` | Pixel-to-camera projection |
| Nav2 goal | `geometry_msgs/PoseStamped` | `/goal_pose` | Center pose, yaw aligned to the detected weld line |
| Debug point | `geometry_msgs/PointStamped` | `/weldline_yolo/debug/center_point` | Exact 3D center coordinate |
| Debug markers | `visualization_msgs/MarkerArray` | `/weldline_yolo/debug/markers` | Center sphere and 3D line |
| Debug image | `sensor_msgs/Image` | `/weldline_yolo/debug/annotated_image` | Bounding box, line, and transformed coordinate |

Coordinates are transformed through TF2 into `output_frame` (default `map`) before publication. A missing transform causes that frame to be discarded, never published with a false frame ID. Set `nav2_planar_goal: true` when Nav2 must receive `z = 0`.

## Build and run

```bash
./build.sh
./launch.sh
```

The launch wrapper uses `/opt/ros/${ROS_DISTRO:-foxy}` and pins CMake's ROS tooling to `/usr/bin/python3`. Common overrides:

```bash
WEIGHTS=/opt/models/weldline.onnx ./launch.sh output_frame:=base_link processing_rate_hz:=30.0
```

`launch.sh` starts `realsense2_camera/rs_launch.py` with color, depth, and aligned depth enabled. The default `camera_name:=camera` produces the input topics configured in the YAML file. Select a physical camera explicitly when several are connected:

```bash
./launch.sh camera_serial_no:=123456789012
```

For an externally managed RealSense driver, keep the detector only:

```bash
./launch.sh start_realsense:=false
```

ROS 2 parameters are in [config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml). The launch file is XML-only; application logic resides in [src/yolo_weldline_3d_node.cpp](src/yolo_weldline_3d_node.cpp).

## Debian packages: AMD and ARM

Run the package build on the target architecture:

```bash
./scripts/build_deb.sh
```

The script detects Debian architecture with `dpkg --print-architecture`, accepts only `amd64`, `arm64`, or `armhf`, generates ROS Debian metadata using `bloom`, and places the native `.deb` under `dist/`. The generated package declares `ros-${ROS_DISTRO}-realsense2-camera` as a Debian dependency, so installing the weldline `.deb` also installs the official RealSense driver and its `librealsense` dependencies from the configured ROS apt repository. It intentionally refuses to overwrite an existing `debian/` directory; review or remove generated metadata before a new generation.

Required build tooling: `python3-bloom`, `dpkg-dev`, the selected ROS distribution, and the package dependencies resolved with `rosdep`.

## Deployment notes

- Use a valid ONNX model supported by the OpenCV version installed on the target. The supplied model must be verified during CI; a corrupt or unsupported ONNX file is rejected at startup with a fatal diagnostic.
- Keep the model and ROS distribution identical across AMD/ARM release builds for reproducible detector behavior.
- Connect the sensor optical frame to `map` (or configure `output_frame` to a valid TF frame) before enabling Nav2 navigation.
# cocelo-hd-detection
