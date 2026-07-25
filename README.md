# Weldline 3D Localization (ROS 2 / C++)

Production-oriented RGB-D weld-line localization package. The detector is a native C++17 ROS 2 node using ONNX Runtime for ONNX inference and OpenCV only for image processing; no Python runtime is part of the new YOLO 3D deployment path. Its bringup launches the official Intel RealSense ROS 2 driver (`realsense2_camera`) alongside the detector.

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
source /opt/ros/<your_ros_distro>/setup.bash  # optional if exactly one ROS 2 distro is installed
./build.sh
./launch.sh
```

Before the first launch, set a unique DDS domain and the physical RealSense USB topology in [config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml). `usb_port_id` is deliberately required when the integrated driver starts, preventing an arbitrary camera from being selected on multi-camera systems.

```yaml
ros_domain_id: 20
usb_port_id: "2-1.3"
```

Inspect USB topology with the visualizer below, then use the `usb_port_id` overlaid on the camera image you want the detector to own. Environment variables override the deployment file: `ROS_DOMAIN_ID=20 USB_PORT_ID=2-1.3 ./launch.sh`.

The scripts use the sourced `ROS_DISTRO`; if none is sourced, they automatically select the sole distribution under `/opt/ros`. When several distributions are installed, source the intended one (or set `ROS_DISTRO`) to avoid an ambiguous build. CMake's ROS tooling is pinned to `/usr/bin/python3`. Common overrides:

```bash
WEIGHTS=/opt/models/weldline.onnx ./launch.sh output_frame:=base_link processing_rate_hz:=30.0
```

An example COCO person detector is included as `weights/person_yolov5n.onnx`. It uses the same `1x3x640x640` RGB input layout; run it with class filtering enabled:

```bash
WEIGHTS=$PWD/weights/person_yolov5n.onnx ./launch.sh target_class_id:=0
```

`./build.sh` prepares an architecture-matched ONNX Runtime C++ SDK when the system does not already provide one, then validates the configured ONNX model with ONNX Runtime on the target machine. This avoids OpenCV DNN parser failures such as unsupported `TopK` nodes on Jetson while still failing the build if the model itself cannot be loaded. Override the model used for validation with `WELDLINE_ONNX_MODEL=/opt/models/weldline.onnx ./build.sh`.

`launch.sh` starts `realsense2_camera/rs_launch.py` with color, depth, and aligned depth enabled. The default `camera_name:=camera` produces the input topics configured in the YAML file. Select a physical camera explicitly when several are connected:

```bash
./launch.sh camera_serial_no:=123456789012
```

For an externally managed RealSense driver, keep the detector only:

```bash
./launch.sh start_realsense:=false
```

## RealSense diagnostics and visualization

The Python visualizer is separate from the production C++ detector and intentionally has no ROS 2 dependency. Run it before `./launch.sh` when you need to decide which physical RealSense should be assigned to `usb_port_id` in `yolo_weldline_3d.yaml`.

It uses `pyrealsense2` to discover and open every connected camera directly, logs each serial number and USB topology, overlays the same identifiers on each video tile, and can draw ONNX Runtime detections using `weights/person_yolov5n.onnx`. OpenCV is used only for image display and drawing, not ONNX inference. Press `q`, `Esc`, or `Ctrl+C` to release every RealSense pipeline.

```bash
sudo apt install python3-numpy python3-opencv
python3 -m pip install --user onnxruntime
python3 scripts/realsense_visualize.py
python3 scripts/realsense_visualize.py --list-only
python3 scripts/realsense_visualize.py --detect-fps 30 --target-class-id 0
python3 scripts/realsense_visualize.py --no-detect
```

The default visualizer profile is `640,480,30`; lower it with `--color-profile 424,240,15` on constrained USB buses. The detector overlay defaults to the COCO person class (`target_class_id=0`). If Python ONNX Runtime is not installed, use `--no-detect` for USB/image-only mode.

ROS 2 parameters are in [config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml). The launch file is XML-only; application logic resides in [src/yolo_weldline_3d_node.cpp](src/yolo_weldline_3d_node.cpp).

## Debian packages: AMD and ARM

Run the package build on the target architecture:

```bash
./scripts/build_deb.sh
```

The script reads both the ROS distribution and Debian architecture from the current machine, accepts `amd64`, `arm64`, or `armhf`, builds with `colcon`, stages the resulting ROS package under `/opt/ros/${ROS_DISTRO}`, and creates the `.deb` with `dpkg-deb`. The filename explicitly identifies its compatibility target, for example `weldline_detector_1.0.0_ros-humble_amd64.deb` or `weldline_detector_1.0.0_ros-jazzy_arm64.deb`; a paired `.build-info` file records the same values.

This native package path does not require `bloom`. The package declares only the direct ROS runtime dependencies, the official `ros-${ROS_DISTRO}-realsense2-camera` driver dependency, `ros-${ROS_DISTRO}-ros2launch`, and the detected OpenCV runtime libraries. Required build tooling is the selected ROS distribution, `colcon`, a C++ compiler, and ordinary package build dependencies already needed by `./build.sh`.

## Deployment notes

- Use a valid ONNX model supported by ONNX Runtime on the target. The supplied model is verified during build; a corrupt or unsupported ONNX file is rejected at build/startup with a fatal diagnostic.
- Keep the model and ROS distribution identical across AMD/ARM release builds for reproducible detector behavior.
- Connect the sensor optical frame to `map` (or configure `output_frame` to a valid TF frame) before enabling Nav2 navigation.
# cocelo-hd-detection
