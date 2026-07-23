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

`launch.sh` starts `realsense2_camera/rs_launch.py` with color, depth, and aligned depth enabled. The default `camera_name:=camera` produces the input topics configured in the YAML file. Select a physical camera explicitly when several are connected:

```bash
./launch.sh camera_serial_no:=123456789012
```

For an externally managed RealSense driver, keep the detector only:

```bash
./launch.sh start_realsense:=false
```

## RealSense diagnostics and visualization

The explicit Python visualizer is separate from the production C++ detector. Run it before `./launch.sh` when you need to decide which physical RealSense should be assigned to `usb_port_id` in `yolo_weldline_3d.yaml`.

It uses `pyrealsense2` to discover every connected camera, starts one RGB-only `realsense2_camera` driver per camera, logs each serial number and USB topology, and overlays the same identifiers on each video tile. The first camera starts immediately, and additional cameras are staggered slightly to avoid transient USB ownership conflicts on Jetson. The visualizer uses a lightweight `640,480,30` RGB profile by default. The image subscribers use ROS sensor-data QoS, matching the RealSense image publishers. Pressing `q`, `Esc`, or `Ctrl+C` stops the temporary drivers it started and gives ROS launch time to release the USB interfaces.

```bash
source /opt/ros/<distro>/setup.bash
ROS_DOMAIN_ID=20 python3 scripts/realsense_visualize.py
```

Use `--no-start-drivers` for read-only observation of already-running camera topics. By default, the script also cleans up stale `visualizer_*` RealSense launch processes left by a previous crash; pass `--keep-stale-drivers` only when you intentionally want to preserve those temporary drivers. If the USB bus is slow to release devices, increase the launch gap with `--driver-start-interval-sec 5` or lower the visualizer stream load with `--color-profile 424,240,15`.

If RealSense logs `Failed to load plugin image_transport/raw_pub` or `No plugins found`, install the selected ROS distribution's image transport package before retrying:

```bash
sudo apt install ros-$ROS_DISTRO-image-transport
```

ROS 2 parameters are in [config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml). The launch file is XML-only; application logic resides in [src/yolo_weldline_3d_node.cpp](src/yolo_weldline_3d_node.cpp).

## Debian packages: AMD and ARM

Run the package build on the target architecture:

```bash
./scripts/build_deb.sh
```

The script reads both the ROS distribution and Debian architecture from the current machine, accepts `amd64`, `arm64`, or `armhf`, generates ROS Debian metadata using `bloom`, and places the native artifact under `dist/`. The filename explicitly identifies its compatibility target, for example `weldline_detector_1.0.0_ros-humble_amd64.deb` or `weldline_detector_1.0.0_ros-jazzy_arm64.deb`; a paired `.build-info` file records the same values. The generated package declares `ros-${ROS_DISTRO}-realsense2-camera` as a Debian dependency, so installing the weldline `.deb` also installs the official RealSense driver and its `librealsense` dependencies from the configured ROS apt repository. It intentionally refuses to overwrite an existing `debian/` directory; review or remove generated metadata before a new generation.

Required build tooling: `python3-bloom`, `dpkg-dev`, the selected ROS distribution, and the package dependencies resolved with `rosdep`.

## Deployment notes

- Use a valid ONNX model supported by the OpenCV version installed on the target. The supplied model must be verified during CI; a corrupt or unsupported ONNX file is rejected at startup with a fatal diagnostic.
- Keep the model and ROS distribution identical across AMD/ARM release builds for reproducible detector behavior.
- Connect the sensor optical frame to `map` (or configure `output_frame` to a valid TF frame) before enabling Nav2 navigation.
# cocelo-hd-detection
