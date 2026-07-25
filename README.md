# Scenario Commander + Weldline 3D Localization (ROS 2 / C++)

This package now contains the scenario commander's wall-alignment foundation in
addition to the RGB-D detector. `scenario_commander_node` consumes the refined
POINT_LIO global map, estimates nearby walls in the robot-link frame, and
continuously publishes the signed link-to-wall angular error required for precise
parallel alignment.

The detector remains a native C++17 ROS 2 node using ONNX Runtime for ONNX inference
and OpenCV only for image processing. The integrated bringup launches RealSense,
the detector, and the commander.

## Scenario waypoint drive contract

The strict route is defined in [config/scenario.yaml](config/scenario.yaml).
Its first waypoint has `source: detector`, references `weights/best.onnx`, and is
locked only after five fresh `/goal_pose` samples agree within the configured
position/yaw spread. Every later waypoint is a predefined map pose. The shipped
fixed coordinates are placeholders and must be replaced with surveyed poses
before a real deployment.

The controller reads the latest `map -> base_link` pose from TF. With
autonomy-light this is the composed `map -> odom -> base_link` chain. Map-frame
position error is converted into `base_link` axes for a holonomic `[vx, vy]`
command. This is intentionally a direct waypoint controller, not a collision
avoiding global/local planner.

| Interface | Type / QoS | Meaning |
| --- | --- | --- |
| Detector waypoint | `geometry_msgs/PoseStamped` on `/goal_pose` | Dynamic first waypoint |
| Robot pose | TF `map -> base_link` | Current planar position and yaw |
| Wall input | Internal wall estimate | Continuous physical alignment error |
| RL command | `std_msgs/String`, depth 10, reliable/volatile on `/fsm_cmd` | `cmd vx vy wz` (`m/s`, `m/s`, `rad/s`) |
| Scenario status | `std_msgs/String` on `/commander/scenario/status` | State, waypoint, errors, command, and stop reason |

Safety and transition rules are fail-closed:

- missing/stale/future TF, detector pose, or wall estimate always emits
  `cmd 0.000000 0.000000 0.000000`;
- wall error above `wall_motion_gate_deg` disables translation and commands only
  proportional `wz`;
- while translating, wall `wz` correction remains active continuously;
- a waypoint advances only after position, scenario yaw, and wall yaw all remain
  within their individual tolerances for `hold_time_sec`;
- an inconsistent scenario yaw and physical wall heading enters
  `HEADING_CONFLICT` and stops;
- the last waypoint stays in closed-loop `HOLDING_FINAL` forever. Position or
  alignment drift immediately reactivates correction.

`yaw_mode: parallel` enforces yaw modulo 180 degrees, which matches an
undirected wall tangent. Use `directional` only when yaw and yaw+180 degrees must
be distinguished. Override the scenario at launch with
`WELDLINE_SCENARIO=/path/to/scenario.yaml ./launch.sh`.

Inspect the live controller:

```bash
ros2 topic echo /fsm_cmd
ros2 topic echo /commander/scenario/status
```

## Commander wall-alignment contract

The commander subscribes directly to `/point_lio/global_map_refined` as
`sensor_msgs/PointCloud2`. Its subscription is `RELIABLE + TRANSIENT_LOCAL`, matching
POINT_LIO's refined-map publisher so a commander started after map creation still
receives the cached map. The topic and all fitting parameters are editable under
`scenario_commander_node.ros__parameters` in
[config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml).

| Interface | Type | Default | Meaning |
| --- | --- | --- | --- |
| Refined map input | `sensor_msgs/PointCloud2` | `/point_lio/global_map_refined` | Cached global XYZ map |
| Angle output | `std_msgs/Float64` | `/commander/wall_alignment/angle_deg` | Signed link-to-wall tangent error in degrees |
| Valid output | `std_msgs/Bool` | `/commander/wall_alignment/valid` | Whether the current estimate passed all checks |
| Metrics output | `geometry_msgs/Vector3Stamped` | `/commander/wall_alignment/metrics` | `x=angle_deg`, `y=distance_m`, `z=fit_rmse_m` |
| Debug markers | `visualization_msgs/MarkerArray` | `/commander/wall_alignment/markers` | Selected wall, nearest point, and fit values |

The angle uses the configured `reference_link` (default `base_link`). It is the
signed shortest rotation from link +X to the selected wall tangent, normalized to
`[-90°, 90°)`: parallel is exactly `0°`, counter-clockwise correction is positive,
and clockwise correction is negative. If no valid wall is available, `valid=false`
and `angle_deg=NaN` are published.

Nearby map points are cropped using the latest map-to-link TF, grouped into XY
voxels, and required to show configurable vertical extent. This suppresses floor
and ceiling returns. Multiple wall hypotheses are extracted with RANSAC and
refined over all inliers using total least squares (PCA); length, inlier count, and
fit RMSE gates reject clutter. `wall_sector` can restrict selection to `any`,
`front`, `rear`, `left`, or `right`. The default `tracked` selection initially
ranks candidates by inlier count times observed length, then maintains angular
continuity with that wall. `wall_tracking_max_angle_deg` limits association jumps.
Use `strongest` for stateless selection or `nearest` only when proximity is the
intended policy.

Run only the commander when the camera detector is not needed:

```bash
source install/setup.bash
ros2 launch weldline_reflectivity_detector scenario_commander.launch.xml
ros2 topic echo /commander/wall_alignment/angle_deg
```

The commander-only launch still waits for the detector waypoint and publishes
zero velocity until a fresh, stable `/goal_pose` is received.

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
./launch.sh  # RealSense + detector + scenario commander
./launch.sh --vis
```

Like `cocelo-hd-autonomy-light`, this repository is managed as a standalone
package. `build.sh` writes all generated data under the repository root:

```text
cocelo-hd-detection/
├── build/
├── install/
├── log/
├── third_party/
├── config/
├── include/
└── src/
```

`COCELO_BUILD_BASE`, `COCELO_INSTALL_BASE`, and `COCELO_LOG_BASE` are available
for isolated CI builds. ONNX Runtime is cached under `third_party/`.

Before the first launch, set a unique DDS domain and the physical RealSense USB topology in [config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml). `usb_port_id` is deliberately required when the integrated driver starts, preventing an arbitrary camera from being selected on multi-camera systems.

```yaml
ros_domain_id: 20
usb_port_id: "2-1.3"
```

Inspect USB topology with the visualizer below, then use the `usb_port_id` overlaid on the camera image you want the detector to own. Environment variables override the deployment file: `ROS_DOMAIN_ID=20 USB_PORT_ID=2-1.3 ./launch.sh`.

The scripts use the sourced `ROS_DISTRO`; if none is sourced, they automatically select the sole distribution under `/opt/ros`. When several distributions are installed, source the intended one (or set `ROS_DISTRO`) to avoid an ambiguous build. Common overrides:

```bash
WEIGHTS=/opt/models/weldline.onnx ./launch.sh output_frame:=base_link processing_rate_hz:=30.0
```

An example COCO person detector is included as `weights/person_yolov5n.onnx`. It uses the same `1x3x640x640` RGB input layout; run it with class filtering enabled:

```bash
WEIGHTS=$PWD/weights/person_yolov5n.onnx ./launch.sh target_class_id:=0
```

`./build.sh` prepares an architecture-matched ONNX Runtime C++ SDK when the system does not already provide one, then validates the configured ONNX model with ONNX Runtime on the target machine. This avoids OpenCV DNN parser failures such as unsupported `TopK` nodes on Jetson while still failing the build if the model itself cannot be loaded. Override the model used for validation with `WELDLINE_ONNX_MODEL=/opt/models/weldline.onnx ./build.sh`.

`launch.sh` starts `realsense2_camera/rs_launch.py`, `yolo_weldline_3d_node`,
and `scenario_commander_node` together. The default `camera_name:=camera`
produces the input topics configured in the YAML file. Select a physical camera
explicitly when several are connected:

```bash
./launch.sh camera_serial_no:=123456789012
```

When RealSense is managed externally, skip only the integrated camera driver;
the detector and commander still start:

```bash
./launch.sh start_realsense:=false
```

The commander is enabled by default. Disable only it when troubleshooting the
detector:

```bash
./launch.sh start_commander:=false
```

Use `--vis` to show `/weldline_yolo/debug/annotated_image` in an OpenCV window while the detector is running. The viewer also subscribes to `/weldline_yolo/debug/center_point` and overlays the latest 3D coordinate at the top of the image. Override viewer topics with `WELDLINE_VIS_IMAGE_TOPIC` and `WELDLINE_VIS_POINT_TOPIC` when needed.

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
./scripts/package_deb.sh
```

The script reads both the ROS distribution and Debian architecture from the current machine, accepts `amd64` or `arm64`, builds with `colcon`, stages the resulting ROS package under `/opt/cocelo/weldline-detector/install`, and creates the `.deb` with `dpkg-deb`. The filename explicitly identifies its compatibility target, for example `cocelo-weldline-detector_1.0.0-1+humble22.04_amd64.deb` or `cocelo-weldline-detector_1.0.0-1+jazzy24.04_arm64.deb`; a paired `.build-info` file records the same values.

This native package path does not require `bloom`. The package assumes ROS 2 is already installed at `/opt/ros/${ROS_DISTRO}`, bundles the detector-specific runtime tree and ONNX Runtime shared library, and declares the official `ros-${ROS_DISTRO}-realsense2-camera` driver dependency. After installation:

```bash
sudo apt install ./dist/cocelo-weldline-detector_<version>_<arch>.deb
weldline-detector-visualize --no-detect
sudoedit /etc/cocelo/weldline-detector/yolo_weldline_3d.yaml
sudoedit /etc/cocelo/weldline-detector/scenario.yaml
weldline-detector --vis
weldline-detector-doctor
```

## Deployment notes

- Use a valid ONNX model supported by ONNX Runtime on the target. The supplied model is verified during build; a corrupt or unsupported ONNX file is rejected at build/startup with a fatal diagnostic.
- Keep the model and ROS distribution identical across AMD/ARM release builds for reproducible detector behavior.
- Connect the sensor optical frame to `map` (or configure `output_frame` to a valid TF frame) before enabling Nav2 navigation.
# cocelo-hd-detection
