# Weldline 3D Goal Publisher

ROS 2 Python package for one job: start an Intel RealSense RGB-D camera, run the
weld-line YOLO model on color frames, project the 2D detection into 3D with the
aligned depth image, transform it with TF2, and publish a Nav2-compatible
`geometry_msgs/PoseStamped` goal.

Downstream navigation code should subscribe to `/weldline/goal_pose` and convert
that single weld-line goal into whatever waypoint format it needs.

## Runtime Interface

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| input | `/camera/camera/color/image_raw` | `sensor_msgs/Image` | YOLO 2D weld-line detection |
| input | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | depth for 2D-to-3D projection |
| input | `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | camera intrinsics |
| output | `/weldline/goal_pose` | `geometry_msgs/PoseStamped` | Nav2-style weld-line goal |

The output pose is transformed into `output_frame` (`map` by default). Set
`nav2_planar_goal: true` to publish `z = 0.0`, which is the default.

## Build

```bash
source /opt/ros/<ros_distro>/setup.bash
./build.sh
```

`build.sh` writes colcon artifacts outside the repository by default:

```text
~/.cache/cocelo/weldline-goal-publisher/
```

## Run

By default, `launch.sh` auto-selects the only connected Intel RealSense device.
If multiple cameras are connected, set the intended USB topology in
[config/yolo_weldline_3d.yaml](config/yolo_weldline_3d.yaml) or override it with
`USB_PORT_ID`.

```bash
./launch.sh
USB_PORT_ID=2-1.3 ./launch.sh
```

Common overrides:

```bash
WEIGHTS=$PWD/weights/best.onnx ./launch.sh output_frame:=map goal_topic:=/weldline/goal_pose
./launch.sh start_realsense:=false
```

`start_realsense:=false` is useful when another bringup already owns the camera.

Enable annotated visualization with:

```bash
weldline-detector --vis
# or from the source tree:
./launch.sh --vis
```

When enabled, annotated color frames are published on `/weldline/debug_image`.
An OpenCV window named `weldline-detector` also opens automatically. The debug
image shows color and depth side-by-side. Detected frames include the weld-line
bounding box, line, center point, confidence, and transformed 3D center
coordinates on both panels. Frames without a detection are published without an
overlay. You can also view the topic with:

```bash
ros2 run rqt_image_view rqt_image_view /weldline/debug_image
```

If the RealSense driver reports a depth stream start failure, retry with lower
USB bandwidth:

```bash
weldline-detector --vis --low-bandwidth
```

## Verify

```bash
source ~/.cache/cocelo/weldline-goal-publisher/install/setup.bash
ros2 topic echo /weldline/goal_pose
```

Python runtime dependencies:

```bash
python3 -m pip install --user -r requirements.txt
```
