from math import sqrt

import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo

from weldline_goal_publisher.yolo_node import Detection2D, pose_from_points, project_pixel, render_debug_image


def camera_info() -> CameraInfo:
    info = CameraInfo()
    info.k = [100.0, 0.0, 10.0, 0.0, 100.0, 20.0, 0.0, 0.0, 1.0]
    return info


def stamped_point(x: float, y: float, z: float = 0.0) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = "map"
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


def test_project_pixel_uses_median_valid_depth_window():
    depth = np.full((5, 5), np.nan, dtype=np.float32)
    depth[1:4, 1:4] = np.array(
        [
            [0.0, 2.0, 100.0],
            [2.0, 2.0, 2.0],
            [2.0, 2.0, 2.0],
        ],
        dtype=np.float32,
    )

    point = project_pixel((2, 2), depth, camera_info(), 3)

    assert point is not None
    assert point.z == 2.0
    assert point.x == -0.16
    assert point.y == -0.36


def test_pose_from_points_publishes_planar_nav2_goal_with_line_yaw():
    pose = pose_from_points(
        stamped_point(1.0, 2.0, 3.0),
        stamped_point(0.0, 0.0),
        stamped_point(0.0, 1.0),
        planar=True,
    )

    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == 2.0
    assert pose.pose.position.z == 0.0
    assert abs(pose.pose.orientation.z - sqrt(0.5)) < 1e-12
    assert abs(pose.pose.orientation.w - sqrt(0.5)) < 1e-12


def test_render_debug_image_returns_plain_frame_without_detection():
    image = np.full((40, 60, 3), 20, dtype=np.uint8)

    rendered = render_debug_image(image, None, None, None)

    assert np.array_equal(rendered[:, : image.shape[1]], image)
    assert rendered.shape == (image.shape[0], image.shape[1] * 2 + 4, 3)
    assert rendered is not image


def test_render_debug_image_draws_detection_overlay_and_coordinates():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    depth = np.linspace(0.5, 2.0, 80 * 120, dtype=np.float32).reshape(80, 120)
    detection = Detection2D(
        box=(10, 20, 50, 30),
        center=(35, 35),
        line_start=(12, 35),
        line_end=(58, 35),
        confidence=0.91,
    )
    center = stamped_point(1.2, -0.3, 2.4)

    rendered = render_debug_image(image, depth, detection, center)

    assert rendered.shape == (image.shape[0], image.shape[1] * 2 + 4, 3)
    assert np.count_nonzero(rendered) > 0
    assert rendered[20, 10].any()
    assert rendered[20, image.shape[1] + 4 + 10].any()
