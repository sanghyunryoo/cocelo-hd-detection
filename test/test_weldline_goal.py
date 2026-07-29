from math import sqrt

import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo

from weldline_goal_publisher.yolo_node import pose_from_points, project_pixel


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
