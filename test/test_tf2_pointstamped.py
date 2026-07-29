import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from tf2_ros import Buffer


def test_pointstamped_transforms_with_buffer_after_importing_node_module():
    import weldline_goal_publisher.yolo_node as yolo_node  # noqa: F401

    rclpy.init()
    try:
        buffer = Buffer()
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = 1.0
        point.point.y = 2.0
        point.point.z = 3.0

        transformed = buffer.transform(point, "map", timeout=Duration(seconds=0.0))

        assert transformed.header.frame_id == "map"
        assert transformed.point.x == 1.0
        assert transformed.point.y == 2.0
        assert transformed.point.z == 3.0
    finally:
        rclpy.shutdown()
