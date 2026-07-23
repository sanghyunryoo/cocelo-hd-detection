#!/usr/bin/env python3
"""Inspect a RealSense ROS 2 graph and visualize its RGB-D streams with OpenCV."""
import argparse
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RealSenseVisualizer(Node):
    def __init__(self, camera_name: str, driver_node: str, expected_usb_port: str) -> None:
        super().__init__("realsense_visualizer")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._color = None
        self._depth = None
        self._last_graph_log = 0.0
        self._expected_usb_port = expected_usb_port
        prefix = f"/{camera_name}/camera"
        self._color_subscription = self.create_subscription(Image, f"{prefix}/color/image_raw", self._on_color, 10)
        self._depth_subscription = self.create_subscription(Image, f"{prefix}/aligned_depth_to_color/image_raw", self._on_depth, 10)
        self._info_subscription = self.create_subscription(CameraInfo, f"{prefix}/color/camera_info", self._on_info, 10)
        self._parameter_client = self.create_client(GetParameters, f"{driver_node}/get_parameters")
        self._query_driver_configuration()
        self.create_timer(0.1, self._render)
        self.get_logger().info(
            f"Domain={os.environ.get('ROS_DOMAIN_ID', '0')}, camera={camera_name}, "
            f"expected usb_port_id={expected_usb_port or '<not provided>'}"
        )

    def _query_driver_configuration(self) -> None:
        if not self._parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("RealSense parameter service is unavailable; checking image topics only.")
            return
        request = GetParameters.Request()
        request.names = ["usb_port_id", "serial_no", "enable_color", "enable_depth", "align_depth.enable"]
        future = self._parameter_client.call_async(request)
        future.add_done_callback(self._on_parameters)

    def _on_parameters(self, future) -> None:
        try:
            values = future.result().values
            labels = ["usb_port_id", "serial_no", "enable_color", "enable_depth", "align_depth.enable"]
            rendered = []
            for label, value in zip(labels, values):
                rendered.append(f"{label}={value.string_value or value.bool_value}")
            self.get_logger().info("Driver parameters: " + ", ".join(rendered))
        except Exception as error:  # ROS service errors must not stop visualization.
            self.get_logger().warn(f"Could not read RealSense parameters: {error}")

    def _on_color(self, message: Image) -> None:
        try:
            with self._lock:
                self._color = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().warn(f"Color conversion failed: {error}")

    def _on_depth(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            with self._lock:
                self._depth = depth
        except Exception as error:
            self.get_logger().warn(f"Depth conversion failed: {error}")

    def _on_info(self, message: CameraInfo) -> None:
        self.get_logger().info(
            f"CameraInfo: {message.width}x{message.height}, fx={message.k[0]:.2f}, fy={message.k[4]:.2f}"
        )
        if self._info_subscription is not None:
            self.destroy_subscription(self._info_subscription)
            self._info_subscription = None

    def _render(self) -> None:
        now = time.monotonic()
        if now - self._last_graph_log > 5.0:
            topics = sorted(name for name, _ in self.get_topic_names_and_types() if "/camera/" in name)
            self.get_logger().info("RealSense topics: " + (", ".join(topics) if topics else "none discovered"))
            self._last_graph_log = now
        with self._lock:
            color = None if self._color is None else self._color.copy()
            depth = None if self._depth is None else self._depth.copy()
        if color is None or depth is None:
            return
        if depth.dtype == np.uint16:
            depth_meters = depth.astype(np.float32) * 0.001
        else:
            depth_meters = depth.astype(np.float32)
        depth_normalized = cv2.normalize(depth_meters, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
        if depth_color.shape[:2] != color.shape[:2]:
            depth_color = cv2.resize(depth_color, (color.shape[1], color.shape[0]))
        canvas = np.hstack((color, depth_color))
        cv2.putText(canvas, "Color", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(canvas, "Aligned depth", (color.shape[1] + 12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("RealSense RGB-D diagnostics (q to quit)", canvas)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--camera-name", default="camera")
    parser.add_argument("--driver-node", default="/camera/camera")
    parser.add_argument("--usb-port-id", default=os.environ.get("USB_PORT_ID", ""))
    arguments = parser.parse_args()
    if not 0 <= arguments.domain_id <= 232:
        parser.error("--domain-id must be in 0..232")
    os.environ["ROS_DOMAIN_ID"] = str(arguments.domain_id)
    rclpy.init()
    node = RealSenseVisualizer(arguments.camera_name, arguments.driver_node, arguments.usb_port_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
