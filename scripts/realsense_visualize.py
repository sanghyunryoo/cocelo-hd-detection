#!/usr/bin/env python3
"""Discover every connected RealSense and visualize every active ROS 2 RGB stream."""
import argparse
import math
import os
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class AllRealSenseVisualizer(Node):
    """Dynamically subscribes to every active <prefix>/color/image_raw stream."""

    def __init__(self) -> None:
        super().__init__("realsense_visualizer")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frames = {}
        self._subscriptions = {}
        self._last_usb_report = ""
        self._last_topic_report = ""
        self.create_timer(1.0, self._discover_streams)
        self.create_timer(5.0, self._report_usb_devices)
        self.create_timer(0.05, self._render)
        self._report_usb_devices()
        self._discover_streams()
        self.get_logger().info(
            f"Discovering all RealSense streams on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}."
        )

    def _report_usb_devices(self) -> None:
        command = shutil.which("rs-enumerate-devices")
        if command is None:
            report = "rs-enumerate-devices is unavailable; physical USB enumeration skipped."
        else:
            try:
                result = subprocess.run(
                    [command, "-s"], text=True, capture_output=True, timeout=4, check=False
                )
                report = result.stdout.strip() or result.stderr.strip() or "No RealSense USB device found."
            except (OSError, subprocess.TimeoutExpired) as error:
                report = f"Could not enumerate RealSense USB devices: {error}"
        if report != self._last_usb_report:
            self.get_logger().info("Physical RealSense USB devices:\n" + report)
            self._last_usb_report = report

    def _discover_streams(self) -> None:
        topic_names = {name for name, _ in self.get_topic_names_and_types()}
        color_topics = sorted(name for name in topic_names if name.endswith("/color/image_raw"))
        for color_topic in color_topics:
            prefix = color_topic[: -len("/color/image_raw")]
            if prefix in self._subscriptions:
                continue
            self._frames[prefix] = {"color": None}
            self._subscriptions[prefix] = self.create_subscription(
                Image, color_topic, lambda msg, key=prefix: self._on_color(key, msg), 10
            )
            self.get_logger().info(f"Subscribed to {color_topic}")
        report = ", ".join(color_topics) if color_topics else "no ROS color streams discovered"
        if report != self._last_topic_report:
            self.get_logger().info("Active RealSense color topics: " + report)
            self._last_topic_report = report

    def _on_color(self, key: str, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self._lock:
                self._frames[key]["color"] = frame
        except Exception as error:
            self.get_logger().warn(f"{key} color conversion failed: {error}")

    @staticmethod
    def _label(frame: np.ndarray, label: str) -> np.ndarray:
        output = frame.copy()
        cv2.rectangle(output, (0, 0), (min(output.shape[1], 420), 32), (0, 0, 0), cv2.FILLED)
        cv2.putText(output, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        return output

    def _render(self) -> None:
        tiles = []
        with self._lock:
            snapshots = [(key, values["color"]) for key, values in self._frames.items()]
        for key, color in snapshots:
            if color is not None:
                tiles.append(self._label(color, f"{key} color"))
        if not tiles:
            return
        target_width, target_height = 640, 360
        tiles = [cv2.resize(tile, (target_width, target_height)) for tile in tiles]
        columns = math.ceil(math.sqrt(len(tiles)))
        rows = math.ceil(len(tiles) / columns)
        blank = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        tiles.extend([blank] * (rows * columns - len(tiles)))
        canvas = np.vstack([np.hstack(tiles[row * columns : (row + 1) * columns]) for row in range(rows)])
        cv2.imshow("All RealSense RGB streams (q or Esc to quit)", canvas)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    arguments = parser.parse_args()
    if not 0 <= arguments.domain_id <= 232:
        parser.error("--domain-id must be in 0..232")
    os.environ["ROS_DOMAIN_ID"] = str(arguments.domain_id)
    rclpy.init()
    node = AllRealSenseVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
