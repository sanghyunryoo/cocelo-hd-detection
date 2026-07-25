#!/usr/bin/env python3
"""OpenCV viewer for weldline annotated image and latest 3D debug point."""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_bgr(message: Image) -> np.ndarray:
    dtype = np.dtype(np.uint8)
    if message.encoding in ("16UC1", "mono16"):
        dtype = np.dtype(np.uint16)
    data = np.frombuffer(message.data, dtype=dtype)
    if message.encoding in ("bgr8", "rgb8"):
        frame = data.reshape((message.height, message.width, 3))
        if message.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()
    if message.encoding in ("mono8",):
        return cv2.cvtColor(data.reshape((message.height, message.width)), cv2.COLOR_GRAY2BGR)
    if message.encoding in ("16UC1", "mono16"):
        mono = data.reshape((message.height, message.width))
        mono8 = cv2.convertScaleAbs(mono, alpha=255.0 / max(1, int(mono.max())))
        return cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {message.encoding}")


class AnnotatedImageViewer(Node):
    def __init__(self, image_topic: str, point_topic: str, window_name: str, max_fps: float) -> None:
        super().__init__("weldline_annotated_image_viewer")
        self._window_name = window_name
        self._period = 1.0 / max_fps
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._point: Optional[PointStamped] = None
        self._closing = False
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(PointStamped, point_topic, self._on_point, 5)
        self.create_timer(self._period, self._render)
        self.get_logger().info(f"Viewing annotated image: {image_topic}")
        self.get_logger().info(f"Overlaying latest 3D point: {point_topic}")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        cv2.destroyAllWindows()

    def _on_image(self, message: Image) -> None:
        try:
            frame = image_to_bgr(message)
        except Exception as error:
            self.get_logger().warn(f"Cannot convert annotated image: {error}")
            return
        with self._lock:
            self._frame = frame

    def _on_point(self, message: PointStamped) -> None:
        with self._lock:
            self._point = message

    def _render(self) -> None:
        if self._closing:
            return
        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
            point = self._point
        if frame is None:
            frame = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "waiting for annotated image", (20, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (180, 180, 180), 2, cv2.LINE_AA)
        self._draw_point(frame, point)
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()

    @staticmethod
    def _draw_point(frame: np.ndarray, point: Optional[PointStamped]) -> None:
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (0, 0, 0), cv2.FILLED)
        if point is None:
            text = "XYZ: waiting for /weldline_yolo/debug/center_point"
            color = (180, 180, 180)
        elif any(math.isnan(value) for value in (point.point.x, point.point.y, point.point.z)):
            text = "XYZ: no valid 3D point"
            color = (0, 200, 255)
        else:
            text = f"XYZ[{point.header.frame_id}]: {point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f} m"
            color = (0, 255, 255)
        cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.68, color, 2, cv2.LINE_AA)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/weldline_yolo/debug/annotated_image")
    parser.add_argument("--point-topic", default="/weldline_yolo/debug/center_point")
    parser.add_argument("--window-name", default="Weldline detection debug (q/Esc to close)")
    parser.add_argument("--max-fps", type=float, default=30.0)
    args = parser.parse_args()
    if args.max_fps <= 0.0:
        parser.error("--max-fps must be > 0")

    rclpy.init()
    node = AnnotatedImageViewer(args.image_topic, args.point_topic, args.window_name, args.max_fps)

    def handle_signal(_signum, _frame) -> None:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
