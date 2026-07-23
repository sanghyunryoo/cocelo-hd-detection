#!/usr/bin/env python3
"""Start RGB drivers for every connected RealSense camera and visualize USB IDs."""
import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class RealSenseDevice:
    serial: str
    usb_port_id: str
    physical_port: str
    name: str


def default_config_path() -> Optional[Path]:
    script = Path(__file__).resolve()
    candidates = [
        script.parents[1] / "config" / "yolo_weldline_3d.yaml",  # source tree
        script.parents[2] / "share" / "weldline_reflectivity_detector" / "config" / "yolo_weldline_3d.yaml",  # installed package
        Path.cwd() / "config" / "yolo_weldline_3d.yaml",
    ]
    return next((path for path in candidates if path.is_file()), None)


def yaml_scalar(path: Optional[Path], key: str, fallback: str = "") -> str:
    if path is None:
        return fallback
    expression = re.compile(rf"^\s*{re.escape(key)}\s*:\s*(.*?)\s*(?:#.*)?$")
    for line in path.read_text(encoding="utf-8").splitlines():
        match = expression.match(line)
        if match:
            return match.group(1).strip().strip('"\'')
    return fallback


def normalize_usb_port_id(physical_port: str) -> str:
    """Return the stable USB topology token used by realsense2_camera, when available."""
    matches = re.findall(r"(?:^|/)([0-9]+-[0-9]+(?:\.[0-9]+)*(?::[0-9.]+)?)(?:/|$)", physical_port)
    if not matches:
        return physical_port
    return matches[-1].split(":", 1)[0]


def enumerate_devices() -> List[RealSenseDevice]:
    """Use librealsense directly: it exposes the physical USB topology needed by rs_launch."""
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError("pyrealsense2 is required to auto-start cameras: " + str(error)) from error
    devices = []
    for device in rs.context().query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        name = device.get_info(rs.camera_info.name)
        try:
            physical_port = device.get_info(rs.camera_info.physical_port)
        except RuntimeError:
            physical_port = ""
        devices.append(
            RealSenseDevice(
                serial=serial,
                usb_port_id=normalize_usb_port_id(physical_port),
                physical_port=physical_port,
                name=name,
            )
        )
    return sorted(devices, key=lambda item: (item.usb_port_id, item.serial))


class AllRealSenseVisualizer(Node):
    def __init__(self, start_drivers: bool) -> None:
        super().__init__("realsense_visualizer")
        self._bridge = CvBridge()
        self._start_drivers = start_drivers
        self._lock = threading.Lock()
        self._frames: Dict[str, Optional[np.ndarray]] = {}
        self._subscriptions = {}
        self._managed_drivers: Dict[str, subprocess.Popen] = {}
        self._labels: Dict[str, str] = {}
        self._last_device_report = ""
        self.create_timer(1.0, self._discover_streams)
        self.create_timer(3.0, self._refresh_devices)
        self.create_timer(0.05, self._render)
        self._refresh_devices()
        self._discover_streams()
        self.get_logger().info(
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}; "
            f"visualizing every connected RealSense RGB stream; auto-start={start_drivers}"
        )

    @staticmethod
    def _camera_name(device: RealSenseDevice) -> str:
        return "visualizer_" + re.sub(r"[^A-Za-z0-9_]", "_", device.serial)

    def _refresh_devices(self) -> None:
        try:
            devices = enumerate_devices()
        except RuntimeError as error:
            self.get_logger().error(str(error))
            return
        report = "\n".join(
            f"{item.name}: serial={item.serial}, usb_port_id={item.usb_port_id or '<unavailable>'}, "
            f"physical_port={item.physical_port or '<unavailable>'}" for item in devices
        ) or "none"
        if report != self._last_device_report:
            self.get_logger().info("Connected RealSense devices:\n" + report)
            self._last_device_report = report
        current_keys = {item.usb_port_id or item.serial for item in devices}
        for key in list(self._managed_drivers):
            if key not in current_keys:
                self._stop_driver(key, "device disconnected")
        if not self._start_drivers:
            return
        for device in devices:
            key = device.usb_port_id or device.serial
            if key in self._managed_drivers:
                continue
            self._start_driver(device)

    def _start_driver(self, device: RealSenseDevice) -> None:
        if shutil.which("ros2") is None:
            self.get_logger().error("ros2 command is unavailable; source the target ROS 2 distribution first.")
            return
        camera_name = self._camera_name(device)
        command = [
            "ros2", "launch", "realsense2_camera", "rs_launch.py",
            f"camera_name:={camera_name}", f"serial_no:=_{device.serial}",
            "enable_color:=true", "enable_depth:=false", "align_depth.enable:=false",
        ]
        try:
            process = subprocess.Popen(command, start_new_session=True)
            key = device.usb_port_id or device.serial
            self._managed_drivers[key] = process
            prefix = f"/{camera_name}/{camera_name}"
            self._labels[prefix] = f"serial={device.serial} | usb_port_id={device.usb_port_id or 'n/a'}"
            with self._lock:
                self._frames[prefix] = None
            self.get_logger().info("Started visualizer driver: " + " ".join(command))
        except OSError as error:
            self.get_logger().error(f"Could not start RealSense driver for {device.serial}: {error}")

    def _stop_driver(self, key: str, reason: str) -> None:
        process = self._managed_drivers.pop(key, None)
        if process is None or process.poll() is not None:
            return
        self.get_logger().info(f"Stopping managed RealSense driver ({reason}): {key}")
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except OSError:
                pass

    def close(self) -> None:
        for key in list(self._managed_drivers):
            self._stop_driver(key, "visualizer shutdown")

    def _discover_streams(self) -> None:
        topics = {name for name, _ in self.get_topic_names_and_types()}
        for color_topic in sorted(name for name in topics if name.endswith("/color/image_raw")):
            prefix = color_topic[: -len("/color/image_raw")]
            if prefix in self._subscriptions:
                continue
            with self._lock:
                self._frames[prefix] = None
            self._subscriptions[prefix] = self.create_subscription(
                Image, color_topic, lambda message, key=prefix: self._on_color(key, message), 10
            )
            self.get_logger().info(f"Visualizing RGB topic: {color_topic}; {self._labels.get(prefix, 'externally managed')} ")

    def _on_color(self, key: str, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self._lock:
                self._frames[key] = frame
        except Exception as error:
            self.get_logger().warn(f"{key} RGB conversion failed: {error}")

    def _render(self) -> None:
        with self._lock:
            snapshots = [(key, None if frame is None else frame.copy()) for key, frame in self._frames.items()]
        tiles = []
        for prefix, frame in snapshots:
            if frame is None:
                frame = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "waiting for RGB stream", (22, 192), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)
            label = self._labels.get(prefix, "external camera")
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), cv2.FILLED)
            cv2.putText(frame, prefix, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, label, (8, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
            tiles.append(cv2.resize(frame, (640, 360)))
        if not tiles:
            return
        columns = math.ceil(math.sqrt(len(tiles)))
        rows = math.ceil(len(tiles) / columns)
        blank = np.zeros((360, 640, 3), dtype=np.uint8)
        tiles.extend([blank] * (rows * columns - len(tiles)))
        canvas = np.vstack([np.hstack(tiles[index * columns : (index + 1) * columns]) for index in range(rows)])
        cv2.imshow("All RealSense RGB streams (q or Esc to quit)", canvas)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()


def main() -> None:
    config = default_config_path()
    config_domain = yaml_scalar(config, "ros_domain_id", "0")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", config_domain)))
    parser.add_argument("--no-start-drivers", action="store_true", help="Only visualize already-running ROS camera topics.")
    arguments = parser.parse_args()
    if not 0 <= arguments.domain_id <= 232:
        parser.error("--domain-id must be in 0..232")
    os.environ["ROS_DOMAIN_ID"] = str(arguments.domain_id)
    rclpy.init()
    node = AllRealSenseVisualizer(not arguments.no_start_drivers)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
