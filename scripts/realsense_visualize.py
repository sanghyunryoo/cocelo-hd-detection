#!/usr/bin/env python3
"""Start RGB drivers for every connected RealSense camera and visualize USB IDs."""
import argparse
import atexit
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
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
    def __init__(self, start_drivers: bool, shutdown_timeout_sec: float, driver_start_interval_sec: float) -> None:
        super().__init__("realsense_visualizer")
        self._bridge = CvBridge()
        self._start_drivers = start_drivers
        self._shutdown_timeout_sec = shutdown_timeout_sec
        self._driver_start_interval_sec = driver_start_interval_sec
        self._closing = False
        self._lock = threading.Lock()
        self._frames: Dict[str, Optional[np.ndarray]] = {}
        self._subscriptions = {}
        self._managed_drivers: Dict[str, subprocess.Popen] = {}
        self._pending_drivers: List[RealSenseDevice] = []
        self._labels: Dict[str, str] = {}
        self._last_device_report = ""
        self._last_topic_report = ""
        self.create_timer(1.0, self._discover_streams)
        self.create_timer(self._driver_start_interval_sec, self._start_next_pending_driver)
        self.create_timer(2.0, self._check_managed_drivers)
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
        if self._closing:
            return
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
        if not self._start_drivers:
            return
        for device in devices:
            key = device.usb_port_id or device.serial
            if key in self._managed_drivers or any((item.usb_port_id or item.serial) == key for item in self._pending_drivers):
                continue
            self._pending_drivers.append(device)
            prefix = f"/{self._camera_name(device)}/{self._camera_name(device)}"
            self._labels[prefix] = f"serial={device.serial} | usb_port_id={device.usb_port_id or 'n/a'}"
            with self._lock:
                self._frames[prefix] = None

    def _check_managed_drivers(self) -> None:
        if self._closing:
            return
        for key in list(self._managed_drivers):
            process = self._managed_drivers.get(key)
            if process is not None and process.poll() is not None:
                self._managed_drivers.pop(key, None)
                self.get_logger().warn(f"Managed RealSense driver exited unexpectedly: {key}")

    def _start_next_pending_driver(self) -> None:
        if self._closing or not self._start_drivers or not self._pending_drivers:
            return
        self._start_driver(self._pending_drivers.pop(0))

    def _start_driver(self, device: RealSenseDevice) -> None:
        if self._closing:
            return
        if shutil.which("ros2") is None:
            self.get_logger().error("ros2 command is unavailable; source the target ROS 2 distribution first.")
            return
        camera_name = self._camera_name(device)
        command = [
            "ros2", "launch", "realsense2_camera", "rs_launch.py",
            f"camera_name:={camera_name}", f"camera_namespace:={camera_name}", f"serial_no:=_{device.serial}",
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

    @staticmethod
    def _wait_process_group(process: subprocess.Popen, timeout_sec: float) -> bool:
        try:
            process.wait(timeout=max(0.1, timeout_sec))
            return True
        except subprocess.TimeoutExpired:
            return False

    @staticmethod
    def _signal_process_group(process: subprocess.Popen, signum: int) -> None:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass
        except OSError:
            try:
                process.send_signal(signum)
            except OSError:
                pass

    def _stop_driver(self, key: str, reason: str) -> None:
        process = self._managed_drivers.pop(key, None)
        if process is None or process.poll() is not None:
            return
        self.get_logger().info(f"Stopping managed RealSense driver ({reason}): {key}")
        self._signal_process_group(process, signal.SIGINT)
        if self._wait_process_group(process, self._shutdown_timeout_sec):
            return
        self._signal_process_group(process, signal.SIGTERM)
        if self._wait_process_group(process, 3.0):
            return
        self._signal_process_group(process, signal.SIGKILL)
        self._wait_process_group(process, 1.0)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        for key in list(self._managed_drivers):
            self._stop_driver(key, "visualizer shutdown")
        time.sleep(0.5)

    def _discover_streams(self) -> None:
        if self._closing:
            return
        topics = {name for name, _ in self.get_topic_names_and_types()}
        for color_topic in sorted(name for name in topics if name.endswith("/color/image_raw")):
            prefix = color_topic[: -len("/color/image_raw")]
            if prefix in self._subscriptions:
                continue
            with self._lock:
                self._frames[prefix] = None
            self._subscriptions[prefix] = self.create_subscription(
                Image, color_topic, lambda message, key=prefix: self._on_color(key, message), qos_profile_sensor_data
            )
            self.get_logger().info(f"Visualizing RGB topic: {color_topic}; {self._labels.get(prefix, 'externally managed')} ")
        report = ", ".join(sorted(name for name in topics if name.endswith("/color/image_raw")))
        if not report:
            report = "no /color/image_raw topics discovered yet"
        if report != self._last_topic_report:
            self.get_logger().info("Active RealSense RGB topics: " + report)
            self._last_topic_report = report

    def _on_color(self, key: str, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self._lock:
                self._frames[key] = frame
        except Exception as error:
            self.get_logger().warn(f"{key} RGB conversion failed: {error}")

    def _render(self) -> None:
        if self._closing:
            return
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
        try:
            cv2.imshow("All RealSense RGB streams (q or Esc to quit)", canvas)
        except cv2.error as error:
            self.get_logger().error(f"OpenCV window unavailable; closing visualizer cleanly: {error}")
            rclpy.shutdown()
            return
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()


def cleanup_stale_visualizer_drivers() -> None:
    """Terminate RealSense launch processes left by a prior visualizer crash."""
    pgrep = shutil.which("pgrep")
    if pgrep is None:
        return
    result = subprocess.run(
        [pgrep, "-af", "camera_name:=visualizer_"],
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode not in (0, 1):
        return
    current_pid = os.getpid()
    stale_pids = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=1)
        if not fields:
            continue
        try:
            pid = int(fields[0])
        except ValueError:
            continue
        if pid != current_pid:
            stale_pids.append(pid)
    for pid in stale_pids:
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        except OSError:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
    deadline = time.monotonic() + 5.0
    for pid in stale_pids:
        while time.monotonic() < deadline:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            try:
                os.killpg(pid, signal.SIGTERM)
            except OSError:
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass


def install_signal_handlers(node_holder: Sequence[Optional[AllRealSenseVisualizer]]) -> None:
    def _handle_signal(signum: int, _frame) -> None:
        node = node_holder[0]
        if node is not None:
            node.get_logger().info(f"Received signal {signum}; releasing RealSense resources.")
            node.close()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def main() -> None:
    config = default_config_path()
    config_domain = yaml_scalar(config, "ros_domain_id", "0")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", config_domain)))
    parser.add_argument("--no-start-drivers", action="store_true", help="Only visualize already-running ROS camera topics.")
    parser.add_argument("--keep-stale-drivers", action="store_true", help="Do not clean up old visualizer_* RealSense drivers before starting.")
    parser.add_argument("--shutdown-timeout-sec", type=float, default=8.0, help="Seconds to wait for each RealSense launch process to exit gracefully.")
    parser.add_argument("--driver-start-interval-sec", type=float, default=3.0, help="Seconds between starting each RealSense driver.")
    arguments = parser.parse_args()
    if not 0 <= arguments.domain_id <= 232:
        parser.error("--domain-id must be in 0..232")
    if arguments.shutdown_timeout_sec < 1.0:
        parser.error("--shutdown-timeout-sec must be >= 1.0")
    if arguments.driver_start_interval_sec < 0.5:
        parser.error("--driver-start-interval-sec must be >= 0.5")
    os.environ["ROS_DOMAIN_ID"] = str(arguments.domain_id)
    if not arguments.keep_stale_drivers:
        cleanup_stale_visualizer_drivers()
    rclpy.init()
    node_holder: List[Optional[AllRealSenseVisualizer]] = [None]
    node = AllRealSenseVisualizer(
        not arguments.no_start_drivers,
        arguments.shutdown_timeout_sec,
        arguments.driver_start_interval_sec,
    )
    node_holder[0] = node
    install_signal_handlers(node_holder)
    atexit.register(node.close)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        atexit.unregister(node.close)
        node_holder[0] = None
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
