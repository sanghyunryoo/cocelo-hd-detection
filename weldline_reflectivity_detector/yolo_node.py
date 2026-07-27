"""Python RGB-D YOLO weld-line localizer."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, isfinite
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  # Registers PointStamped / PoseStamped transform support for tf2.
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Detection:
    box: tuple[int, int, int, int]
    center: tuple[int, int]
    line_start: tuple[int, int]
    line_end: tuple[int, int]
    confidence: float


class YoloWeldline3DNode(Node):
    def __init__(self):
        super().__init__("yolo_weldline_3d_node")
        self.weights = self.declare_parameter("weights", "").value
        self.output_frame = self.declare_parameter("output_frame", "map").value
        self.rate = float(self.declare_parameter("processing_rate_hz", 30.0).value)
        self.target_class = int(self.declare_parameter("target_class_id", -1).value)
        self.confidence = float(self.declare_parameter("confidence_threshold", 0.40).value)
        self.nms = float(self.declare_parameter("nms_threshold", 0.25).value)
        self.depth_window = int(self.declare_parameter("depth_window", 7).value)
        self.use_lines = bool(self.declare_parameter("use_line_detection", True).value)
        self.planar_goal = bool(self.declare_parameter("nav2_planar_goal", False).value)
        self.bridge, self.latest = CvBridge(), None
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.session = self.load_session(self.weights)
        self.goal_pub = self.create_publisher(PoseStamped, self.declare_parameter("nav2_goal_topic", "/goal_pose").value, 10)
        self.point_pub = self.create_publisher(PointStamped, self.declare_parameter("debug_point_topic", "/weldline_yolo/debug/center_point").value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.declare_parameter("marker_topic", "/weldline_yolo/debug/markers").value, 5)
        self.image_pub = self.create_publisher(Image, self.declare_parameter("annotated_topic", "/weldline_yolo/debug/annotated_image").value, 5)
        color = message_filters.Subscriber(self, Image, self.declare_parameter("color_topic", "/camera/color/image_raw").value)
        depth = message_filters.Subscriber(self, Image, self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw").value)
        info = message_filters.Subscriber(self, CameraInfo, self.declare_parameter("camera_info_topic", "/camera/color/camera_info").value)
        sync = message_filters.ApproximateTimeSynchronizer([color, depth, info], 10, 0.05); sync.registerCallback(self.on_frame); self.sync = sync
        self.create_timer(1.0 / self.rate, self.process_frame)

    def load_session(self, weights: str):
        import onnxruntime
        if not weights or not Path(weights).is_file(): raise ValueError(f"ONNX model does not exist: {weights}")
        return onnxruntime.InferenceSession(weights, providers=["CPUExecutionProvider"])

    def on_frame(self, color: Image, depth: Image, info: CameraInfo) -> None: self.latest = (color, depth, info)

    def process_frame(self) -> None:
        if self.latest is None: return
        color, depth, info = self.latest; self.latest = None
        try:
            image = self.bridge.imgmsg_to_cv2(color, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough").astype(np.float32)
            if depth.encoding in ("16UC1", "mono16"): depth_image *= 0.001
            elif depth.encoding != "32FC1": return self.publish_status(color, image, "UNSUPPORTED DEPTH")
            detection = self.infer(image)
            if detection is None: return self.publish_status(color, image, "NO DETECTION")
            point = self.project(detection.center, depth_image, info)
            if point is None: return self.publish_status(color, self.annotate(image, detection), "DETECTION / NO DEPTH")
            center = self.transform(point, depth.header)
            if center is None: return self.publish_status(color, self.annotate(image, detection), "DETECTION / TF UNAVAILABLE")
            start = self.project(detection.line_start, depth_image, info); end = self.project(detection.line_end, depth_image, info)
            start, end = (self.transform(start, depth.header) if start else None), (self.transform(end, depth.header) if end else None)
            goal = PoseStamped(); goal.header = center.header; goal.pose.position = center.point; goal.pose.position.z = 0.0 if self.planar_goal else center.point.z
            yaw = atan2(end.point.y - start.point.y, end.point.x - start.point.x) if start and end else 0.0; goal.pose.orientation.z, goal.pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            self.goal_pub.publish(goal); self.point_pub.publish(center); self.publish_markers(center, start, end)
            image = self.annotate(image, detection); cv2.putText(image, f"{center.point.x:.3f}, {center.point.y:.3f}, {center.point.z:.3f} m", detection.center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except Exception as error: self.get_logger().error(f"Frame processing failure: {error}")

    def infer(self, image: np.ndarray) -> Detection | None:
        height, width = image.shape[:2]; scale = min(640 / width, 640 / height); resized = cv2.resize(image, (round(width * scale), round(height * scale)))
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8); px, py = (640 - resized.shape[1]) // 2, (640 - resized.shape[0]) // 2; canvas[py:py + resized.shape[0], px:px + resized.shape[1]] = resized
        input_name = self.session.get_inputs()[0].name; output = self.session.run(None, {input_name: canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0})[0]; rows = np.squeeze(output); rows = rows.T if rows.ndim == 2 and rows.shape[0] < rows.shape[1] else rows
        boxes, scores = [], []
        for row in rows:
            if len(row) < 5: continue
            score, class_id = float(row[4]), -1
            if len(row) > 6:
                classes, objectness = row[5:] if len(row) != 84 else row[4:], float(row[4]) if len(row) != 84 else 1.0
                class_id = self.target_class if self.target_class >= 0 else int(np.argmax(classes)); score = objectness * float(classes[class_id])
            if score < self.confidence or (self.target_class >= 0 and class_id >= 0 and class_id != self.target_class): continue
            x1, y1, x2, y2 = map(float, row[:4])
            if x2 <= x1 or y2 <= y1: x1, y1, x2, y2 = x1 - x2 / 2, y1 - y2 / 2, x1 + x2 / 2, y1 + y2 / 2
            x1, y1, x2, y2 = (x1 - px) / scale, (y1 - py) / scale, (x2 - px) / scale, (y2 - py) / scale
            boxes.append([max(0, round(x1)), max(0, round(y1)), max(1, round(x2 - x1)), max(1, round(y2 - y1))]); scores.append(score)
        kept = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.nms)
        if len(kept) == 0: return None
        index = max(np.asarray(kept).reshape(-1), key=lambda item: scores[item]); x, y, w, h = boxes[index]; center, start, end = (x + w // 2, y + h // 2), (x, y + h // 2), (x + w, y + h // 2)
        if self.use_lines:
            lines = cv2.HoughLinesP(cv2.Canny(cv2.cvtColor(image[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY), 40, 120), 1, np.pi / 180, 25, minLineLength=max(10, w // 4), maxLineGap=12)
            if lines is not None:
                line = max(lines[:, 0], key=lambda item: (item[2] - item[0]) ** 2 + (item[3] - item[1]) ** 2); start, end = (x + int(line[0]), y + int(line[1])), (x + int(line[2]), y + int(line[3])); center = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
        return Detection((x, y, w, h), center, start, end, scores[index])

    def project(self, pixel, depth, info):
        x, y = pixel; radius = self.depth_window // 2
        if x < 0 or y < 0 or x >= depth.shape[1] or y >= depth.shape[0] or not info.k[0] or not info.k[4]: return None
        samples = depth[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1]; samples = samples[np.isfinite(samples) & (samples > 0.05)]
        if not len(samples): return None
        z = float(np.median(samples)); return Point(x=(x - info.k[2]) * z / info.k[0], y=(y - info.k[5]) * z / info.k[4], z=z)

    def transform(self, point, header):
        source = PointStamped(); source.header, source.point = header, point
        try: return source if not self.output_frame or self.output_frame == header.frame_id else self.buffer.transform(source, self.output_frame, timeout=Duration(seconds=0.02))
        except TransformException: return None

    def annotate(self, image, detection):
        x, y, w, h = detection.box; image = image.copy(); cv2.rectangle(image, (x, y), (x + w, y + h), (60, 220, 255), 2); cv2.line(image, detection.line_start, detection.line_end, (0, 255, 40), 2); cv2.circle(image, detection.center, 5, (0, 220, 255), -1); return image

    def publish_status(self, source, image, status):
        cv2.rectangle(image, (0, 0), (image.shape[1], 40), (0, 0, 0), -1); cv2.putText(image, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 200, 255), 2); result = self.bridge.cv2_to_imgmsg(image, "bgr8"); result.header = source.header; self.image_pub.publish(result); self.publish_empty(source.header)

    def publish_empty(self, header):
        point = PointStamped(); point.header = header; point.point.x = point.point.y = point.point.z = float("nan"); self.point_pub.publish(point); marker = Marker(action=Marker.DELETEALL); marker.header = header; self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_markers(self, center, start, end):
        sphere = Marker(); sphere.header = center.header; sphere.ns, sphere.id, sphere.type, sphere.action = "weldline", 0, Marker.SPHERE, Marker.ADD; sphere.pose.position, sphere.pose.orientation.w = center.point, 1.0; sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05; sphere.color.r = sphere.color.g = sphere.color.a = 1.0; markers = [sphere]
        if start and end:
            line = Marker(); line.header = center.header; line.ns, line.id, line.type, line.action = "weldline", 1, Marker.LINE_STRIP, Marker.ADD; line.scale.x, line.color.g, line.color.a, line.points = 0.012, 1.0, 1.0, [start.point, end.point]; markers.append(line)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main() -> None:
    rclpy.init(); node = YoloWeldline3DNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
