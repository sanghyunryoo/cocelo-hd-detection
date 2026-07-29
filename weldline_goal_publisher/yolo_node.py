"""Publish a Nav2-style weld-line goal from RealSense RGB-D and YOLO."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  Registers PointStamped transforms.
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


MODEL_SIZE = 640


@dataclass(frozen=True)
class Detection2D:
    box: tuple[int, int, int, int]
    center: tuple[int, int]
    line_start: tuple[int, int]
    line_end: tuple[int, int]
    confidence: float


@dataclass(frozen=True)
class ProjectedDetection:
    pose: PoseStamped
    center: PointStamped


def depth_to_meters(depth_msg: Image, bridge: CvBridge) -> np.ndarray | None:
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)
    if depth_msg.encoding in ("16UC1", "mono16"):
        return depth * 0.001
    if depth_msg.encoding == "32FC1":
        return depth
    return None


def project_pixel(pixel: tuple[int, int], depth: np.ndarray, info: CameraInfo, window: int) -> Point | None:
    x, y = pixel
    if x < 0 or y < 0 or x >= depth.shape[1] or y >= depth.shape[0]:
        return None
    if not info.k[0] or not info.k[4]:
        return None

    radius = max(0, window // 2)
    samples = depth[
        max(0, y - radius) : y + radius + 1,
        max(0, x - radius) : x + radius + 1,
    ]
    samples = samples[np.isfinite(samples) & (samples > 0.05)]
    if samples.size == 0:
        return None

    z = float(np.median(samples))
    return Point(
        x=(x - info.k[2]) * z / info.k[0],
        y=(y - info.k[5]) * z / info.k[4],
        z=z,
    )


def pose_from_points(center: PointStamped, start: PointStamped | None, end: PointStamped | None, planar: bool) -> PoseStamped:
    goal = PoseStamped()
    goal.header = center.header
    goal.pose.position.x = center.point.x
    goal.pose.position.y = center.point.y
    goal.pose.position.z = 0.0 if planar else center.point.z

    yaw = 0.0
    if start is not None and end is not None:
        yaw = atan2(end.point.y - start.point.y, end.point.x - start.point.x)
    goal.pose.orientation.z = sin(yaw * 0.5)
    goal.pose.orientation.w = cos(yaw * 0.5)
    return goal


class WeldlineGoalNode(Node):
    """Convert 2D YOLO weld-line detections into 3D Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("weldline_goal_node")
        self.bridge = CvBridge()
        self.latest_frame: tuple[Image, Image, CameraInfo] | None = None

        self.weights = str(self.declare_parameter("weights", "").value)
        self.output_frame = str(self.declare_parameter("output_frame", "map").value)
        self.processing_rate_hz = float(self.declare_parameter("processing_rate_hz", 30.0).value)
        self.target_class_id = int(self.declare_parameter("target_class_id", -1).value)
        self.confidence_threshold = float(self.declare_parameter("confidence_threshold", 0.40).value)
        self.nms_threshold = float(self.declare_parameter("nms_threshold", 0.25).value)
        self.depth_window = int(self.declare_parameter("depth_window", 7).value)
        self.use_line_detection = bool(self.declare_parameter("use_line_detection", True).value)
        self.planar_goal = bool(self.declare_parameter("nav2_planar_goal", True).value)
        self.allow_latest_tf = bool(self.declare_parameter("allow_latest_tf_fallback", True).value)
        self.visualize = bool(self.declare_parameter("visualize", False).value)
        self.debug_image_topic = str(self.declare_parameter("debug_image_topic", "/weldline/debug_image").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.session = self.load_session(self.weights)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.declare_parameter("goal_topic", "/weldline/goal_pose").value),
            10,
        )
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10) if self.visualize else None

        color_sub = message_filters.Subscriber(
            self, Image, str(self.declare_parameter("color_topic", "/camera/camera/color/image_raw").value)
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            str(self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw").value),
        )
        info_sub = message_filters.Subscriber(
            self, CameraInfo, str(self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info").value)
        )
        self.sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, info_sub], 10, 0.05)
        self.sync.registerCallback(self.on_frame)
        self.create_timer(1.0 / self.processing_rate_hz, self.process_latest_frame)

    @staticmethod
    def load_session(weights: str):
        import onnxruntime

        if not weights or not Path(weights).is_file():
            raise ValueError(f"ONNX model does not exist: {weights}")
        return onnxruntime.InferenceSession(weights, providers=["CPUExecutionProvider"])

    def on_frame(self, color: Image, depth: Image, info: CameraInfo) -> None:
        self.latest_frame = (color, depth, info)

    def process_latest_frame(self) -> None:
        if self.latest_frame is None:
            return

        color_msg, depth_msg, camera_info = self.latest_frame
        self.latest_frame = None

        try:
            image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth = depth_to_meters(depth_msg, self.bridge)
            if depth is None:
                self.get_logger().warn(f"Unsupported depth encoding: {depth_msg.encoding}", throttle_duration_sec=2.0)
                self.publish_debug_image(image, None, color_msg.header, None, None)
                return

            detection = self.detect_weldline(image)
            if detection is None:
                self.publish_debug_image(image, depth, color_msg.header, None, None)
                return

            projection = self.project_detection(detection, depth, depth_msg.header, camera_info)
            if projection is not None:
                self.goal_pub.publish(projection.pose)
            self.publish_debug_image(image, depth, color_msg.header, detection, None if projection is None else projection.center)
        except Exception as error:  # Keep the camera pipeline alive after a bad frame.
            self.get_logger().error(f"Failed to publish weldline goal: {error}")

    def goal_from_detection(self, detection: Detection2D, depth: np.ndarray, header, info: CameraInfo) -> PoseStamped | None:
        projection = self.project_detection(detection, depth, header, info)
        return None if projection is None else projection.pose

    def project_detection(
        self,
        detection: Detection2D,
        depth: np.ndarray,
        header,
        info: CameraInfo,
    ) -> ProjectedDetection | None:
        center = self.transform_point(project_pixel(detection.center, depth, info, self.depth_window), header)
        if center is None:
            return None

        start = self.transform_point(project_pixel(detection.line_start, depth, info, self.depth_window), header)
        end = self.transform_point(project_pixel(detection.line_end, depth, info, self.depth_window), header)
        return ProjectedDetection(pose_from_points(center, start, end, self.planar_goal), center)

    def publish_debug_image(
        self,
        image: np.ndarray,
        depth: np.ndarray | None,
        header,
        detection: Detection2D | None,
        center_3d: PointStamped | None,
    ) -> None:
        if self.debug_image_pub is None:
            return

        debug_image = render_debug_image(image, depth, detection, center_3d)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)

    def detect_weldline(self, image: np.ndarray) -> Detection2D | None:
        network_input, scale, pad_x, pad_y = letterbox(image)
        input_name = self.session.get_inputs()[0].name
        output = self.session.run(None, {input_name: to_yolo_tensor(network_input)})[0]
        rows = normalize_yolo_output(output)

        boxes: list[list[int]] = []
        scores: list[float] = []
        for row in rows:
            parsed = self.parse_detection_row(row, scale, pad_x, pad_y, image.shape[1], image.shape[0])
            if parsed is None:
                continue
            box, score = parsed
            boxes.append(box)
            scores.append(score)

        kept = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        if len(kept) == 0:
            return None

        best = max(np.asarray(kept).reshape(-1), key=lambda index: scores[int(index)])
        x, y, width, height = boxes[int(best)]
        center = (x + width // 2, y + height // 2)
        line_start = (x, y + height // 2)
        line_end = (x + width, y + height // 2)

        if self.use_line_detection:
            line_start, line_end, center = detect_line(image, x, y, width, height, line_start, line_end, center)

        return Detection2D((x, y, width, height), center, line_start, line_end, scores[int(best)])

    def parse_detection_row(
        self,
        row: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
        image_width: int,
        image_height: int,
    ) -> tuple[list[int], float] | None:
        if row.size < 5:
            return None

        score = float(row[4])
        class_id = -1
        if row.size > 6:
            class_scores = row[5:] if row.size != 84 else row[4:]
            objectness = float(row[4]) if row.size != 84 else 1.0
            if class_scores.size == 0:
                return None
            class_id = int(np.argmax(class_scores))
            score = objectness * float(class_scores[class_id])

        if self.target_class_id >= 0 and class_id >= 0 and class_id != self.target_class_id:
            return None
        if score < self.confidence_threshold:
            return None

        x1, y1, x2, y2 = map(float, row[:4])
        if x2 <= x1 or y2 <= y1:
            x1, y1, x2, y2 = x1 - x2 * 0.5, y1 - y2 * 0.5, x1 + x2 * 0.5, y1 + y2 * 0.5

        x1 = np.clip((x1 - pad_x) / scale, 0, image_width - 1)
        y1 = np.clip((y1 - pad_y) / scale, 0, image_height - 1)
        x2 = np.clip((x2 - pad_x) / scale, 0, image_width - 1)
        y2 = np.clip((y2 - pad_y) / scale, 0, image_height - 1)
        width = max(1, int(round(x2 - x1)))
        height = max(1, int(round(y2 - y1)))
        return [int(round(x1)), int(round(y1)), width, height], score

    def transform_point(self, point: Point | None, header) -> PointStamped | None:
        if point is None:
            return None

        source = PointStamped()
        source.header = header
        source.point = point
        if not self.output_frame or self.output_frame == header.frame_id:
            return source

        try:
            return self.tf_buffer.transform(source, self.output_frame, timeout=Duration(seconds=0.02))
        except TransformException:
            if not self.allow_latest_tf:
                return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            return tf2_geometry_msgs.do_transform_point(source, transform)
        except TransformException:
            return None


def letterbox(image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(MODEL_SIZE / width, MODEL_SIZE / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    canvas = np.full((MODEL_SIZE, MODEL_SIZE, 3), 114, dtype=np.uint8)
    pad_x = (MODEL_SIZE - resized.shape[1]) // 2
    pad_y = (MODEL_SIZE - resized.shape[0]) // 2
    canvas[pad_y : pad_y + resized.shape[0], pad_x : pad_x + resized.shape[1]] = resized
    return canvas, scale, pad_x, pad_y


def to_yolo_tensor(image: np.ndarray) -> np.ndarray:
    return image[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0


def normalize_yolo_output(output: np.ndarray) -> np.ndarray:
    rows = np.squeeze(output)
    if rows.ndim == 1:
        return rows.reshape(1, -1)
    if rows.ndim == 2 and rows.shape[0] < rows.shape[1]:
        return rows.T
    return rows


def detect_line(
    image: np.ndarray,
    x: int,
    y: int,
    width: int,
    height: int,
    fallback_start: tuple[int, int],
    fallback_end: tuple[int, int],
    fallback_center: tuple[int, int],
) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
    crop = image[y : y + height, x : x + width]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 40, 120)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        25,
        minLineLength=max(10, width // 4),
        maxLineGap=12,
    )
    if lines is None:
        return fallback_start, fallback_end, fallback_center

    line = max(lines[:, 0], key=lambda item: (item[2] - item[0]) ** 2 + (item[3] - item[1]) ** 2)
    start = (x + int(line[0]), y + int(line[1]))
    end = (x + int(line[2]), y + int(line[3]))
    center = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
    return start, end, center


def render_debug_image(
    image: np.ndarray,
    depth: np.ndarray | None,
    detection: Detection2D | None,
    center_3d: PointStamped | None,
) -> np.ndarray:
    color_panel = image.copy()
    depth_panel = depth_to_colormap(depth, image.shape[:2])
    if detection is None:
        return stack_debug_panels(color_panel, depth_panel)

    label = f"conf={detection.confidence:.2f}"
    if center_3d is not None:
        point = center_3d.point
        frame = center_3d.header.frame_id
        label = f"{label} x={point.x:.2f} y={point.y:.2f} z={point.z:.2f} {frame}"

    draw_detection_overlay(color_panel, detection, label)
    draw_detection_overlay(depth_panel, detection, label)
    return stack_debug_panels(color_panel, depth_panel)


def depth_to_colormap(depth: np.ndarray | None, image_shape: tuple[int, int]) -> np.ndarray:
    height, width = image_shape
    if depth is None:
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(panel, "depth unavailable", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)
        return panel

    valid_depth = depth[np.isfinite(depth) & (depth > 0.05)]
    if valid_depth.size == 0:
        normalized = np.zeros(depth.shape, dtype=np.uint8)
    else:
        min_depth = float(np.percentile(valid_depth, 2))
        max_depth = float(np.percentile(valid_depth, 98))
        if max_depth <= min_depth:
            max_depth = min_depth + 0.1
        clipped = np.clip(depth, min_depth, max_depth)
        clipped = np.where(np.isfinite(clipped) & (depth > 0.05), clipped, min_depth)
        normalized = ((clipped - min_depth) * 255.0 / (max_depth - min_depth)).astype(np.uint8)
        normalized[~np.isfinite(depth) | (depth <= 0.05)] = 0

    panel = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    if panel.shape[:2] != (height, width):
        panel = cv2.resize(panel, (width, height), interpolation=cv2.INTER_NEAREST)
    return panel


def stack_debug_panels(color_panel: np.ndarray, depth_panel: np.ndarray) -> np.ndarray:
    if depth_panel.shape[:2] != color_panel.shape[:2]:
        depth_panel = cv2.resize(depth_panel, (color_panel.shape[1], color_panel.shape[0]), interpolation=cv2.INTER_NEAREST)
    separator = np.full((color_panel.shape[0], 4, 3), 32, dtype=np.uint8)
    return np.hstack((color_panel, separator, depth_panel))


def draw_detection_overlay(image: np.ndarray, detection: Detection2D, label: str) -> None:
    x, y, width, height = detection.box
    x2 = x + width
    y2 = y + height
    cv2.rectangle(image, (x, y), (x2, y2), (0, 220, 0), 2)
    cv2.line(image, detection.line_start, detection.line_end, (255, 180, 0), 2)
    cv2.circle(image, detection.center, 4, (0, 0, 255), -1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    label_x = max(0, min(x, image.shape[1] - text_width - 8))
    label_top = y - text_height - baseline - 8
    if label_top < 0:
        label_top = min(image.shape[0] - text_height - baseline - 8, y2 + 4)
    label_top = max(0, label_top)
    label_bottom = min(image.shape[0] - 1, label_top + text_height + baseline + 8)
    cv2.rectangle(
        image,
        (label_x, label_top),
        (min(image.shape[1] - 1, label_x + text_width + 8), label_bottom),
        (0, 220, 0),
        -1,
    )
    cv2.putText(image, label, (label_x + 4, label_bottom - baseline - 4), font, font_scale, (0, 0, 0), thickness)


def main() -> None:
    rclpy.init()
    node = WeldlineGoalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
