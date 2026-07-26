"""ROS 2 scenario commander and PointCloud2 wall-alignment node."""
from __future__ import annotations

from collections import deque
from math import atan2, cos, isfinite, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .point_cloud_xyz import decode_point_cloud_xyz
from .scenario import Pose2D, ScenarioPlanner, format_fsm_command, load_scenario, normalize_angle, normalize_parallel_angle, wall_heading_error
from .wall_alignment import Point2D, WallConfig, estimate_wall


def _yaw(quaternion) -> float:
    return atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def _rotate(point, quaternion):
    x, y, z = point
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    tx, ty, tz = 2 * (qy * z - qz * y), 2 * (qz * x - qx * z), 2 * (qx * y - qy * x)
    return x + qw * tx + qy * tz - qz * ty, y + qw * ty + qz * tx - qx * tz, z + qw * tz + qx * ty - qy * tx


class ScenarioCommanderNode(Node):
    def __init__(self):
        super().__init__("scenario_commander_node")
        self.map_topic = self.declare_parameter("global_map_topic", "/point_lio/global_map_refined").value
        self.reference_link = self.declare_parameter("reference_link", "base_link").value
        self.scenario_file = self.declare_parameter("scenario_file", "").value
        self.rate = float(self.declare_parameter("wall_processing_rate_hz", 5.0).value)
        self.tf_timeout = float(self.declare_parameter("wall_tf_timeout_sec", 0.05).value)
        self.min_range = float(self.declare_parameter("wall_minimum_range", 0.25).value)
        self.max_range = float(self.declare_parameter("wall_search_radius", 3.0).value)
        self.min_height = float(self.declare_parameter("wall_minimum_height", -2.0).value)
        self.max_height = float(self.declare_parameter("wall_maximum_height", 2.0).value)
        self.voxel_size = float(self.declare_parameter("wall_xy_voxel_size", 0.04).value)
        self.min_span = float(self.declare_parameter("wall_minimum_vertical_span", 0.35).value)
        self.min_voxel_points = int(self.declare_parameter("wall_minimum_voxel_points", 3).value)
        self.stride = int(self.declare_parameter("wall_point_stride", 1).value)
        self.smoothing = float(self.declare_parameter("wall_angle_smoothing_alpha", 1.0).value)
        self.simulator_mode = bool(self.declare_parameter("simulator_mode", False).value)
        self.simulator_odom_topic = self.declare_parameter("simulator_odom_topic", "/odom_gt").value
        self.simulator_wall_heading = float(self.declare_parameter("simulator_wall_heading_deg", 0.0).value) * 0.017453292519943295
        self.simulator_skip_detector = bool(self.declare_parameter("simulator_skip_detector_waypoint", True).value)
        self.wall_config = WallConfig(
            float(self.declare_parameter("wall_ransac_distance_threshold", 0.04).value), int(self.declare_parameter("wall_ransac_iterations", 300).value), int(self.declare_parameter("wall_max_candidates", 6).value), int(self.declare_parameter("wall_minimum_inliers", 30).value), float(self.declare_parameter("wall_minimum_length", 0.8).value), float(self.declare_parameter("wall_max_fit_rmse", 0.04).value), self.declare_parameter("wall_selection", "tracked").value, self.declare_parameter("wall_sector", "front").value, float(self.declare_parameter("wall_sector_half_angle_deg", 70.0).value) * 0.017453292519943295, float(self.declare_parameter("wall_tracking_max_angle_deg", 20.0).value) * 0.017453292519943295)
        self.buffer, self.listener, self.cached_map, self.tracked_angle, self.filtered_angle = Buffer(), None, None, None, None
        self.listener = TransformListener(self.buffer, self)
        self.angle_pub = self.create_publisher(Float64, self.declare_parameter("wall_angle_topic", "/commander/wall_alignment/angle_deg").value, 10)
        self.distance_pub = self.create_publisher(Float64, self.declare_parameter("wall_distance_topic", "/commander/wall_alignment/distance_m").value, 10)
        self.valid_pub = self.create_publisher(Bool, self.declare_parameter("wall_valid_topic", "/commander/wall_alignment/valid").value, 10)
        self.metrics_pub = self.create_publisher(Vector3Stamped, self.declare_parameter("wall_metrics_topic", "/commander/wall_alignment/metrics").value, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self.declare_parameter("wall_marker_topic", "/commander/wall_alignment/markers").value, 5)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, self.map_topic, self.on_map, qos)
        self.create_timer(1.0 / self.rate, self.process_wall)
        self.simulator_pose = None
        self.rl_enable_deadline = monotonic() + 2.0
        if self.simulator_mode: self.create_subscription(Odometry, self.simulator_odom_topic, self.on_simulator_odom, qos_profile_sensor_data)
        self.load_control()

    def load_control(self) -> None:
        self.planner = ScenarioPlanner(load_scenario(self.scenario_file))
        scenario = self.planner.scenario
        if scenario.control.robot_frame != self.reference_link: raise ValueError("scenario robot_frame must match reference_link")
        self.command_pub = self.create_publisher(String, scenario.control.command_topic, 10)
        self.status_pub = self.create_publisher(String, scenario.control.status_topic, 10)
        self.goal_samples = deque(maxlen=scenario.detector.stable_samples)
        self.goal_sub = self.create_subscription(PoseStamped, scenario.detector.goal_topic, self.on_goal, 10)
        self.create_timer(1.0 / scenario.control.rate_hz, self.process_control)

    def on_map(self, message: PointCloud2) -> None:
        try:
            self.cached_map = (message.header, decode_point_cloud_xyz(message, self.stride))
            self.get_logger().info(f"Cached global map: {len(self.cached_map[1])} finite points")
        except ValueError as error: self.get_logger().error(f"Rejected global map: {error}")

    def on_simulator_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.simulator_pose = Pose2D(pose.position.x, pose.position.y, _yaw(pose.orientation))
        if self.simulator_skip_detector and not self.planner.detector_resolved:
            self.planner.set_detector_waypoint(self.simulator_pose)
            self.get_logger().info("Simulator mode locked the detector waypoint at the initial /odom_gt pose")

    def on_goal(self, message: PoseStamped) -> None:
        if self.planner.detector_resolved or not message.header.frame_id: return
        scenario = self.planner.scenario
        try:
            if message.header.frame_id != scenario.frame_id: message = self.buffer.transform(message, scenario.frame_id, timeout=Duration(seconds=scenario.control.tf_timeout))
        except TransformException as error:
            self.get_logger().warning(f"Cannot transform detector waypoint: {error}"); return
        pose = Pose2D(message.pose.position.x, message.pose.position.y, _yaw(message.pose.orientation))
        if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)): return
        self.goal_samples.append(pose)
        if len(self.goal_samples) < self.goal_samples.maxlen: return
        mean_x = sum(item.x for item in self.goal_samples) / len(self.goal_samples)
        mean_y = sum(item.y for item in self.goal_samples) / len(self.goal_samples)
        mode = scenario.waypoints[0].yaw_mode
        multiplier = 2 if mode == "parallel" else 1
        yaw = atan2(sum(sin(multiplier * item.yaw) for item in self.goal_samples), sum(cos(multiplier * item.yaw) for item in self.goal_samples)) / multiplier
        difference = normalize_parallel_angle if mode == "parallel" else normalize_angle
        if max(((item.x - mean_x) ** 2 + (item.y - mean_y) ** 2) ** 0.5 for item in self.goal_samples) <= scenario.detector.max_position_spread and max(abs(difference(item.yaw, yaw)) for item in self.goal_samples) <= scenario.detector.max_yaw_spread:
            self.planner.set_detector_waypoint(Pose2D(mean_x, mean_y, yaw)); self.destroy_subscription(self.goal_sub); self.get_logger().info("Locked stable detector waypoint")

    def process_wall(self) -> None:
        if self.cached_map is None: return self.publish_invalid()
        header, points = self.cached_map
        try: transform = self.buffer.lookup_transform(self.reference_link, header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout))
        except TransformException: return self.publish_invalid()
        translation, rotation, voxels = transform.transform.translation, transform.transform.rotation, {}
        for item in points:
            x, y, z = _rotate((item.x, item.y, item.z), rotation); x, y, z = x + translation.x, y + translation.y, z + translation.z
            distance = (x * x + y * y) ** 0.5
            if self.min_range <= distance <= self.max_range and self.min_height <= z <= self.max_height:
                cell = voxels.setdefault((int(x // self.voxel_size), int(y // self.voxel_size)), [0.0, 0.0, z, z, 0]); cell[0] += x; cell[1] += y; cell[2] = min(cell[2], z); cell[3] = max(cell[3], z); cell[4] += 1
        wall_points = [Point2D(cell[0] / cell[4], cell[1] / cell[4]) for cell in voxels.values() if cell[4] >= self.min_voxel_points and cell[3] - cell[2] >= self.min_span]
        estimate = estimate_wall(wall_points, self.wall_config, self.tracked_angle)
        if estimate is None: return self.publish_invalid()
        angle = estimate.angle if self.filtered_angle is None else normalize_parallel_angle(self.filtered_angle + self.smoothing * normalize_parallel_angle(estimate.angle - self.filtered_angle))
        self.tracked_angle = self.filtered_angle = angle
        self.valid_pub.publish(Bool(data=True)); self.angle_pub.publish(Float64(data=angle * 57.295779513)); self.distance_pub.publish(Float64(data=estimate.distance)); metrics = Vector3Stamped(); metrics.header.frame_id = self.reference_link; metrics.header.stamp = self.get_clock().now().to_msg(); metrics.vector.x, metrics.vector.y, metrics.vector.z = angle * 57.295779513, estimate.distance, estimate.rmse; self.metrics_pub.publish(metrics)

    def publish_invalid(self) -> None:
        self.tracked_angle = self.filtered_angle = None; self.valid_pub.publish(Bool(data=False)); self.angle_pub.publish(Float64(data=float("nan"))); self.distance_pub.publish(Float64(data=float("nan")))

    def robot_pose(self):
        if self.simulator_mode: return self.simulator_pose
        scenario = self.planner.scenario
        try: transform = self.buffer.lookup_transform(scenario.frame_id, scenario.control.robot_frame, rclpy.time.Time(), timeout=Duration(seconds=scenario.control.tf_timeout))
        except TransformException: return None
        return Pose2D(transform.transform.translation.x, transform.transform.translation.y, _yaw(transform.transform.rotation))

    def process_control(self) -> None:
        now, pose = monotonic(), self.robot_pose()
        if self.simulator_mode and now < self.rl_enable_deadline:
            self.command_pub.publish(String(data="RL"))
            self.command_pub.publish(String(data=format_fsm_command(0.0, 0.0, 0.0)))
            return
        wall_error = wall_heading_error(pose.yaw, self.simulator_wall_heading) if self.simulator_mode and pose else self.filtered_angle
        output = self.planner.update(pose, wall_error, now)
        self.command_pub.publish(String(data=format_fsm_command(output.vx, output.vy, output.wz)))
        self.status_pub.publish(String(data=f"{output.state}: {output.detail}"))


def main() -> None:
    rclpy.init(); node = ScenarioCommanderNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
