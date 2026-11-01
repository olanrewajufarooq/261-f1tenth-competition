"""Fail-closed, opt-in integration of steering and clearance checks."""

import math
import time
from bisect import bisect_right
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from quick_chase_controller.collision_avoidance import CollisionAvoidance
from quick_chase_controller.clearance_path_planner import ClearancePathPlanner
from quick_chase_controller.sensor_safety import SensorSafety
from quick_chase_controller.occupancy_map import OccupancyMap
from quick_chase_controller.pure_pursuit_controller import PurePursuitController
from quick_chase_controller.reference_path import ReferencePath
from quick_chase_controller.speed_profile import SpeedProfile


class ControllerNode(Node):
    """Connect ROS sensor messages to the independent controller classes."""

    def __init__(self):
        super().__init__("quick_chase_controller")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("maximum_speed", 0.6)
        self.declare_parameter("lookahead", 0.65)
        self.declare_parameter("lateral_acceleration", 0.6)
        self.motion_enabled = self.get_parameter("enable_motion").value
        self.maximum_speed = self.get_parameter("maximum_speed").value
        self.lookahead = self.get_parameter("lookahead").value
        if not math.isfinite(self.maximum_speed) or not 0 < self.maximum_speed <= 1.5:
            raise ValueError("maximum_speed must be in (0, 1.5].")
        if not math.isfinite(self.lookahead) or self.lookahead <= 0.0:
            raise ValueError("lookahead must be finite and positive.")
        self.controller = None
        self.startup_error = None
        self.avoidance = CollisionAvoidance()
        self.speed_profile = SpeedProfile(
            lateral_acceleration=self.get_parameter("lateral_acceleration").value)

        self.command_deadline_failure = None
        self.last_speed = 0.0
        self.last_publish_at = time.monotonic()
        self.scan = None
        self.odom = None
        self.scan_received_at = None
        self.odom_received_at = None
        self.last_report_at = 0.0

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback,
            qos_profile_sensor_data,
        )

        # A failed or unfinished startup never exposes a controller to motion.
        self.publish_stop()
        try:
            maps = Path(get_package_share_directory("f1tenth_gym_ros")) / "maps"
            baseline = ReferencePath.from_csv(maps / "levine_centerline.csv")
            self.occupancy_map = OccupancyMap.from_yaml(maps / "levine.yaml")
            expected_origin = (-51.224998, -51.224998, 0.0)
            if (self.occupancy_map.image.size != (2048, 2048)
                    or not math.isclose(self.occupancy_map.resolution, 0.05, abs_tol=1e-9)
                    or any(not math.isclose(actual, expected, abs_tol=1e-5)
                            for actual, expected in zip(self.occupancy_map.origin,
                                                        expected_origin))):
                raise ValueError("Unexpected Levine map geometry; refusing motion.")
            planner = ClearancePathPlanner()
            path = planner.plan(baseline, self.occupancy_map)
            if not planner.diagnostics.get("success", False):
                raise ValueError(f"Planner rejected route: {planner.diagnostics}")
            # Validate the returned geometry before handing it to steering.
            for index, (x, y) in enumerate(path.points):
                end = path.points[(index + 1) % len(path.points)]
                yaw = math.atan2(end[1] - y, end[0] - x)
                length = path.segment_lengths[index]
                samples = max(1, math.ceil(length / 0.025))
                for sample in range(samples + 1):
                    fraction = sample / samples
                    rear_x = x + fraction * (end[0] - x)
                    rear_y = y + fraction * (end[1] - y)
                    padding = self.avoidance.clearance_margin + 0.025
                    if not self.occupancy_map.is_rectangle_free(
                            rear_x + 0.17145 * math.cos(yaw),
                            rear_y + 0.17145 * math.sin(yaw), yaw,
                            0.58 + 2 * padding, 0.31 + 2 * padding):
                        raise ValueError(f"Planned footprint blocked at segment {index}.")
            self.controller = PurePursuitController(path, lookahead=self.lookahead)
            self.get_logger().info(f"Planning diagnostics: {planner.diagnostics}")
        except (ValueError, OSError, RuntimeError) as error:
            self.startup_error = str(error)
            self.get_logger().error(f"Startup stopped: {error}")
        self.publish_stop()

        self.timer = self.create_timer(0.05, self.control_callback)
        self.get_logger().info(
            "Controller startup complete. Motion requires a validated route and enable_motion:=true."
        )

    def scan_callback(self, message):
        self.scan = message
        self.scan_received_at = time.monotonic()

    def odom_callback(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()

    def vehicle_pose(self):
        """Extract a planar map-frame pose from the latest odometry."""
        if self.odom.header.frame_id != "map":
            raise ValueError("Odometry must use the map frame.")

        position = self.odom.pose.pose.position
        quaternion = self.odom.pose.pose.orientation

        values = (
            position.x, position.y,
            quaternion.x, quaternion.y, quaternion.z, quaternion.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Odometry contains nonfinite values.")

        norm = math.sqrt(
            quaternion.x ** 2 + quaternion.y ** 2
            + quaternion.z ** 2 + quaternion.w ** 2
        )
        if norm < 1e-6:
            raise ValueError("Odometry quaternion is invalid.")

        qx = quaternion.x / norm
        qy = quaternion.y / norm
        qz = quaternion.z / norm
        qw = quaternion.w / norm

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return position.x, position.y, yaw

    def control_callback(self):
        # Publish only after validating inputs and the calculated command.

        now = time.monotonic()
        if self.command_deadline_failure is None:
            self.command_deadline_failure = SensorSafety.command_gap_reason(
                now, self.last_publish_at, self.last_speed)
        if self.command_deadline_failure:
            self.publish_stop()
            self.report(f"stop_reason={self.command_deadline_failure}; restart required")
            return

        if self.controller is None:
            self.publish_stop()
            self.report(f"stop_reason=startup: {self.startup_error}")
            return
        reason = self.sensor_freshness(now)
        if reason:
            self.publish_stop()
            self.report(f"stop_reason={reason}")
            return

        started_at = time.monotonic()

        context = "x=unknown y=unknown yaw=unknown path_error=unknown segment=unknown"
        try:
            pose = self.vehicle_pose()
            progress, path_error = self.controller.reference_path.project(*pose[:2])
            segment = bisect_right(
                self.controller.reference_path.cumulative_distances, progress) - 1
            context = (f"x={pose[0]:.5f} y={pose[1]:.5f} yaw={pose[2]:.5f} "
                        f"path_error={path_error:.3f} segment={segment}")
            steering = self.controller.calculate_steering(*pose)
            velocity = self.odom.twist.twist.linear
            measured_speed = math.hypot(velocity.x, velocity.y)
            if not math.isfinite(measured_speed):
                raise ValueError("Odometry speed is nonfinite.")
            scan = self.scan

            if scan.header.frame_id != "ego_racecar/laser":
                raise ValueError("Unexpected LiDAR coordinate frame.")

            points = self.avoidance.scan_to_points(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
            )

            observed_angle_max = (
                scan.angle_min
                + (len(scan.ranges) - 1) * scan.angle_increment
            )
            if (
                scan.angle_min > -math.pi / 2.0
                or observed_angle_max < math.pi / 2.0
            ):
                raise ValueError("Scan does not cover the forward hemisphere.")

            # Reserve space for the body and sensor offset within range_max.
            horizon = self.avoidance.required_horizon(measured_speed, self.maximum_speed)
            if horizon > scan.range_max - 0.8:
                raise ValueError("LiDAR range cannot cover measured-speed stopping horizon.")

            preview_distance = max(self.lookahead, horizon)
            nominal_speed = self.speed_profile.calculate(
                self.controller.reference_path, progress, self.maximum_speed,
                preview_distance)
            map_clearance = self.avoidance.clearance_on_map(
                self.occupancy_map, pose, steering, horizon=horizon
            )
            lidar_clearance = self.avoidance.clearance_along_arc(
                points, steering, horizon=horizon
            )
            clearance = min(map_clearance, lidar_clearance)
            limiting_check = "map" if map_clearance <= lidar_clearance else "lidar"
            candidate_speed = self.avoidance.limit_speed(nominal_speed, clearance)
            stop_reason = "none"
            if clearance < self.avoidance.stopping_distance(measured_speed):
                candidate_speed = 0.0
                stop_reason = "measured_speed_braking_reserve"
            elif candidate_speed == 0.0:
                stop_reason = "clearance"

        except ValueError as error:
            self.publish_stop()
            elapsed_ms = (time.monotonic() - started_at) * 1000.0
            self.report(f"{context} limiting_check=input_validation "
                        f"compute={elapsed_ms:.1f} ms stop_reason={error}")
            return

        finished_at = time.monotonic()
        elapsed_ms = (finished_at - started_at) * 1000.0

        self.command_deadline_failure = SensorSafety.command_gap_reason(
            finished_at, self.last_publish_at, self.last_speed)
        reason = (self.command_deadline_failure
                    or ("calculation_deadline" if elapsed_ms > 50.0
                        else self.sensor_freshness(finished_at)))
        if reason:
            self.publish_stop()
            self.report(f"{context} limiting_check={limiting_check} "
                        f"stop_reason={reason}; compute={elapsed_ms:.1f} ms")
            return

        if self.motion_enabled:
            dt = min(0.05, finished_at - self.last_publish_at)
            speed = min(candidate_speed, self.last_speed + 0.5 * dt)
            self.publish_command(speed, steering)
        else:
            self.publish_stop()
            stop_reason = "motion_disabled"

        self.report(
            f"x={pose[0]:.5f} y={pose[1]:.5f} yaw={pose[2]:.5f} "
            f"path_error={path_error:.3f} segment={segment} "
            f"limiting_check={limiting_check} stop_reason={stop_reason} "
            f"measured_speed={measured_speed:.3f} steering={steering:.3f} rad, "
            f"map={map_clearance:.2f} m, "
            f"lidar={lidar_clearance:.2f} m, "
            f"nominal={nominal_speed:.2f} candidate={candidate_speed:.2f} m/s, "
            f"compute={elapsed_ms:.1f} ms; published={self.last_speed:.2f} m/s"
        )

    def sensor_freshness(self, now):
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        for name in ("scan", "odom"):
            message = getattr(self, name)
            if message is None:
                return f"waiting_for_{name}"
            stamp = message.header.stamp
            reason = SensorSafety.check(
                getattr(self, f"{name}_received_at"),
                stamp.sec + stamp.nanosec * 1e-9, now, ros_now)
            if reason:
                return f"{name}: {reason}"
        return None

    def report(self, message):
        """Limit diagnostic output to approximately once per second."""
        now = time.monotonic()
        if now - self.last_report_at >= 1.0:
            self.get_logger().info(message)
            self.last_report_at = now

    def publish_command(self, speed, steering):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "ego_racecar/base_link"
        command.drive.speed = float(speed)
        command.drive.steering_angle = float(steering)
        self.drive_pub.publish(command)
        self.last_speed = float(speed)
        self.last_publish_at = time.monotonic()

    def publish_stop(self):
        self.publish_command(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()