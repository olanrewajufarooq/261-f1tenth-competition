
"""Observe controller decisions while keeping the vehicle stopped."""

import math
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from quick_chase_controller.collision_avoidance import CollisionAvoidance
from quick_chase_controller.occupancy_map import OccupancyMap
from quick_chase_controller.pure_pursuit_controller import PurePursuitController
from quick_chase_controller.reference_path import ReferencePath


class ControllerNode(Node):
    """Connect ROS sensor messages to the independent controller classes."""

    def __init__(self):
        super().__init__("quick_chase_controller")

        maps = Path(
            get_package_share_directory("f1tenth_gym_ros")
        ) / "maps"

        path = ReferencePath.from_csv(maps / "levine_centerline.csv")
        self.occupancy_map = OccupancyMap.from_yaml(maps / "levine.yaml")
        self.controller = PurePursuitController(path)
        self.avoidance = CollisionAvoidance()

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("maximum_speed", 0.2)
        self.motion_enabled = self.get_parameter("enable_motion").value
        self.maximum_speed = self.get_parameter("maximum_speed").value

        if not math.isfinite(self.maximum_speed) or not 0 < self.maximum_speed <= 0.6:
            raise ValueError("maximum_speed must be in (0, 0.6].")

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

        self.timer = self.create_timer(0.05, self.control_callback)
        self.get_logger().info(
            "Controller ready. Motion requires enable_motion:=true."
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

        if self.scan is None or self.odom is None:
            self.publish_stop()
            self.report("Waiting for LiDAR and odometry.")
            return

        if (
            now - self.scan_received_at > 0.25
            or now - self.odom_received_at > 0.25
        ):
            self.publish_stop()
            self.report("Stopped: sensor data is stale.")
            return

        started_at = time.monotonic()

        try:
            pose = self.vehicle_pose()
            steering = self.controller.calculate_steering(*pose)
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
            horizon = min(0.75, scan.range_max - 0.8)
            if horizon <= 0.0:
                raise ValueError("LiDAR range is too short.")

            map_clearance = self.avoidance.clearance_on_map(
                self.occupancy_map, pose, steering, horizon=horizon
            )
            lidar_clearance = self.avoidance.clearance_along_arc(
                points, steering, horizon=horizon
            )
            clearance = min(map_clearance, lidar_clearance)
            candidate_speed = self.avoidance.limit_speed(self.maximum_speed, clearance)

        except ValueError as error:
            self.publish_stop()
            self.report(f"Stopped: {error}")
            return

        finished_at = time.monotonic()
        elapsed_ms = (finished_at - started_at) * 1000.0

        if (
            elapsed_ms > 50.0
            or finished_at - self.scan_received_at > 0.25
            or finished_at - self.odom_received_at > 0.25
        ):
            self.publish_stop()
            self.report("Stopped: calculation deadline or sensor freshness exceeded.")
            return

        if self.motion_enabled:
            dt = min(0.05, finished_at - self.last_publish_at)
            speed = min(candidate_speed, self.last_speed + 0.5 * dt)
            self.publish_command(speed, steering)
        else:
            self.publish_stop()

        self.report(
            f"steering={steering:.3f} rad, "
            f"map={map_clearance:.2f} m, "
            f"lidar={lidar_clearance:.2f} m, "
            f"candidate={candidate_speed:.2f} m/s, "
            f"compute={elapsed_ms:.1f} ms; published={self.last_speed:.2f} m/s"
        )

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