"""Verify sensor connections while keeping the car stopped."""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class QuickChaseController(Node):
    def __init__(self):
        super().__init__("quick_chase_controller")

        self.scan_received = False
        self.odom_received = False

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

        self.timer = self.create_timer(0.05, self.publish_stop)
        self.get_logger().info(
            "Controller started in zero-speed verification mode."
        )

    def scan_callback(self, message):
        if not self.scan_received:
            self.get_logger().info(
                f"LiDAR connected: {len(message.ranges)} beams."
            )
            self.scan_received = True

    def odom_callback(self, message):
        if not self.odom_received:
            position = message.pose.pose.position
            self.get_logger().info(
                f"Odometry connected: x={position.x:.3f}, "
                f"y={position.y:.3f}, frame={message.header.frame_id}."
            )
            self.odom_received = True

    def publish_stop(self):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "ego_racecar/base_link"
        command.drive.speed = 0.0
        command.drive.steering_angle = 0.0
        self.drive_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = QuickChaseController()

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
