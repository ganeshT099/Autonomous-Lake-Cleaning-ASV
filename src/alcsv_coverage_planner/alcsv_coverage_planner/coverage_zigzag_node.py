#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
import math


class ZigZagCoverage(Node):

    def __init__(self):
        super().__init__('zigzag_coverage_node')

        # ==============================
        # QoS (for RViz reliability)
        # ==============================
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ==============================
        # Publishers
        # ==============================
        self.waypoint_pub = self.create_publisher(
            PointStamped,
            '/coverage/waypoint',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path',
            qos
        )

        # ==============================
        # Circular Boundary
        # ==============================
        self.center_x = 0.0
        self.center_y = 0.0
        self.radius = 50.0

        self.spacing = 1.5

        self.waypoints = self.generate_zigzag()
        self.index = 0

        # Publish path continuously (important fix)
        self.path_timer = self.create_timer(1.0, self.publish_path)

        # Publish waypoints sequentially
        self.timer = self.create_timer(1.0, self.publish_waypoint)

        self.get_logger().info("Circular Zig-Zag Coverage Planner started")

    # ======================================================
    # Circular ZigZag Generator
    # ======================================================

    def generate_zigzag(self):

        waypoints = []

        y = self.center_y - self.radius
        direction = 1

        while y <= self.center_y + self.radius:

            inside_term = self.radius**2 - (y - self.center_y)**2

            if inside_term >= 0:

                x_limit = math.sqrt(inside_term)

                x_left = self.center_x - x_limit
                x_right = self.center_x + x_limit

                if direction == 1:
                    waypoints.append((x_left, y))
                    waypoints.append((x_right, y))
                else:
                    waypoints.append((x_right, y))
                    waypoints.append((x_left, y))

                direction *= -1

            y += self.spacing

        return waypoints

    # ======================================================
    # Waypoint Publisher
    # ======================================================

    def publish_waypoint(self):

        if self.index >= len(self.waypoints):
            self.get_logger().info("Coverage completed")
            self.timer.cancel()
            return

        x, y = self.waypoints[self.index]

        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0

        self.waypoint_pub.publish(msg)

        self.get_logger().info(
            f"Published waypoint {self.index}: ({x:.2f}, {y:.2f})"
        )

        self.index += 1

    # ======================================================
    # Full Path Publisher (for RViz)
    # ======================================================

    def publish_path(self):

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = ZigZagCoverage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
