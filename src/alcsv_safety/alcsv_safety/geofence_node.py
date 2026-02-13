#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import math


class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence_node')

        self.geofence_radius = 50.0
        self.center_x = 0.0
        self.center_y = 0.0

        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            Bool,
            '/geofence_status',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/geofence_marker',
            10
        )

        # Timer for visualization
        self.timer = self.create_timer(
            1.0,
            self.publish_geofence_marker
        )

        self.get_logger().info("Geofence Node Started (Radius = 50m)")

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        distance = math.sqrt((x - self.center_x)**2 + (y - self.center_y)**2)
        inside = distance <= self.geofence_radius

        status_msg = Bool()
        status_msg.data = inside
        self.status_pub.publish(status_msg)

        if not inside:
            self.get_logger().warn(f"🚨 Geofence Breach! Distance = {distance:.2f} m")

    def publish_geofence_marker(self):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for angle in range(0, 361, 5):
            p = Point()
            p.x = self.center_x + self.geofence_radius * math.cos(math.radians(angle))
            p.y = self.center_y + self.geofence_radius * math.sin(math.radians(angle))
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
