#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence_node')

        # ==============================
        # PARAMETERS
        # ==============================
        self.geofence_radius = 50.0
        self.center_x = 0.0
        self.center_y = 0.0

        # ==============================
        # SUBSCRIBER
        # ==============================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # ==============================
        # MARKER PUBLISHER
        # ==============================
        self.marker_pub = self.create_publisher(
            Marker,
            '/geofence_marker',
            10
        )

        # ==============================
        # TIMER
        # ==============================
        self.timer = self.create_timer(
            1.0,
            self.publish_geofence_marker
        )

        self.get_logger().info(
            f"Geofence Monitor Started (Radius = {self.geofence_radius} m)"
        )

    # ==========================================
    # MUST BE INDENTED INSIDE CLASS
    # ==========================================

    def pose_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y

        distance = ((x - self.center_x)**2 + (y - self.center_y)**2) ** 0.5

        if distance > self.geofence_radius:
            self.get_logger().warn("⚠️ GEOFENCE VIOLATION!")

    def publish_geofence_marker(self):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.id = 0

        import math
        from geometry_msgs.msg import Point

        for angle in range(0, 361, 5):
            p = Point()
            p.x = self.center_x + self.geofence_radius * math.cos(math.radians(angle))
            p.y = self.center_y + self.geofence_radius * math.sin(math.radians(angle))
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)
