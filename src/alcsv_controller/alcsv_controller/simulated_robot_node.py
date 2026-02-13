#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
import math


class SimulatedRobot(Node):

    def __init__(self):
        super().__init__('simulated_robot')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.speed = 1.0  # m/s

        self.target_x = None
        self.target_y = None

        # Subscribe to coverage waypoints
        self.sub = self.create_subscription(
            PointStamped,
            '/coverage/waypoint',
            self.waypoint_callback,
            10
        )

        # Publish robot pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )

        # Timer loop (10 Hz)
        self.timer = self.create_timer(0.1, self.update_motion)

        self.get_logger().info("Simulated Robot Started")

    def waypoint_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y

    def update_motion(self):

        if self.target_x is None:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.2:
            return

        angle = math.atan2(dy, dx)

        self.x += self.speed * 0.1 * math.cos(angle)
        self.y += self.speed * 0.1 * math.sin(angle)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
