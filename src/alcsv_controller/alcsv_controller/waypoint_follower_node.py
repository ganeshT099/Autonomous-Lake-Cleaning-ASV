#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
import math


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.current_pose = None
        self.current_waypoint = None

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        self.waypoint_sub = self.create_subscription(
            PointStamped,
            '/coverage/waypoint',
            self.waypoint_callback,
            10
        )

        # Publisher (just for visualization of movement state)
        self.goal_pub = self.create_publisher(
            PointStamped,
            '/goal_pose',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Waypoint Follower Started")

    def pose_callback(self, msg):
        self.current_pose = msg

    def waypoint_callback(self, msg):
        self.current_waypoint = msg
        self.get_logger().info("New waypoint received")

    def control_loop(self):

        if self.current_pose is None or self.current_waypoint is None:
            return

        dx = self.current_waypoint.point.x - self.current_pose.pose.position.x
        dy = self.current_waypoint.point.y - self.current_pose.pose.position.y

        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.5:
            self.get_logger().info("Waypoint reached")
            return

        # Just republish goal for now (placeholder control)
        self.goal_pub.publish(self.current_waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
