#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
import math


class RobotSim(Node):

    def __init__(self):
        super().__init__('robot_sim')

        # Current robot state
        self.x = 0.0
        self.y = 0.0
        self.speed = 1.0  # m/s

        self.target_x = None
        self.target_y = None

        # Subscribe to waypoint
        self.create_subscription(
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

        # Timer for motion update
        self.timer = self.create_timer(0.1, self.update_motion)

        self.get_logger().info("Robot Simulator Started")

    def waypoint_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.get_logger().info(
            f"New Target: ({self.target_x:.2f}, {self.target_y:.2f})"
        )

    def update_motion(self):

        if self.target_x is None:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.2:
            return  # reached waypoint

        # Normalize direction
        vx = dx / distance
        vy = dy / distance

        # Move robot
        self.x += vx * self.speed * 0.1
        self.y += vy * self.speed * 0.1

        # Publish pose
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
