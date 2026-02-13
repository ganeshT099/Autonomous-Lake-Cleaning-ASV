#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PointStamped, Twist
import math


class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(PointStamped, '/coverage/waypoint', self.wp_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.curr_x = None
        self.curr_y = None
        self.curr_yaw = None

        self.target_x = None
        self.target_y = None

        # Control parameters
        self.kp_yaw = 1.5
        self.forward_speed = 0.5   # m/s
        self.slow_speed = 0.25

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Heading Controller started")

    # ---------------- Callbacks ----------------

    def gps_callback(self, msg):
        self.curr_x = msg.latitude
        self.curr_y = msg.longitude

    def imu_callback(self, msg):
        q = msg.orientation
        self.curr_yaw = self.quaternion_to_yaw(q)

    def wp_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y

    # ---------------- Control ----------------

    def control_loop(self):
        if None in (self.curr_x, self.curr_y, self.curr_yaw,
                    self.target_x, self.target_y):
            return

        dx = self.target_x - self.curr_x
        dy = self.target_y - self.curr_y

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.curr_yaw)

        cmd = Twist()

        # Angular control
        cmd.angular.z = self.kp_yaw * yaw_error

        # Speed control
        if abs(yaw_error) < 0.3:
            cmd.linear.x = self.forward_speed
        else:
            cmd.linear.x = self.slow_speed

        self.cmd_pub.publish(cmd)

    # ---------------- Helpers ----------------

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z)
        cosy = 1.0 - 2.0 * (q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

