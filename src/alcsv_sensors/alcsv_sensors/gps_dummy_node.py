#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math


class DummyGPS(Node):

    def __init__(self):
        super().__init__('gps_dummy_node')

        self.publisher_ = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )

        self.timer = self.create_timer(0.2, self.publish_gps)

        # Simulated motion
        self.lat = 12.9716     # example latitude
        self.lon = 77.5946     # example longitude
        self.step = 0.00001

        self.get_logger().info("Dummy GPS node started")

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"

        # Simple forward motion
        self.lat += self.step * math.cos(0.1)
        self.lon += self.step * math.sin(0.1)

        msg.latitude = self.lat
        msg.longitude = self.lon
        msg.altitude = 0.0

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
