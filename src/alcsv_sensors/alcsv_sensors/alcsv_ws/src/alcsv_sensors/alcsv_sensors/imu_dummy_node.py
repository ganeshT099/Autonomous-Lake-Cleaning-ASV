import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class IMUDummyNode(Node):
    def __init__(self):
        super().__init__('imu_dummy')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)
        self.start_time = time.time()
        self.get_logger().info('IMU Dummy Node started')

    def publish_imu(self):
        msg = Imu()
        t = time.time() - self.start_time

        # Simulated yaw rotation
        yaw = math.sin(t) * 0.5

        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUDummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
