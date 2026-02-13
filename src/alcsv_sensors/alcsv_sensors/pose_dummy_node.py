import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DummyPose(Node):
    def __init__(self):
        super().__init__('dummy_pose_node')
        self.pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.x = 0.0

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.x
        msg.pose.position.y = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.x += 0.1

def main():
    rclpy.init()
    node = DummyPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
