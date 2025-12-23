import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher
        self.publisher = self.create_publisher(
            PoseStamped,
            '/drone/pose_stamped',
            10
        )

        # Time variable for motion
        self.t = 0.0

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.get_logger().info('Pose Publisher started')

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = 3.0 * math.cos(self.t)
        msg.pose.position.y = 3.0 * math.sin(self.t)
        msg.pose.position.z = 1.0

        self.publisher.publish(msg)
        self.t += 0.1


def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
