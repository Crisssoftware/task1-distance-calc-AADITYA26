import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
    def pose_callback(self, msg):
        x, y = msg.x, msg.y
        distance = math.sqrt(x**2 + y**2)
        self.get_logger().info(f'Distance from origin: {distance:.2f}')


def main():
    rclpy.init()
    node = DistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
