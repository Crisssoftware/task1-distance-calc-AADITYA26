import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.create_subscriber()
        self.create_publisher()
        self.x = 0.0
        self.y = 0.0
    
    def create_subscriber(self):
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
    
    def create_publisher(self):
        self.publisher_ = self.create_publisher(Float32, '/turtle1/distance_from_origin', 10)
    
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.publish_distance()
    
    def publish_distance(self):
        distance = math.sqrt(self.x**2 + self.y**2)
        distance_msg = Float32()
        distance_msg.data = distance
        self.publisher_.publish(distance_msg)
        self.get_logger().info(f'Pose -> x: {self.x}, y: {self.y}, Distance: {distance:.2f}')

def main():
    rclpy.init()
    node = DistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
