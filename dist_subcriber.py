import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.x = 0.0
        self.y = 0.0
        self.distance = 0.0
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/turtle1/distance_from_origin', 10)
        self.timer = self.create_timer(0.5, self.publish_distance)
    
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.distance = math.sqrt(self.x**2 + self.y**2)
    
    def publish_distance(self):
        distance_msg = Float32()
        distance_msg.data = self.distance
        self.publisher_.publish(distance_msg)
        self.get_logger().info(f'Distance: {self.distance:.2f}')

def main():
    rclpy.init()
    node = DistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
