import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.x = 0.0
        self.y = 0.0
    
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.get_logger().info(f'Received Pose -> x: {self.x}, y: {self.y}')

class DistancePublisher(Node):
    def __init__(self, subscriber_node):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Float32, '/turtle1/distance_from_origin', 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscriber_node = subscriber_node
    
    def timer_callback(self):
        distance = math.sqrt(self.subscriber_node.x**2 + self.subscriber_node.y**2)
        distance_msg = Float32()
        distance_msg.data = distance
        self.publisher_.publish(distance_msg)
        self.get_logger().info(f'Publishing Distance: {distance:.2f}')

def main():
    rclpy.init()
    subscriber_node = DistanceSubscriber()
    publisher_node = DistancePublisher(subscriber_node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(subscriber_node)
    executor.add_node(publisher_node)
    executor.spin()
    subscriber_node.destroy_node()
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
