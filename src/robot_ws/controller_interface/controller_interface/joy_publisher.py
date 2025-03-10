#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        # Create a subscriber to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        # Create a publisher to forward the joy data
        self.publisher = self.create_publisher(
            Joy,
            '/robot_joy',
            10)
        self.get_logger().info('Joy Publisher Node Started')

    def joy_callback(self, msg):
        # Forward the joy message to the robot
        self.publisher.publish(msg)
        self.get_logger().info(f'Buttons: {msg.buttons}, Axes: {msg.axes}')

def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    rclpy.spin(joy_publisher)
    joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
