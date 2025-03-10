#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/robot_joy',
            self.joy_callback,
            10)
        self.get_logger().info('Joy Subscriber Node Started')

    def joy_callback(self, msg):
        # Print the joy data received
        self.get_logger().info(f'Received: Buttons: {msg.buttons}, Axes: {msg.axes}')
        # Process the controller input here
        # Example: if button 0 is pressed
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.get_logger().info('Button A pressed!')

def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
