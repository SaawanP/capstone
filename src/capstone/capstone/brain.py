#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_interface.msg import Speed

import math


class Brain(Node):
    MANUAL = 'manual'
    AUTO = 'autonomous'

    def __init__(self):
        super().__init__('brain')
        self.JOY_RANGE = 30000

        # Subscribers and publishers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.robot_speed_pub = self.create_publisher(Speed, 'robot_speed', 10)
        self.camera_speed_pub = self.create_publisher(Speed, 'camera_speed', 10)

        self.state = self.AUTO

    def joy_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.buttons)
        self.get_logger().info('I heard: "%s"' % msg.axes)

        if msg.buttons[0] != 1:  # TODO fix index
            self.state = self.AUTO
            return

        self.state = self.MANUAL
        robot_speed = Speed()
        robot_speed.x = msg.axes[0] / self.JOY_RANGE  # TODO fix index
        robot_speed.y = msg.axes[0] / self.JOY_RANGE
        robot_speed.dist = math.sqrt(robot_speed.x ** 2 + robot_speed.y ** 2)
        self.robot_speed_pub.publish(robot_speed)

        camera_speed = Speed()
        camera_speed.x = msg.axes[0] / self.JOY_RANGE  # TODO fix index
        camera_speed.y = msg.axes[0] / self.JOY_RANGE
        camera_speed.dist = math.sqrt(camera_speed.x ** 2 + camera_speed.y ** 2)
        self.camera_speed_pub.publish(camera_speed)


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()
