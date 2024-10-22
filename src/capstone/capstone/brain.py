#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_interface.msg import Speed

import math
from enum import Enum


class State(Enum):
    MANUAL = "Manual"
    AUTONOMOUS = "Autonomous"


class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.JOY_RANGE = 30000  # TODO fix numbers
        self.DEADBAND = 10

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
        self.curr_position = [0, 0]  # TODO update using cameras pose

    def joy_callback(self, msg):
        if msg.buttons[0] != 1:  # TODO fix index
            self.state = self.AUTO
            return

        self.state = self.MANUAL
        robot_speed = Speed()  # TODO fix index
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            robot_speed.x = msg.axes[0] / self.JOY_RANGE
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            robot_speed.y = msg.axes[0] / self.JOY_RANGE
        robot_speed.dist = math.sqrt(robot_speed.x ** 2 + robot_speed.y ** 2)
        self.robot_speed_pub.publish(robot_speed)

        camera_speed = Speed()  # TODO fix index
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            camera_speed.x = msg.axes[0] / self.JOY_RANGE
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            camera_speed.y = msg.axes[0] / self.JOY_RANGE
        camera_speed.dist = math.sqrt(camera_speed.x ** 2 + camera_speed.y ** 2)
        self.camera_speed_pub.publish(camera_speed)


def main(args=None):
    brain = Brain()
    try:
        rclpy.init(args=args)
        rclpy.spin(brain)
    finally:
        brain.destroy_node()
        rclpy.shutdown()
