#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import Speed


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.MAX_SPEED = 10  # m/s TODO fix numbers
        self.MAX_ANGULAR_ROT = 10  # rad/s

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(
            Speed,
            'robot_speed',
            self.speed_callback,
            10
        )

        self.speed_msg: Speed = Speed()

    def speed_callback(self, msg):
        self.speed_msg = msg


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()