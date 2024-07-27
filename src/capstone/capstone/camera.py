#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import Speed


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(
            Speed,
            'camera_speed',
            self.speed_callback,
            10
        )

        self.speed_msg: Speed = Speed()

    def speed_callback(self, msg):
        self.speed_msg = msg


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
