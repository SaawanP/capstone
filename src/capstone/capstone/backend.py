#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class Backend(Node):
    def __init__(self):
        super().__init__('backend')

        # Subscribers and publishers


def main(args=None):
    backend = Backend()
    try:
        rclpy.init(args=args)
        rclpy.spin(backend)
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
