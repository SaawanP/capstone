#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from robot_interface.msg import Defect, Save
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from PyQt5 import QtGui, QtCore, QtWidgets


class Backend(Node):
    def __init__(self):
        super().__init__('backend')

        self.bridge = CvBridge()
        self.qt_image = QtGui.QImage()

        # Subscribers and publishers
        self.rgb_sub = self.create_subscription(Image, 'image_stream', self.stream_image, 10)
        self.defect_sub = self.create_subscription(Defect, 'defect_location', self.defect_callback, 10)

        self.save_pub = self.create_publisher(Save, 'save_report', 10)
        self.start_pub = self.create_publisher(Save, 'start_report', 10)

    def stream_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.qt_image = QtGui.QImage(self.image.data, self.image.shape[1], self.image.shape[0], QtGui.QImage.Format_RGB888)

    def defect_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg.image)
        location = [msg.location.x, msg.location.y, msg.location.z]

    def start_report(self):
        self.start_pub.publish(Header())

    def save_report(self):
        msg = Save()
        msg.save_location = "change later"
        msg.report_name = "change later"
        msg.point_cloud_save_type = "change later"
        self.save_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    backend = Backend()
    try:
        rclpy.spin(backend)
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
