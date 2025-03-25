#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from robot_interface.msg import Defect, Save
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from PySide6.QtCore import QObject, Signal, Slot
import numpy as np


class CameraSignals(QObject):
    image_ready = Signal(np.ndarray)
    defect_detect = Signal(np.ndarray)


class Backend(Node):
    def __init__(self):
        super().__init__('backend')

        self.bridge = CvBridge()

        # Subscribers and publishers
        self.rgb_sub = self.create_subscription(Image, 'image_stream', self.stream_image, 10)
        self.defect_sub = self.create_subscription(Defect, 'defect_location', self.defect_callback, 10)

        self.save_pub = self.create_publisher(Save, 'save_report', 10)
        self.start_pub = self.create_publisher(Save, 'start_report', 10)

        self.signals = CameraSignals()

    def stream_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        self.signals.image_ready.emit(cv_image)

    def defect_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg.image)
        location = [msg.location.x, msg.location.y, msg.location.z]
        text = "Location: " + str(location)
        position = (50, 50)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 255, 255)
        thickness = 2
        line_type = cv2.LINE_AA
        cv2.putText(cv_image, text, position, font, font_scale, color, thickness, line_type)

        self.signals.defect_detect.emit(cv_image)

    @Slot(dict)
    def send_report_details(self, message):
        self.get_logger().info("got start/stop")
        msg = Save()
        msg.save_location = message["save_location"]
        msg.report_name = message["report_name"]
        msg.point_cloud_save_type = message["point_cloud_save_type"]
        if message["status"]:
            self.start_pub.publish(msg)
        else:
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
