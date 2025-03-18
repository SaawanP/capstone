import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import depthai as dai
import cv2

import RPi.GPIO as GPIO

class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        self.rgb_pub = self.create_publisher(Image, 'image_stream', 10)
        self.pipeline = dai.Pipeline()
        camRGB = self.pipeline.create(dai.node.ColorCamera)
        rgb_xout = self.pipeline.create(dai.node.XLinkOut)

        # Camera settings
        camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRGB.setInterleaved(False)
        camRGB.setPreviewSize(300, 300)  # TODO change to neural network dimensions
        camRGB.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRGB.setFps(30)

        camRGB.preview.link(rgb_xout.input)
        rgb_xout.setStreamName("rgb")


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    try:
        GPIO.setmode(GPIO.BCM)
        with dai.Device(camera.pipeline) as device:
            q_RGB = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            while True:
                in_rgb = q_RGB.get()

                if in_rgb:
                    frame = in_rgb.getCvFrame()
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    camera.broadcast_frame(frame)

                rclpy.spin_once(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()