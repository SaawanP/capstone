#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import Speed

import depthai as dai
import open3d as o3d
from pathlib import Path


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

        # Camera Pipeline Setup
        FPS = 30
        self.pipeline = dai.Pipeline()
        camRGB = self.pipeline.create(dai.node.ColorCamera)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)
        pointcloud = self.pipeline.create(dai.node.PointCloud)
        sync = self.pipeline.create(dai.node.Sync)
        xOut = self.pipeline.create(dai.node.XLinkOut)
        nn = self.pipeline.create(dai.node.NeuralNetwork)
        cast = self.pipeline.create(dai.node.Cast)
        nn_xout = self.pipeline.create(dai.node.XLinkOut)

        camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRGB.setIspScale(1, 3)
        camRGB.setFps(FPS)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_left.setFps(FPS)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera("right")
        mono_right.setFps(FPS)

        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(True)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(True)
        depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Pipeline Linking
        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)
        depth.depth.link(pointcloud.inputDepth)
        camRGB.isp.link(sync.inputs["rgb"])
        pointcloud.outputPointCloud.link(sync.inputs["pcl"])
        sync.out.link(xOut.input)
        xOut.setStreamName("out")



    def speed_callback(self, msg):
        self.speed_msg = msg


def main(args=None):
    camera = Camera()
    try:
        rclpy.init(args=args)
        rclpy.spin(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
