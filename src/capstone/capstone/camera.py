#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import Speed

import depthai as dai
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
        # TODO set up IMU in pipeline
        FPS = 30
        self.pipeline = dai.Pipeline()
        camRGB = self.pipeline.create(dai.node.ColorCamera)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)
        pointcloud = self.pipeline.create(dai.node.PointCloud)
        nn = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)

        nn_xout = self.pipeline.create(dai.node.XLinkOut)
        rgb_xout = self.pipeline.create(dai.node.XLinkOut)
        pcl_xout = self.pipeline.create(dai.node.XLinkOut)

        nnPath = str((Path(__file__).parent / Path('../models/.blob')).resolve().absolute())  # TODO change to current

        # Camera settings
        camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRGB.setInterleaved(False)
        camRGB.setPreviewSize(300, 300)  # TODO change to neural network dimensions
        camRGB.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRGB.setFps(FPS)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_left.setFps(FPS)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera("right")
        mono_right.setFps(FPS)

        # Stereo Settings
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(True)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(False)
        depth.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())
        depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # YOLO settings
        nn.setBlobPath(nnPath)
        nn.setConfidenceThreshold(0.5)
        nn.setNumClasses(80)
        nn.setCoordinateSize(4)
        nn.setIouThreshold(0.5)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        # Spatial settings
        nn.setBoundingBoxScaleFactor(0.5)
        nn.setDepthLowerThreshold(100)
        nn.setDepthUpperThreshold(5000)

        # Pipeline Linking
        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)
        depth.depth.link(pointcloud.inputDepth)
        pointcloud.outputPointCloud.link(pcl_xout)
        camRGB.preview.link(nn.input)
        depth.depth.link(nn.inputDepth)
        nn.passthrough.link(rgb_xout.input)
        nn.out.link(nn_xout.input)
        rgb_xout.setStreamName("out")
        nn_xout.setStreamName("nn")

        """
        Linking Diagram:
        
                                          ----.out------------->nn_xout
        camRGB--------------------->nn----|----.passthrough---->rgb_xout
                               |                            
        mono_left---|          |                             
                    --->depth--|-------->pointcloud------------>pcl_xout        
        mono_right--|                     
        """

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
