#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

from robot_interface.msg import Speed
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header

import depthai as dai
import cv2
from pathlib import Path


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        self.bridge = CvBridge()

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(Speed, 'camera_speed', self.speed_callback, 10)

        # TODO test using https://wiki.ros.org/image_view
        self.rgb_pub = self.create_publisher(Image, 'image_stream', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        # TODO expand message for more information
        self.defect_pub = self.create_publisher(Point, 'defect_location', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.depth_map_pub = self.create_publisher(Image, 'depth_map', 10)

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
        depth_xout = self.pipeline.create(dai.node.XLinkOut)

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
        camRGB.preview.link(nn.input)
        depth.depth.link(nn.inputDepth)
        nn.passthroughDepth.link(pointcloud.inputDepth)
        nn.passthroughDepth.link(depth_xout.input)
        nn.passthrough.link(rgb_xout.input)
        nn.out.link(nn_xout.input)
        pointcloud.outputPointCloud.link(pcl_xout.input)
        rgb_xout.setStreamName("rgb")
        nn_xout.setStreamName("detections")
        pcl_xout.setStreamName("pcl")
        depth_xout.setStreamName("depth")

        """
        Linking Diagram:
        
                                          ----.out-------------------------->nn_xout
        camRGB--------------------->nn----|----.passthrough----------------->rgb_xout
                               |          |                  
        mono_left---|          |          |                  
                    --->depth--|          --.passthroughDepth-->pointcloud-->pcl_xout        
        mono_right--|                                         |------------->depth_xout
        """

    def speed_callback(self, msg):
        pass

    def broadcast_frame(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame)
        self.rgb_pub.publish(ros_image)

    def broadcast_defect(self, detection):
        point = Point()
        point.x = detection.spatialCoordinates.x
        point.y = detection.spatialCoordinates.y
        point.z = detection.spatialCoordinates.z
        self.defect_pub.publish(point)

    def broadcast_pointcloud_frame(self, pointcloud):
        header = Header()
        header.stamp = self.get_clock().now()
        points = pointcloud.getPoints()
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        ros_pc = pc2.create_cloud(header, fields, points)
        self.pointcloud_pub.publish(ros_pc)

    def broadcast_depth_map(self, depth_map):
        ros_image = self.bridge.cv2_to_imgmsg(depth_map)
        self.depth_map_pub.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    try:
        with dai.Device(camera.pipeline) as device:
            q_detections = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            q_RGB = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_pointcloud = device.getOutputQueue(name="pcl", maxSize=4, blocking=False)
            q_depthmap = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            while True:
                in_detections = q_detections.tryGet()
                in_rgb = q_RGB.tryGet()
                in_pointcloud = q_pointcloud.tryGet()
                in_depthmap = q_depthmap.tryGet()

                if in_rgb:
                    frame = in_rgb.getCvFrame()
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    camera.broadcast_frame(frame)

                if in_detections:
                    detections = in_detections.detections
                    for detection in detections:
                        camera.broadcast_defect(detection)

                if in_pointcloud:
                    points = in_pointcloud.getPoints()
                    camera.broadcast_pointcloud_frame(points)

                if in_depthmap:
                    depth_map = in_depthmap.getFrame()
                    camera.broadcast_depth_map(depth_map)

                rclpy.spin_once(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
