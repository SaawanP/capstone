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

from motor import Servo
import RPi.GPIO as GPIO


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        self.bridge = CvBridge()
        self.start_time = None

        # Servo setup
        self.MAX_CAMERA_SPEED = 5  # degrees/sec
        self.servo_x = Servo(10)  # TODO change pin
        self.servo_y = Servo(11)
        self.last_servo_move = self.get_clock().now()

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
        FPS = 30
        self.pipeline = dai.Pipeline()
        camRGB = self.pipeline.create(dai.node.ColorCamera)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)
        pointcloud = self.pipeline.create(dai.node.PointCloud)
        nn = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        imu = self.pipeline.create(dai.node.IMU)

        nn_xout = self.pipeline.create(dai.node.XLinkOut)
        rgb_xout = self.pipeline.create(dai.node.XLinkOut)
        pcl_xout = self.pipeline.create(dai.node.XLinkOut)
        depth_xout = self.pipeline.create(dai.node.XLinkOut)
        imu_xout = self.pipeline.create(dai.node.XLinkOut)

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

        # IMU settings
        imu.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION, 400)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

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
        imu.out.link(imu_xout.input)

        rgb_xout.setStreamName("rgb")
        nn_xout.setStreamName("detections")
        pcl_xout.setStreamName("pcl")
        depth_xout.setStreamName("depth")
        imu_xout.setStreamName("imu")

        """
        Linking Diagram:
        
                                          ----.out-------------------------->nn_xout
        camRGB--------------------->nn----|----.passthrough----------------->rgb_xout
                               |          |                  
        mono_left---|          |          |                  
                    --->depth--|          --.passthroughDepth-->pointcloud-->pcl_xout        
        mono_right--|                                         |------------->depth_xout
        
        imu----------------------------------------------------------------->imu_xout
        """

    def speed_callback(self, msg):
        dt = self.get_clock().now() - self.last_servo_move

        if msg.dist == -1:  # Reset servos to neutral position
            self.servo_x.reset()
            self.servo_y.reset()

        x_speed = self.MAX_CAMERA_SPEED * msg.x
        y_speed = self.MAX_CAMERA_SPEED * msg.y

        dx = x_speed * dt
        dy = y_speed * dt

        x = self.servo_x.angle + dx
        y = self.servo_y.angle + dy

        self.servo_x.set_angle(x)
        self.servo_y.set_angle(y)

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

    def broadcast_imu_data(self, imu_packet):
        lin_accel_values = imu_packet.linearAcceleration
        rot_vect_values = imu_packet.rotationVector
        gyro_values = imu_packet.gyroscope

        lin_accel_time = lin_accel_values.getTimestamp()
        rot_vect_time = rot_vect_values.getTimestamp()
        gyro_time = gyro_values.getTimestamp()
        if self.start_time is None:
            self.start_time = min(lin_accel_time, rot_vect_time, gyro_time)
        lin_accel_time = (lin_accel_time - self.start_time).total_seconds()
        rot_vect_time = (rot_vect_time - self.start_time).total_seconds()
        gyro_time = (gyro_time - self.start_time).total_seconds()
        avg_time = (lin_accel_time + rot_vect_time + gyro_time) / 3

        header = Header()
        header.stamp = rclpy.time.Time(seconds=avg_time)

        imu = Imu()
        imu.header = header
        imu.orientation.x = rot_vect_values.i
        imu.orientation.y = rot_vect_values.j
        imu.orientation.z = rot_vect_values.k
        imu.orientation.w = rot_vect_values.real
        imu.angular_velocity.x = gyro_values.x
        imu.angular_velocity.y = gyro_values.y
        imu.angular_velocity.z = gyro_values.z
        imu.linear_acceleration.x = lin_accel_values.x
        imu.linear_acceleration.y = lin_accel_values.y
        imu.linear_acceleration.z = lin_accel_values.z

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    try:
        GPIO.setmode(GPIO.BOARD)
        with dai.Device(camera.pipeline) as device:
            q_detections = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            q_RGB = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_pointcloud = device.getOutputQueue(name="pcl", maxSize=4, blocking=False)
            q_depthmap = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            q_imu = device.getOutputQueue(name="imu", maxSize=4, blocking=False)

            while True:
                in_detections = q_detections.tryGet()
                in_rgb = q_RGB.tryGet()
                in_pointcloud = q_pointcloud.tryGet()
                in_depthmap = q_depthmap.tryGet()
                in_imu = q_imu.tryGet()

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

                if in_imu:
                    imu_packets = in_imu.packets
                    for imu_packet in imu_packets:
                        camera.broadcast_imu_data(imu_packet)

                rclpy.spin_once(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
