#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from robot_interface.msg import CameraSpeed, Defect, Save
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

import depthai as dai
import cv2
from pathlib import Path
import math
import numpy as np
import struct

from capstone.transformation_matrix import Transformation
from capstone.motor import Servo
import RPi.GPIO as GPIO


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        # Constants
        self.declare_parameter('max_camera_speed', 0.5)
        self.declare_parameter('max_camera_range', 20)
        self.declare_parameter('starting_camera_angle', 90)
        self.declare_parameter('fps', 30)

        self.MAX_CAMERA_SPEED = self.get_parameter('max_camera_speed').get_parameter_value().double_value
        self.MAX_CAMERA_RANGE = self.get_parameter('max_camera_range').get_parameter_value().integer_value
        self.START_CAMERA_ANGLE = self.get_parameter('starting_camera_angle').get_parameter_value().integer_value
        FPS = self.get_parameter('fps').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.start_time = None
        self.seen_defects = []

        # Servo setup
        self.servo_x = Servo(17, self.START_CAMERA_ANGLE)
        self.servo_y = Servo(27, self.START_CAMERA_ANGLE)
        self.last_servo_move = self.get_clock().now()
        self.camera_position = [0, 0]
        self.transformation = Transformation()
        self.running = False

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(CameraSpeed, 'camera_speed', self.speed_callback, 10)
        self.position_sub = self.create_subscription(Vector3, 'position', self.position_callback, 10)
        self.start_sub = self.create_subscription(Save, 'start_report', self.start_runnning, 10)
        self.save_sub = self.create_subscription(Save, 'save_report', self.complete_run, 10)

        # TODO test using https://wiki.ros.org/image_view
        self.rgb_pub = self.create_publisher(Image, 'image_stream', 10)
        self.defect_pub = self.create_publisher(Defect, 'defect_location', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # Camera Pipeline Setup
        self.pipeline = dai.Pipeline()

        """
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
        camRGB.preview.link(nn.input)
        depth.depth.link(nn.inputDepth)
        nn.passthroughDepth.link(pointcloud.inputDepth)
        nn.passthrough.link(rgb_xout.input)
        nn.out.link(nn_xout.input)
        pointcloud.outputPointCloud.link(pcl_xout.input)

        rgb_xout.setStreamName("rgb")
        nn_xout.setStreamName("detections")
        pcl_xout.setStreamName("pcl")
        """
        """
        Linking Diagram:

                                          ----.out-------------------------->nn_xout
        camRGB--------------------->nn----|----.passthrough----------------->rgb_xout
                               |          |
        mono_left---|          |          |
                    --->depth--|          --.passthroughDepth-->pointcloud-->pcl_xout
        mono_right--|

        """

        # Dummy Pipeline
        # Define the color camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # Create output stream
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)  # Fixed typo: XaLinkOut -> XLinkOut
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

    def start_runnning(self, msg):
        self.running = True

    def speed_callback(self, msg):
        now = self.get_clock().now()
        dt = now - self.last_servo_move
        self.last_servo_move = now

        # Reset servos to neutral position
        if msg.reset == True:
            self.camera_position = [0, 0]
            self.servo_x.reset()
            self.servo_y.reset()
            return

        x_speed = self.MAX_CAMERA_SPEED * msg.wx
        y_speed = self.MAX_CAMERA_SPEED * msg.wy

        dx = x_speed * dt
        dy = y_speed * dt

        x = self.servo_x.angle + dx
        y = self.servo_y.angle + dy

        if abs(x) > self.MAX_CAMERA_RANGE:
            x = math.copysign(self.MAX_CAMERA_RANGE, x)
        if abs(y) > self.MAX_CAMERA_RANGE:
            y = math.copysign(self.MAX_CAMERA_RANGE, y)

        self.camera_position = [x, y]
        self.servo_x.set_angle(self.START_CAMERA_ANGLE + x)
        self.servo_y.set_angle(self.START_CAMERA_ANGLE + y)

    def position_callback(self, msg):
        translation = [msg.x, msg.y, msg.z]
        self.transformation.translation = translation

    def broadcast_frame(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame)
        self.rgb_pub.publish(ros_image)

    def broadcast_defect(self, detection, frame):
        def get_closest_seen_defect(pt):
            def manhattan_distance(p1, p2):
                x = abs(p1[0] - p2[0])
                y = abs(p1[1] - p2[1])
                z = abs(p1[2] - p2[2])
                return x + y + z

            if len(self.seen_defects) == 0:
                return -1, -1
            i = min(range(len(self.seen_defects)), key=lambda a: manhattan_distance(self.seen_defects[a], pt))
            distance = manhattan_distance(self.seen_defects[i], pt)
            return i, distance

        def is_near_frame_edge():
            # TODO Change thresholds
            # top edge
            if detection.ymin < 0.05 and detection.ymax < 0.1:
                return True
            # bottom edge
            if detection.ymax > 0.95 and detection.ymin > 0.9:
                return True
            # left edge
            if detection.xmin < 0.05 and detection.xmax < 0.1:
                return True
            # right edge
            if detection.xmax < 0.95 and detection.xmin > 0.9:
                return True

            return False

        point = [detection.spatialCoordinates.x, detection.spatialCoordinates.y, detection.spatialCoordinates.z]
        ind, dist = get_closest_seen_defect(point)
        height = frame.shape[0]
        width = frame.shape[1]
        # Denormalize bounding box
        x1 = int(detection.xmin * width)
        x2 = int(detection.xmax * width)
        y1 = int(detection.ymin * height)
        y2 = int(detection.ymax * height)

        if dist < 0.5 and ind != -1:  # TODO fix threshold value
            self.seen_defects[ind] = point
            if is_near_frame_edge():
                del self.seen_defects[ind]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255))
            return frame

        defect = Defect()
        point = [detection.spatialCoordinates.x, detection.spatialCoordinates.y, detection.spatialCoordinates.z]
        point = self.transformation.apply(point)
        defect.location.x = point[0]
        defect.location.y = point[1]
        defect.location.z = point[2]

        roi = frame[y1:y2, x1:x2]
        defect.image = self.bridge.cv2_to_imgmsg(roi)
        self.defect_pub.publish(defect)
        self.seen_defects.append(point)

        # Add bounding box to rbg image
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255))
        return frame

    def broadcast_pointcloud_frame(self, pointcloud, frame):
        points = pointcloud.getPoints().astype(np.float64)

        # Remove zero points
        non_zero_mask = ~np.all(points == 0, axis=1)
        points = points[non_zero_mask]

        # Filter based on distances
        distances = points[:, 2]
        counts, bin_edges = np.histogram(distances, bins=100, density=False)
        # changes = [counts[i] - counts[i + 1] for i in range(len(counts) - 1)]
        # max_dist = bin_edges[np.argmax(changes[1:]) + 2]
        max_dist = bin_edges[2]
        # Apply max_dist filtering
        valid_points_mask = points[:, 2] <= max_dist
        points = points[valid_points_mask]

        # Filter based on radius
        q1_points = points[(points[:, 0] > 0) & (points[:, 1] > 0)]
        q1_centre = np.array((np.average(q1_points[:, 0]), np.average(q1_points[:, 1])))
        q2_points = points[(points[:, 0] < 0) & (points[:, 1] > 0)]
        q2_centre = np.array((np.average(q1_points[:, 0]), np.average(q1_points[:, 1])))
        q3_points = points[(points[:, 0] < 0) & (points[:, 1] < 0)]
        q3_centre = np.array((np.average(q3_points[:, 0]), np.average(q3_points[:, 1])))
        q4_points = points[(points[:, 0] > 0) & (points[:, 1] < 0)]
        q4_centre = np.array((np.average(q4_points[:, 0]), np.average(q4_points[:, 1])))
        centre = (q1_centre + q2_centre + q3_centre + q4_centre) / 4
        radial_distance = np.sqrt((points[:, 0] - centre[0]) ** 2 + (points[:, 1] - centre[1]) ** 2)
        rad_count, rad_bin_edges = np.histogram(radial_distance, bins=100, density=False)

        diff_counts = np.diff(rad_count)
        threshold = np.mean(diff_counts) + np.std(diff_counts)
        big_jump_indices = np.where(diff_counts > threshold)[0]  # Get indices where jump is large

        if len(big_jump_indices) >= 2:
            first_jump_idx, second_jump_idx = big_jump_indices[:2]
            first_jump_bin = rad_bin_edges[first_jump_idx - 1]
            second_jump_bin = rad_bin_edges[second_jump_idx + 3]

        radius_mask = radial_distance <= second_jump_bin
        points = points[radius_mask]

        # Ensure colors are filtered accordingly
        colors = (frame.reshape(-1, 3) / 255.0).astype(np.float64)
        colors = colors[non_zero_mask]
        colors = colors[valid_points_mask]
        colors = colors[radius_mask]

        combined_colors = []
        for color in colors:
            r, g, b = color
            a = 255
            rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            combined_colors.append(rgba)
        combined_colors = np.transpose([combined_colors])
        pc = np.append(points, combined_colors, axis=1)

        # Create ROS message
        header = Header()
        header.stamp = self.get_clock().now()
        header.frame_id = 'map'
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 16, PointField.UINT32, 1),
        ]
        ros_pc = pc2.create_cloud(header, fields, pc)
        self.pointcloud_pub.publish(ros_pc)

    def complete_run(self, msg):
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    try:
        GPIO.setmode(GPIO.BCM)
        camera = Camera()
        with dai.Device(camera.pipeline) as device:
            camera.get_logger().info("Device connected")
            q_RGB = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            # q_detections = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            # q_pointcloud = device.getOutputQueue(name="pcl", maxSize=4, blocking=False)

            # while camera.running:
            #     in_rgb = q_RGB.get()
            #     # in_detections = q_detections.get()
            #     # in_pointcloud = q_pointcloud.get()

            #     frame = in_rgb.getCvFrame()
            #     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            #     # detections = in_detections.detections
            #     # for detection in detections:
            #     #     frame = camera.broadcast_defect(detection, frame)

            #     camera.broadcast_frame(frame)

            #     # points = in_pointcloud.getPoints()
            #     # camera.broadcast_pointcloud_frame(points, frame)

            #     rclpy.spin_once(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
