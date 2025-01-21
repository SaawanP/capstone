#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from robot_interface.msg import Speed, Defect
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header

import depthai as dai
import cv2
from pathlib import Path

from capstone.transformation_matrix.py import Transformation
from capstone.motor import Servo
import RPi.GPIO as GPIO


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        # Constants
        self.declare_parameter('max_camera_speed', 0)
        self.declare_parameter('fps', 0)
        self.declare_parameter('test_env', False)

        self.MAX_CAMERA_SPEED = self.get_parameter('max_camera_speed').get_parameter_value().integer_value
        self.TEST_ENV = self.get_parameter('test_env').get_parameter_value().bool_value
        FPS = self.get_parameter('fps').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.start_time = None
        self.seen_defects = []

        # Servo setup
        self.MAX_CAMERA_SPEED = 5  # degrees/sec
        self.servo_x = Servo(10)  # TODO change pin
        self.servo_y = Servo(11)
        self.last_servo_move = self.get_clock().now()
        self.camera_position = [0, 0]
        self.transformation = Transformation()

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(Speed, 'camera_speed', self.speed_callback, 10)
        self.position_sub = self.create_subscription(Vector3, 'robot_position', self.position_callback,10)

        # TODO test using https://wiki.ros.org/image_view
        self.rgb_pub = self.create_publisher(Image, 'image_stream', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.defect_pub = self.create_publisher(Defect, 'defect_location', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.depth_map_pub = self.create_publisher(Image, 'depth_map', 10)

        # Camera Pipeline Setup
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

        self.camera_position = [x, y]
        self.servo_x.set_angle(x)
        self.servo_y.set_angle(y)

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

    def broadcast_pointcloud_frame(self, pointcloud):
        header = Header()
        header.stamp = self.get_clock().now()
        points = pointcloud.getPoints()
        points = self.transformation.apply_multiple(points)
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
        lin_accel = imu_packet.linearAcceleration
        rot_vect = imu_packet.rotationVector
        gyro = imu_packet.gyroscope

        lin_accel_time = lin_accel.getTimestamp()
        rot_vect_time = rot_vect.getTimestamp()
        gyro_time = gyro.getTimestamp()
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
        imu.orientation.x = rot_vect.i
        imu.orientation.y = rot_vect.j
        imu.orientation.z = rot_vect.k
        imu.orientation.w = rot_vect.real
        self.transformation.rotation = [rot_vect.i, rot_vect.j, rot_vect.k, rot_vect.real]
        imu.angular_velocity.x = gyro.x
        imu.angular_velocity.y = gyro.y
        imu.angular_velocity.z = gyro.z
        imu.linear_acceleration.x = lin_accel.x
        imu.linear_acceleration.y = lin_accel.y
        imu.linear_acceleration.z = lin_accel.z

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    try:
        GPIO.setmode(GPIO.BCM)
        with dai.Device(camera.pipeline) as device:
            q_detections = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            q_RGB = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_pointcloud = device.getOutputQueue(name="pcl", maxSize=4, blocking=False)
            q_depthmap = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            q_imu = device.getOutputQueue(name="imu", maxSize=4, blocking=False)

            while True:
                in_detections = q_detections.get()
                in_rgb = q_RGB.get()
                in_pointcloud = q_pointcloud.get()
                in_depthmap = q_depthmap.get()
                in_imu = q_imu.get()

                frame = in_rgb.getCvFrame()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                detections = in_detections.detections
                for detection in detections:
                    frame = camera.broadcast_defect(detection, frame)

                camera.broadcast_frame(frame)

                points = in_pointcloud.getPoints()
                camera.broadcast_pointcloud_frame(points)

                depth_map = in_depthmap.getFrame()
                camera.broadcast_depth_map(depth_map)

                imu_packets = in_imu.packets
                for imu_packet in imu_packets:
                    camera.broadcast_imu_data(imu_packet)

                rclpy.spin_once(camera)
    finally:
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
