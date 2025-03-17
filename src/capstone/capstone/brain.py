#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from sensor_msgs.msg import Joy
from robot_interface.msg import RobotSpeed, CameraSpeed, Defect, Save
from sensor_msgs.msg import Imu, Image, PointCloud2
from geometry_msgs.msg import Vector3

import math
import open3d as o3d
import cv2
import yaml


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # Constants
        self.declare_parameter('joy_range', 0)
        self.declare_parameter('max_speed', 0)

        self.JOY_RANGE = self.get_parameter('joy_range').get_parameter_value().integer_value
        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().integer_value

        # Subscribers and publishers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, 'point_cloud', self.pointcloud_callback, 10)
        self.defect_sub = self.create_subscription(Defect, 'defect_location', self.defect_location_callback, 10)
        self.motor_sub = self.create_subscription(Vector3, 'controller_pose_position', self.motor_position_callback, 10)
        self.save_sub = self.create_subscription(Save, 'save_report', self.save_report_to_file, 10)

        self.robot_speed_pub = self.create_publisher(RobotSpeed, 'robot_speed', 10)
        self.camera_speed_pub = self.create_publisher(CameraSpeed, 'camera_speed', 10)

        self.bridge = CvBridge()
        self.point_cloud = []
        self.defect_locations: list[Defect] = []
        self.curr_position: Vector3 = Vector3()
        self.last_velocity: Vector3 = Vector3()
        self.last_imu_msg = Imu()
        self.last_imu_msg.header.stamp = self.get_clock().now().to_msg()

    def joy_callback(self, msg: Joy):
        # All values are -1 to 1
        robot_speed = RobotSpeed()  # TODO fix index
        vx = 0
        vy = 0
        vx = msg.axes[2] / self.JOY_RANGE
        robot_speed.direction = math.copysign(1, vx)
        vy = msg.axes[3] / self.JOY_RANGE
        robot_speed.turning_radius = msg.axes[0] / self.JOY_RANGE
        robot_speed.speed = math.sqrt(vx ** 2 + vy ** 2)
        self.robot_speed_pub.publish(robot_speed)

        camera_speed = CameraSpeed()  # TODO fix index
        camera_speed.reset = False
        camera_speed.wx = msg.axes[0] / self.JOY_RANGE
        camera_speed.wy = msg.axes[1] / self.JOY_RANGE
        self.camera_speed_pub.publish(camera_speed)

    def pointcloud_callback(self, msg: PointCloud2):
        points = list(pc2.read_points(msg, skip_nans=True))
        self.point_cloud += points

    def defect_location_callback(self, msg):
        msg.location.x += self.curr_position.x
        msg.location.y += self.curr_position.y
        msg.location.z += self.curr_position.z
        self.defect_locations.append(msg)

    def save_report_to_file(self, msg):
        folder = msg.save_location + "/" + msg.report_name

        # Save pointcloud data
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        pcd_location = folder + "/point_cloud." + msg.point_cloud_save_type
        o3d.io.write_point_cloud(pcd_location, pcd)

        # Save defect images
        defects = {}
        for i, d in enumerate(self.defect_locations):
            location = (d.location.x, d.location.y, d.location.z)
            image_location = folder + f"/crack_{i}.png"
            cv_image = self.bridge.imgmsg_to_cv2(d.image)
            cv2.imwrite(image_location, cv_image)
            defects[location] = image_location

        # Save yaml
        yaml_location = folder + "/defects.yaml"
        with open(yaml_location, 'w') as f:
            yaml.dump(defects, f)


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    try:
        rclpy.spin(brain)
    finally:
        brain.destroy_node()
        rclpy.shutdown()
