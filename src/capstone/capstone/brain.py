#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from sensor_msgs.msg import Joy
from robot_interface.msg import RobotSpeed, CameraSpeed, Defect, Save
from sensor_msgs.msg import Imu, Image, PointCloud2
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

import math
# import open3d as o3d
# import cv2
import yaml
# import h5py
import struct
import time
import numpy as np


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # Constants
        self.declare_parameter('max_speed', 1.412)
        self.declare_parameter('max_track_angle', 40)

        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.MAX_TRACK_ANGLE = self.get_parameter('max_track_angle').get_parameter_value().integer_value

        # Subscribers and publishers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, 'point_cloud', self.pointcloud_callback, 10)
        self.defect_sub = self.create_subscription(Defect, 'defect_location', self.defect_location_callback, 10)
        self.save_sub = self.create_subscription(Save, 'save_report', self.save_report_to_file, 10)
        self.start_sub = self.create_subscription(Save, 'start_report', self.start_running, 10)

        self.robot_speed_pub = self.create_publisher(RobotSpeed, 'robot_speed', 10)
        self.camera_speed_pub = self.create_publisher(CameraSpeed, 'camera_speed', 10)

        # self.bridge = CvBridge()
        self.defect_locations: list[Defect] = []
        self.curr_position: Vector3 = Vector3()
        self.last_velocity: Vector3 = Vector3()
        self.last_imu_msg = Imu()
        self.last_imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_timestep = 0
        self.running = False
        self.folder = ""
        self.h5_file = None
        self.track_angle = 0.0
        self.light_level = 0

    def start_running(self, msg):
        self.running = True
        # self.folder = msg.save_location + "/" + msg.report_name
        # h5_filename = self.folder + 'data_points.h5'
        # self.h5_file = h5py.File(h5_filename, 'a')

    def joy_callback(self, msg: Joy):
        if not self.running:
            return

        # All values are -1 to 1
        robot_speed = RobotSpeed()
        robot_speed.header.stamp = msg.header.stamp
        vx = msg.axes[3]
        vy = msg.axes[2]
        robot_speed.direction = math.copysign(1, vx)
        robot_speed.turning_radius = vx
        robot_speed.speed = min(math.sqrt(vx ** 2 + vy ** 2), 1.0)

        if msg.buttons[0] == 1:
            self.light_level += 5
            self.light_level = min(self.light_level, 100)
        if msg.buttons[3] == 1:
            self.light_level -= 5
            self.light_level = max(self.light_level, 0)
        robot_speed.lights = self.light_level

        if msg.buttons[1] == 1:
            self.track_angle += 5
            self.track_angle = min(self.track_angle, self.MAX_TRACK_ANGLE)
        if msg.buttons[2] == 1:
            self.track_angle -= 5
            self.track_angle = max(self.track_angle, 0)
        robot_speed.track_angle = float(self.track_angle)

        self.robot_speed_pub.publish(robot_speed)

        camera_speed = CameraSpeed()
        camera_speed.wx = msg.axes[0]
        camera_speed.wy = msg.axes[1]
        self.camera_speed_pub.publish(camera_speed)

    def pointcloud_callback(self, msg: PointCloud2):
        if not self.running:
            return

        # colored_points = np.array(pc2.read_points(msg, skip_nans=True))
        # points = np.zeros(shape=(len(colored_points), 3))
        # colors = np.zeros(shape=(len(colored_points), 3))
        # for i, colored_point in enumerate(colored_points):
        #     points[i] = colored_point[0: 3]
        #     b, g, r, a = struct.unpack('BBBB', colored_point[3].to_bytes(4, byteorder='little'))
        #     colors[i] = [r, g, b]

        # try:
        #     frame_id = f"frame_{self.current_timestep}"
        #     grp = self.h5_file.create_group(frame_id)
        #     grp.create_dataset("points", data=points)
        #     grp.create_dataset("colors", data=colors)
        #     grp.attrs["timestamp"] = time.time()
        #     self.h5_file.flush()
        # except Exception as e:
        #     self.get_logger().error(f"Error saving to H5 file: {e}")

    def defect_location_callback(self, msg):
        if not self.running:
            return

        msg.location.x += self.curr_position.x
        msg.location.y += self.curr_position.y
        msg.location.z += self.curr_position.z
        self.defect_locations.append(msg)

    def save_report_to_file(self, msg):
        return
        # # Save pointcloud data
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        # pcd_location = self.folder + "/point_cloud." + msg.point_cloud_save_type
        # o3d.io.write_point_cloud(pcd_location, pcd)

        # # Save defect images
        # defects = {}
        # for i, d in enumerate(self.defect_locations):
        #     location = (d.location.x, d.location.y, d.location.z)
        #     image_location = self.folder + f"/crack_{i}.png"
        #     cv_image = self.bridge.imgmsg_to_cv2(d.image)
        #     cv2.imwrite(image_location, cv_image)
        #     defects[location] = image_location

        # # Save yaml
        # yaml_location = self.folder + "/defects.yaml"
        # with open(yaml_location, 'w') as f:
        #     yaml.dump(defects, f)


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    try:
        while not brain.running:
            brain.get_logger().info("Waiting to start brain", throttle_duration_sec=1)
        rclpy.spin(brain)
    finally:
        brain.destroy_node()
        rclpy.shutdown()
