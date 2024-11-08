#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Joy
from robot_interface.msg import Speed, Defect
from sensor_msgs.msg import Imu, Image, PointCloud2
from geometry_msgs.msg import Vector3

import math
import numpy as np
from enum import Enum
import open3d as o3d
import cv2
import yaml


class State(Enum):
    MANUAL = "Manual"
    AUTONOMOUS = "Autonomous"


class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.JOY_RANGE = 30000  # TODO fix numbers
        self.DEADBAND = 10
        self.MAX_SPEED = 70  # m/min
        self.OPERATIONAL_SPEED = 20  # m/min

        # Subscribers and publishers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, 'point_cloud', self.pointcloud_callback, 10)
        self.defect_sub = self.create_subscription(Defect, 'defect_location', self.defect_location_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'depth_map', self.autonomous_callback, 10)
        self.motor_sub = self.create_subscription(Vector3, 'motor_controller_position', self.motor_position_callback,10)

        self.robot_speed_pub = self.create_publisher(Speed, 'robot_speed', 10)
        self.camera_speed_pub = self.create_publisher(Speed, 'camera_speed', 10)
        self.position_pub = self.create_publisher(Vector3, 'imu_position', 10)

        self.state = self.AUTO
        self.bridge = CvBridge()
        self.point_cloud = []
        self.defect_locations: list[Defect] = []
        self.curr_position: Vector3 = Vector3()
        self.last_velocity: Vector3 = Vector3()
        self.last_imu_msg = Imu()
        self.last_imu_msg.header.stamp = self.get_clock().now()

    def joy_callback(self, msg: Joy):
        if msg.buttons[0] != 1:  # TODO fix index
            self.state = State.AUTONOMOUS
            return

        self.state = State.MANUAL
        # All values are -1 to 1
        robot_speed = Speed()  # TODO fix index
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            robot_speed.x = msg.axes[0] / self.JOY_RANGE
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            robot_speed.y = msg.axes[0] / self.JOY_RANGE
        robot_speed.dist = math.sqrt(robot_speed.x ** 2 + robot_speed.y ** 2)
        self.robot_speed_pub.publish(robot_speed)

        camera_speed = Speed()  # TODO fix index
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            camera_speed.x = msg.axes[0] / self.JOY_RANGE
        if msg.axes[0] > self.DEADBAND or msg.axes < -self.DEADBAND:
            camera_speed.y = msg.axes[0] / self.JOY_RANGE
        camera_speed.dist = math.sqrt(camera_speed.x ** 2 + camera_speed.y ** 2)
        self.camera_speed_pub.publish(camera_speed)

    def imu_callback(self, msg: Imu):
        t1 = self.last_imu_msg.header.stamp.nanoseconds * 1e-9  # s
        t2 = msg.header.stamp.nanoseconds * 1e-9  # s
        dt = t2 - t1

        dax = msg.linear_acceleration.x - self.last_imu_msg.linear_acceleration.x
        day = msg.linear_acceleration.y - self.last_imu_msg.linear_acceleration.y
        daz = msg.linear_acceleration.z - self.last_imu_msg.linear_acceleration.z

        dvx = dt * self.last_imu_msg.linear_acceleration.x + dt * dax / 2
        dvy = dt * self.last_imu_msg.linear_acceleration.y + dt * day / 2
        dvz = dt * self.last_imu_msg.linear_acceleration.z + dt * daz / 2

        dx = dt * self.last_velocity.x + dt * dvx / 2
        dy = dt * self.last_velocity.y + dt * dvy / 2
        dz = dt * self.last_velocity.z + dt * dvz / 2

        self.curr_position.x += dx
        self.curr_position.y += dy
        self.curr_position.z += dz
        self.position_pub.publish(self.curr_position)

        self.last_velocity.x += dvx
        self.last_velocity.y += dvy
        self.last_velocity.z += dvz

    def pointcloud_callback(self, msg: PointCloud2):
        points = list(pc2.read_points(msg, skip_nans=True))
        for point in points:
            point = [point[0] + self.curr_position.x,
                     point[1] + self.curr_position.y,
                     point[2] + self.curr_position.z]
            self.point_cloud.append(point)

    def defect_location_callback(self, msg):
        msg.location.x += self.curr_position.x
        msg.location.y += self.curr_position.y
        msg.location.z += self.curr_position.z
        self.defect_locations.append(msg)

    def autonomous_callback(self, msg: Image):
        if self.state == State.MANUAL:
            return

        # Calculate average deviation from center
        depth = np.array(msg.data).reshape(msg.height, msg.width)
        left_depth, right_depth = np.hsplit(depth, 2)
        left_avg_dist = sum(sum(left_depth))
        right_avg_dist = sum(sum(right_depth))
        diff = right_avg_dist - left_avg_dist

        # Turn robot proportional to deviation, largest turn_angle=45 degrees
        speed = Speed()
        turn_angle = math.atan(diff) / 2
        speed.dist = self.OPERATIONAL_SPEED / self.MAX_SPEED
        speed.y = math.sin(turn_angle)
        speed.x = math.cos(turn_angle)
        self.robot_speed_pub.publish(speed)

        camera_speed = Speed()
        camera_speed.dist = -1  # Reset to neutral position
        self.camera_speed_pub.publish(camera_speed)

    def motor_position_callback(self, msg):
        dx = abs(msg.x - self.curr_position.x)
        dy = abs(msg.y - self.curr_position.y)
        dz = abs(msg.z - self.curr_position.z)
        dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def save_report_to_file(self):
        # Save pointcloud data
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        name = "report.ply"  # Decide whether to use ply or stl
        o3d.io.write_point_cloud(name, pcd)

        # Save defect images
        defect = {}
        for i, d in enumerate(self.defect_locations):
            location = (d.location.x, d.location.y, d.location.z)
            image_name = f"crack_{i}.png"
            cv_image = self.bridge.imgmsg_to_cv2(d.image)
            cv2.imwrite(image_name, cv_image)
            defect[location] = image_name

        # Save yaml
        yaml_name = "defects.yaml"
        with open(yaml_name, 'w') as f:
            yaml.dump(defect, f)


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    try:
        rclpy.spin(brain)
    finally:
        brain.destroy_node()
        rclpy.shutdown()
