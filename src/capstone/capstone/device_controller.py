#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import RobotSpeed, CameraSpeed
from geometry_msgs.msg import Vector3

import math
from capstone.devices import Motor, PID, Servo, LED
import RPi.GPIO as GPIO


class DeviceController(Node):
    def __init__(self):
        super().__init__('device_controller')

        # Constants
        self.declare_parameter('max_camera_speed', 0.5)
        self.declare_parameter('max_camera_range', 20)
        self.declare_parameter('starting_camera_angle', 90)
        self.declare_parameter('max_rpm', 10)
        self.declare_parameter('radius', 0.02247)
        self.declare_parameter('length', 0.149)
        self.declare_parameter('circ', 0.1412)
        self.declare_parameter('width', 0.1218)
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 1.0)
        self.declare_parameter('Kd', 1.0)

        self.MAX_CAMERA_SPEED = self.get_parameter('max_camera_speed').get_parameter_value().double_value
        self.MAX_CAMERA_RANGE = self.get_parameter('max_camera_range').get_parameter_value().integer_value
        self.START_CAMERA_ANGLE = self.get_parameter('starting_camera_angle').get_parameter_value().integer_value
        self.MAX_RPM = self.get_parameter('max_rpm').get_parameter_value().integer_value
        self.WHEEL_RADIUS = self.get_parameter('radius').get_parameter_value().double_value
        self.LENGTH = self.get_parameter('length').get_parameter_value().double_value
        self.CIRC = self.get_parameter('circ').get_parameter_value().double_value
        self.MAX_SPEED = self.MAX_RPM * self.CIRC / 60  # m/s
        self.WIDTH = self.get_parameter('width').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        # Subscribers and publishers
        self.robot_speed_sub = self.create_subscription(RobotSpeed, 'robot_speed', self.robot_speed_callback, 10)
        self.camera_speed_sub = self.create_subscription(CameraSpeed, 'camera_speed', self.camera_speed_callback, 10)

        self.position_pub = self.create_publisher(Vector3, 'position', 10)

        # Motor setup
        self.M_left = Motor(5, 6, 13, 19, 26, self.MAX_RPM)
        self.M_right = Motor(14, 15, 18, 23, 24, self.MAX_RPM)
        self.left_rpm = 0
        self.right_rpm = 0
        self.angle = 0
        self.turning_radius = 0
        self.position = Vector3()

        # PID setup
        timer_period = 0.1  # s
        self.PID_left = PID(self.M_left, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        self.PID_right = PID(self.M_right, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        # self.PID_timer = self.create_timer(timer_period, self.PID_controller)
        self.pose_timer = self.create_timer(timer_period, lambda: self.pose_estimation(timer_period))

        # Track servo
        self.servo_track = Servo(22)

        # Camera setup
        self.servo_x = Servo(17, self.START_CAMERA_ANGLE, logger=self.get_logger())
        self.servo_y = Servo(27, self.START_CAMERA_ANGLE, logger=self.get_logger())
        self.last_servo_move = self.get_clock().now()

        # Led
        self.led = LED(25)

    def camera_speed_callback(self, msg):
        now = self.get_clock().now()
        dt = now - self.last_servo_move
        self.last_servo_move = now

        x_speed = self.MAX_CAMERA_SPEED * msg.wx
        y_speed = self.MAX_CAMERA_SPEED * msg.wy

        dx = x_speed * 1000000000 * dt.nanoseconds()
        dy = y_speed * 1000000000 * dt.nanoseconds()

        x = self.servo_x.angle + dx
        y = self.servo_y.angle + dy

        if abs(x) > self.MAX_CAMERA_RANGE:
            x = math.copysign(self.MAX_CAMERA_RANGE, x)
        if abs(y) > self.MAX_CAMERA_RANGE:
            y = math.copysign(self.MAX_CAMERA_RANGE, y)

        self.servo_x.set_angle(self.START_CAMERA_ANGLE + x)
        self.servo_y.set_angle(self.START_CAMERA_ANGLE + y)

    def robot_speed_callback(self, msg):
        for_move = msg.direction * self.MAX_RPM
        side_move = msg.pivot_direction * self.MAX_RPM

        if side_move == 0:
            self.left_rpm = for_move
            self.right_rpm = for_move
        elif for_move == 0:
            self.left_rpm = -side_move
            self.right_rpm = side_move
        else:
            if side_move < 0:
                self.right_rpm = for_move / 2
                self.left_rpm = for_move
            else:
                self.right_rpm = for_move
                self.left_rpm = for_move / 2

        self.M_left.set_rpm(self.left_rpm)
        self.M_right.set_rpm(self.right_rpm)
        self.led.set_level(msg.lights)
        self.servo_track.set_angle(msg.track_angle)
        # self.get_logger().info(f"right rpm: {self.right_rpm}, left rpm: {self.left_rpm}, direction: {msg.direction}")
        # self.get_logger().info(f"lights state {msg.lights}")
        # self.get_logger().info(f"track angle {msg.track_angle}")

    def PID_controller(self):
        self.PID_left.set_target_rpm(self.left_rpm)
        self.PID_right.set_target_rpm(self.right_rpm)

    def pose_estimation(self, dt):
        left_rpm = self.M_left.speed
        right_rpm = self.M_right.speed
        v = (left_rpm + right_rpm) * self.WHEEL_RADIUS / 2
        w = (left_rpm - right_rpm) * self.WHEEL_RADIUS / self.WIDTH

        # Update position
        self.position.x += math.cos(self.angle) * v * dt
        self.position.y += math.sin(self.angle) * v * dt
        self.position.z = 0.0
        self.position_pub.publish(self.position)

        # Update angle
        self.angle += w * dt


def main(args=None):
    rclpy.init(args=args)
    try:
        GPIO.setmode(GPIO.BCM)
        device_controller = DeviceController()
        rclpy.spin(device_controller)
    finally:
        device_controller.M_left.shutdown()
        device_controller.M_right.shutdown()
        device_controller.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
