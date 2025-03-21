#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import RobotSpeed
from geometry_msgs.msg import Vector3

import math
from capstone.motor import Motor, PID, Servo, LED
import RPi.GPIO as GPIO


class DeviceController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Constants
        self.declare_parameter('max_rpm', 0)
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('length', 0.0)
        self.declare_parameter('circ', 0.0)
        self.declare_parameter('width', 0.0)
        self.declare_parameter('Kp', 0.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)

        # TODO move parameters to ROS param
        self.MAX_RPM = self.get_parameter('max_rpm').get_parameter_value().integer_value
        self.WHEEL_RADIUS = self.get_parameter('radius').get_parameter_value().double_value
        self.LENGTH = self.get_parameter('length').get_parameter_value().double_value
        self.CIRC = self.get_parameter('circ').get_parameter_value().double_value
        self.MAX_SPEED = self.MAX_RPM * self.CIRC / 60  # m/s
        self.WIDTH = self.get_parameter('width').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.get_logger().info(f"{self.MAX_RPM}")

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(RobotSpeed, 'robot_speed', self.speed_callback,10)

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
        timer_period = 0.1  # s TODO find best period
        self.PID_left = PID(self.M_left, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        self.PID_right = PID(self.M_right, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        self.PID_timer = self.create_timer(timer_period, self.PID_controller)
        self.pose_timer = self.create_timer(timer_period, lambda: self.pose_estimation(timer_period))

        # Track servo
        self.servo_track = Servo(17)

        # Led
        self.led = LED(25)


    def speed_callback(self, msg):
        speed = msg.speed * self.MAX_SPEED

        if msg.turning_radius == 0:
            self.left_rpm = self.right_rpm = speed / self.WHEEL_RADIUS
            return

        self.turning_radius = r = msg.turning_radius  # cm
        ang_speed = speed / r
        s1 = speed + self.WIDTH * ang_speed / 2
        s2 = speed - self.WIDTH * ang_speed / 2
        w1 = (s1 + (self.LENGTH * ang_speed / 2) ** 2 / s1) / self.WHEEL_RADIUS
        w2 = (s2 + (self.LENGTH * ang_speed / 2) ** 2 / s2) / self.WHEEL_RADIUS

        if msg.turning_radius > 0:
            self.right_rpm = w1
            self.left_rpm = w2
        else:
            self.right_rpm = w2
            self.left_rpm = w1

        self.get_logger().debug(
            f"right rpm: {self.right_rpm}, left rpm: {self.left_rpm}, direction: {msg.direction}")
        
        self.led.set_state(msg.lights)
        self.servo_track.set_angle(msg.track_angle)

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
        self.position.z = 0
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
