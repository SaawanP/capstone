#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from robot_interface.msg import Speed
from motor import Motor, PID

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # TODO move parameters to ROS param
        self.MAX_RPM = 70  # rpm TODO fix numbers
        self.CIRC = 1  # m
        self.MAX_SPEED = self.MAX_RPM * self.CIRC  # m/min
        self.WIDTH = 5  # cm
        self.Kp = 0.8  # TODO tune parameters
        self.Ki = 0.2
        self.Kd = 0.1

        # Motor setup
        self.M1 = Motor(9, 10, 11, 12, 13, self.MAX_SPEED)
        self.M2 = Motor(14,15, 16, 17,18, self.MAX_SPEED)
        self.left_rpm = 0
        self.right_rpm = 0


        # PID setup
        timer_period = 0.1  # s TODO find best period
        self.PID1 = PID(self.M1, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        self.PID2 = PID(self.M2, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki)
        self.timer = self.create_timer(timer_period, self.PID_controller)

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(
            Speed,
            'robot_speed',
            self.speed_callback,
            10
        )

    def speed_callback(self, msg):
        speed = msg.dist * self.MAX_SPEED

        if msg.y == 0:
            self.left_rpm = self.right_rpm = speed / self.CIRC
            return

        r = abs(1 / msg.y)  # cm
        w1 = r / (r + self.WIDTH / 2) * speed / self.CIRC
        w2 = (r + self.WIDTH) / (r + self.WIDTH / 2) * speed / self.CIRC

        if msg.y > 0:
            self.right_rpm = w1
            self.left_rpm = w2
        else:
            self.right_rpm = w2
            self.left_rpm = w1

        self.get_logger().debug(
            f"right rpm: {self.right_rpm}, left rpm: {self.left_rpm}, direction: {msg.x / abs(msg.x)}")

    def PID_controller(self):
        self.PID1.set_target_speed(self.left_rpm)
        self.PID2.set_target_speed(self.right_rpm)


def main(args=None):
    motor_controller = MotorController()
    try:
        GPIO.setmode(GPIO.BOARD)
        rclpy.init(args=args)
        rclpy.spin(motor_controller)
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
