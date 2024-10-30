#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd

from robot_interface.msg import Speed
from motor import Motor, PID


def speed_monitor():
    m1_data = pd.read_csv('data/M1.csv')
    m1_time = m1_data["Time"]
    m1_speed = m1_data["Speed"]

    m2_data = pd.read_csv('data/M2.csv')
    m2_time = m2_data["Time"]
    m2_speed = m2_data["Speed"]

    pid1_data = pd.read_csv('data/PID1.csv')
    pid1_time = pid1_data["Time"]
    pid1_speed = pid1_data["Speed"]
    pid1_control = pid1_data["Control Signal"]

    pid2_data = pd.read_csv('data/PID2.csv')
    pid2_time = pid2_data["Time"]
    pid2_speed = pid2_data["Speed"]
    pid2_control = pid2_data["Control Signal"]

    plt.cla()
    plt.plot(m1_time, m1_speed, label="M1 Speed")
    plt.plot(m2_time, m2_speed, label="M2 Speed")
    plt.plot(pid1_time, pid1_speed, label="PID1 Calculated Speed")
    plt.plot(pid1_time, pid1_control, label="PID1 Control Signal")
    plt.plot(pid2_time, pid2_speed, label="PID2 Calculated Speed")
    plt.plot(pid2_time, pid2_control, label="PID2 Control Signal")

    plt.legend(loc='upper left')


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
        tracking = True

        # Motor setup
        self.M1 = Motor(9, 10, 11, 12, 13, self.MAX_SPEED, tracking_name="M1" if tracking else None)
        self.M2 = Motor(14, 15, 16, 17, 18, self.MAX_SPEED, tracking_name="M2" if tracking else None)
        self.left_rpm = 0
        self.right_rpm = 0

        # PID setup
        timer_period = 0.1  # s TODO find best period
        self.PID1 = PID(self.M1, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki, tracking_name="PID1" if tracking else None)
        self.PID2 = PID(self.M2, timer_period, kp=self.Kp, kd=self.Kd, ki=self.Ki, tracking_name="PID2" if tracking else None)
        self.timer = self.create_timer(timer_period, self.PID_controller)

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(
            Speed,
            'robot_speed',
            self.speed_callback,
            10
        )

        if tracking:
            ani = FuncAnimation(plt.gcf(), speed_monitor, timer_period * 1000)
            plt.ion()
            plt.show(block=False)

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
        motor_controller.M1.shutdown()
        motor_controller.M2.shutdown()
        motor_controller.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
