#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interface.msg import Speed


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.MAX_RPM = 70  # rpm TODO fix numbers
        self.CIRC = 1  # m
        self.MAX_SPEED = self.MAX_RPM * self.CIRC  # m/min
        self.WIDTH = 5  # cm
        self.Kp = 0.8  # TODO tune parameters
        self.Ki = 0.2
        self.Kd = 0.1

        # Subscribers and publishers
        self.speed_sub = self.create_subscription(
            Speed,
            'robot_speed',
            self.speed_callback,
            10
        )

        self.timer_period = 0.5  # s
        self.timer = self.create_timer(self.timer_period, self.PID_controller)  # change period
        self.left_integral = 0
        self.right_integral = 0
        self.left_e_prev = 0
        self.right_e_prev = 0

        self.left_rpm = 0
        self.right_rpm = 0

    def speed_callback(self, msg):
        speed = msg.dist * self.MAX_SPEED
        if msg.x / abs(msg.x) > 0:
            # TODO high on Pin1 for both motors and low on Pin 2
            pass
        else:
            # TODO high on Pin2 for both motors and low on Pin 1
            pass
        if msg.y == 0:
            self.left_rpm = self.right_rpm = speed / self.CIRC
            return

        r = abs(1 / msg.y)  # cm
        w1 = r / (r + self.WIDTH / 2) * speed / self.CIRC
        w2 = (r + self.width) / (r + self.WIDTH / 2) * speed / self.CIRC

        if msg.y > 0:
            self.right_rpm = w1
            self.left_rpm = w2
        else:
            self.right_rpm = w2
            self.left_rpm = w1

        self.get_logger().debug(
            f"right rpm: {self.right_rpm}, left rpm: {self.left_rpm}, direction: {msg.x / abs(msg.x)}")

    def PID_controller(self):
        # TODO get current rpm
        left_measurement = 0
        right_measurement = 0

        # left PID controller
        e = self.left_rpm - left_measurement
        P = self.Kp * e
        self.left_integral += self.Ki * e * self.timer_period
        D = self.Kd * (e - self.left_e_prev) / self.timer_period
        left_rpm = P + self.integral + D

        # right PID controller
        e = self.right_rpm - right_measurement
        P = self.Kp * e
        self.right_integral += self.Ki * e * self.timer_period
        D = self.Kd * (e - self.right_e_prev) / self.timer_period
        right_rpm = P + self.integral + D

        left_pwm = self.pwm_map(left_rpm)
        right_pwm = self.pwm_map(right_rpm)

        # TODO set pwm for motors

    def pwm_map(self, x):
        # TODO check if rpm is a linear function to rpm
        if 0 < x < self.MAX_RPM:
            self.get_logger().info(f"rpm is {x} which is out of range")
        return x / self.MAX_RPM * 255


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
