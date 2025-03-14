#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Initialize GPIO for the L293D motor controller, servo, and LED.
        GPIO.setmode(GPIO.BCM)
        self.motor_pin1 = 17  # Example: forward direction
        self.motor_pin2 = 18  # Example: reverse direction
        self.led_pin    = 22  # Example: LED control
        self.servo_pin  = 27  # Example: Servo PWM signal
        self.en = 28
        
        GPIO.setup(self.motor_pin1, GPIO.OUT)
        GPIO.setup(self.motor_pin2, GPIO.OUT)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # Setup PWM for the servo (50Hz is typical)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)
        self.servo_pwm.start(7.5)  # Neutral position

        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.pwm(self.en, 1000)
        self.pwm.start(0)

        self.led = False

    def joy_callback(self, msg):
        # --- Motor Control ---
        if msg.axes[1] > 50:
            self.pwm.ChangeDutyCycle(msg.axes[1]//380)
            GPIO.output(self.motor_pin1, GPIO.HIGH)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.get_logger().info('Motor: Forward')
        elif msg.axes[1] < 50:
            self.pwm.ChangeDutyCycle(msg.axes[1] // 380)
            GPIO.output(self.motor_pin1, GPIO.LOW)
            GPIO.output(self.motor_pin2, GPIO.HIGH)
            self.get_logger().info('Motor: Reverse')
        else:
            GPIO.output(self.motor_pin1, GPIO.LOW)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.get_logger().info('Motor: Stop')

        # --- LED Control ---
        if msg.buttons[0]:
            self.led = not self.led
            if self.led:
                GPIO.output(self.led_pin, GPIO.HIGH)
                self.get_logger().info('LED: ON')
            else:
                GPIO.output(self.led_pin, GPIO.LOW)
                self.get_logger().info('LED: OFF')

        # --- Servo Control ---
        # Use button 1 with axis[0] to control the servo position:
        if msg.buttons[1]:
            # Map axis value (-1.0 to 1.0) to a duty cycle range (e.g., 5% to 10%)
            duty = 7.5 + (msg.axes[0] * 2.5)
            self.servo_pwm.ChangeDutyCycle(duty)
            self.get_logger().info(f'Servo: Duty Cycle set to {duty:.2f}')

    def shutdown(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    GPIO.setmode(GPIO.BCM)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

