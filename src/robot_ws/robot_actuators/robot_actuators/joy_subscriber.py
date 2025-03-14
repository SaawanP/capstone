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
        self.servo_pin_track  = 27  # Example: Servo PWM signal
        self.servo_pin_x_camera = 23  # Example: Servo PWM signal
        self.servo_pin_y_camera = 24  # Example: Servo PWM signal
        self.en = 25
        
        GPIO.setup(self.motor_pin1, GPIO.OUT)
        GPIO.setup(self.motor_pin2, GPIO.OUT)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.servo_pin_track, GPIO.OUT)
        GPIO.setup(self.servo_pin_x_camera, GPIO.OUT)
        GPIO.setup(self.servo_pin_y_camera, GPIO.OUT)

        # Setup PWM for the servo (50Hz is typical)
        self.servo_pwm_track = GPIO.PWM(self.servo_pin_track, 50)
        self.track_angle = 0
        self.servo_pwm_track.start(0)  # Neutral position
        self.servo_pwm_x_camera = GPIO.PWM(self.servo_pin_x_camera, 50)
        self.servo_pwm_x_camera.start(50)  # Neutral position
        self.servo_pwm_y_camera = GPIO.PWM(self.servo_pin_y_camera, 50)
        self.servo_pwm_y_camera.start(50)  # Neutral position

        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(0)

        self.led = False

    def joy_callback(self, msg):
        # --- Motor Control ---
        if msg.axes[1] > 50:
            self.pwm.ChangeDutyCycle(msg.axes[1]//32767)
            GPIO.output(self.motor_pin1, GPIO.HIGH)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.get_logger().info('Motor: Forward')
        elif msg.axes[1] < 50:
            self.pwm.ChangeDutyCycle(msg.axes[1] // 32767)
            GPIO.output(self.motor_pin1, GPIO.LOW)
            GPIO.output(self.motor_pin2, GPIO.HIGH)
            self.get_logger().info('Motor: Reverse')
        else:
            GPIO.output(self.motor_pin1, GPIO.LOW)
            GPIO.output(self.motor_pin2, GPIO.LOW)
            self.get_logger().info('Motor: Stop')

        # Map axis value (-1.0 to 1.0) to a duty cycle range (e.g., 5% to 10%)
        duty = (msg.axes[3] // 32767 + 1) * 50
        self.servo_pwm_y_camera.ChangeDutyCycle(duty)

        duty = (msg.axes[2] // 32767 + 1) * 50
        self.servo_pwm_x_camera.ChangeDutyCycle(duty)

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
            self.track_angle += 0.1
            if self.track_angle > 5:
                self.track_angle = 5
        if msg.buttons[2]:
            self.track_angle -= 0.1
            if self.track_angle < 0:
                self.track_angle = 0

        duty = self.track_angle * 100 / 180
        self.servo_pwm_track.ChangeDutyCycle(duty)

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

