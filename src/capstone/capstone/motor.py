import RPi.GPIO as GPIO
import time
import csv
from typing import Optional


class Motor:
    def __init__(self, in1, in2, en, enc_A, enc_B, max_rpm, encoder_ticks=350):
        self.max_rpm = max_rpm
        self.encoder_ticks = encoder_ticks
        self.in1 = in1
        self.in2 = in2
        self.en = en
        self.enc_A = enc_A
        self.enc_B = enc_B
        self.pos = 0
        self.last_pos = 0
        self.last_event_time = time.time()
        self.speed = 0

        # Pin setup
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(0)

        GPIO.setup(self.enc_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.enc_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.enc_A, GPIO.RISING, callback=self.handle_event)

    def handle_event(self, pin):
        if GPIO.input(self.enc_B):
            self.pos = self.pos + 1
        else:
            self.pos = self.pos - 1

        dt = time.time() - self.last_event_time
        self.last_event_time = time.time()
        dx = self.pos - self.last_pos
        self.last_pos = self.pos
        v = dx / dt  # ticks/sec
        self.speed = v / self.encoder_ticks * 60  # RPM

    def convert_to_duty(self, x):
        if x > self.max_rpm:
            raise ValueError(f"Needed speed of {x} is higher than max speed of {self.max_rpm}")
        elif x < 0:
            raise ValueError(f"Needed speed of {x} is less than 0")

        return (x * 100) // self.max_rpm

    def set_rpm(self, rpm):
        duty = self.convert_to_duty(abs(rpm))
        self.pwm.ChangeDutyCycle(duty)
        if rpm > 0:
            GPIO.output(self.in1, 1)
            GPIO.output(self.in2, 0)
        else:
            GPIO.output(self.in1, 0)
            GPIO.output(self.in2, 1)

    def shutdown(self):
        self.pwm.stop()


class PID:
    def __init__(self, M: Motor, period, kp=1, kd=0, ki=0, e_prev=0, e_integral=0):
        self.M: Motor = M  # Motor object
        self.period = period
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.e_prev = e_prev
        self.e_integral = e_integral
        self.posPrev = 0
        self.calculated_speed = 0

    def eval(self, value, target, deltaT):
        # Proportional
        e = target - value

        # Derivative
        de_dt = (e - self.e_prev) / deltaT

        # Integral
        self.e_integral += e * deltaT

        # Control signal
        u = self.kp * e + self.kd * de_dt + self.ki * self.e_integral

        self.e_prev = e
        return u

    # Function for closed loop speed control
    def set_target_rpm(self, target):
        pos = self.M.pos

        # Target RPM
        vt = target

        # Current encoder tick rate
        velocity = (pos - self.posPrev) / self.period  # ticks/sec
        self.posPrev = pos

        # Converted to RPM
        v = velocity / self.M.encoder_ticks * 60

        # Call for control signal
        x = int(self.eval(v, vt, self.period))

        # Bound control signal
        x = min(max(x, -self.M.max_rpm), self.M.max_rpm)


        # Set the motor speed
        self.M.set_rpm(x)


class Servo:
    def __init__(self, pin, starting_angle=0):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, 1)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(7)  # Start servo at 90 degrees
        self.angle = starting_angle
        self.set_angle(self.angle)

    def set_angle(self, angle):
        self.angle = angle
        duty = angle / 18 + 2
        self.pwm.ChangeDutyCycle(duty)

    def reset(self):
        self.set_angle(90)

    def shutdown(self):
        self.pwm.stop()

class LED:
    def __init__(self, pin):
        GPIO.setup(pin,GPIO.OUT)

    def set_state(self, state):
        if state:
            GPIO.output(18,GPIO.HIGH)
        else:
            GPIO.output(18,GPIO.LOW)
