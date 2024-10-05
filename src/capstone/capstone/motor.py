import RPi.GPIO as GPIO


class Motor:
    def __init__(self, in1, in2, en, enc_A, enc_B, max_speed):
        self.in1 = in1
        self.in2 = in2
        self.en = en
        self.enc_A = enc_A
        self.enc_B = enc_B
        self.pos = 0
        self.max_speed = max_speed

        # Pin setup
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.pwm(self.en, 1000)  # TODO change frequency
        self.pwm.start(0)

        GPIO.setup(self.enc_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.enc_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.enc_A, GPIO.RISING, callback=self.handle_event)

    def handle_event(self, pin):
        if GPIO.input(self.enc_B):
            self.pos = self.pos + 1
        else:
            self.pos = self.pos - 1

    def convertToDuty(self, x):
        if x > self.max_speed:
            ValueError(f"Needed speed of {x} is higher than max speed of {self.max_speed}")
        elif x < 0:
            ValueError(f"Needed speed of {x} is less than 0")

        return (x * 100) // self.max_speed

    def set_speed(self, speed):
        duty = self.convertToDuty(abs(speed))
        self.pwm.ChangeDutyCycle(duty)
        if speed > 0:
            GPIO.output(self.in1, 1)
            GPIO.output(self.in2, 0)
        else:
            GPIO.output(self.in1, 0)
            GPIO.output(self.in2, 1)


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

    def eval(self, value, target, deltaT):

        # Propotional
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
    def set_target_speed(self, target):
        pos = self.M.pos

        # Target RPM
        vt = target

        # Current encoder tick rate
        velocity = (pos - self.posPrev) / self.period  # ticks/sec
        self.posPrev = pos

        # Converted to RPM
        # 350 ticks per revolution of output shaft of the motor
        v = velocity / 350 * 60

        # Call for control signal
        x = int(self.eval(v, vt, self.period))

        # Bound control signal
        x = min(max(x, -self.M.max_speed), self.M.max_speed)

        # Set the motor speed
        self.M.set_speed(x)
        # print(v, vt)  # For debugging

