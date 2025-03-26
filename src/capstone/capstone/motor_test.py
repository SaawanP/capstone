import RPi.GPIO as GPIO
import time

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
        self.pwm = GPIO.pwm(self.en, 1000)
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


m = Motor(17, 27, 22, 23, 25, 10, 11)

m.set_rpm(m.max_rpm)

start = time.time()
while time.time() - start < 10:
    print((m.pos, m.speed))


m.shutdown()

