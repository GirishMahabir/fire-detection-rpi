import RPi.GPIO as GPIO
from time import sleep

class MotorControl:
    def __init__(self, in1, in2, ena, in3, in4, enb):
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.in3 = in3
        self.in4 = in4
        self.enb = enb
        self.speed = 50

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ena, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.enb, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)

        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)

        self.motor_one = GPIO.PWM(self.ena, 1000)
        self.motor_two = GPIO.PWM(self.enb, 1000)

        self.motor_one.start(self.speed)
        self.motor_two.start(self.speed)

    def forward(self):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

    def reverse(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)

    def stop(self):
        self.motor_one.stop()
        self.motor_two.stop()
        GPIO.cleanup()

    def right(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

    def left(self):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)

    def set_speed(self, speed):
        self.speed = speed
        self.motor_one.ChangeDutyCycle(self.speed)
        self.motor_two.ChangeDutyCycle(self.speed)

    def get_speed(self):
        return self.speed


if __name__ == '__main__':
    # DEFINE PINS
    ENA = 25
    IN1 = 24
    IN2 = 23
    IN3 = 26
    IN4 = 19
    ENB = 13

    # CREATE MOTOR CONTROL OBJECT
    motor_control = MotorControl(IN1, IN2, ENA, IN3, IN4, ENB)

    # TEST MOTOR CONTROL
    motor_control.forward()
    sleep(10)
    motor_control.reverse()
    sleep(10)

    # TEST MOTOR DIRECTION
    motor_control.set_speed(50)
    motor_control.forward()
    sleep(10)
    motor_control.right()
    sleep(10)
    motor_control.left()
    sleep(10)
    motor_control.stop()
