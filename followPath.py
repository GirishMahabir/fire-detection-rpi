import RPi.GPIO as GPIO
from gpiozero import Servo
from time import sleep

# Set up GPIO pins
# IR sensors
GPIO.setmode(GPIO.BOARD)
# Left
GPIO.setup(17, GPIO.IN)
# Right
GPIO.setup(27, GPIO.IN)

# Ultrasonic sensor
# Trigger
GPIO.setup(21, GPIO.OUT)
# Echo
GPIO.setup(20, GPIO.IN)


GPIO.setmode(GPIO.BCM)
# Motor control
ENA = 25
ENB = 13
IN1 = 24
IN2 = 23
IN3 = 26
IN4 = 19

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)
speed = 50
turnSpeed = 50
stop = 0
forward = 1
backward = 2
left = 3
right = 4