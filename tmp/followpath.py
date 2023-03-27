from time import sleep
from CLASS.motorControl import MotorControl
from gpiozero import LineSensor
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

# Define the pins for the IR sensors
LEFT_PIN = 17
RIGHT_PIN = 27

left_sensor = LineSensor(LEFT_PIN)
right_sensor= LineSensor(RIGHT_PIN)

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)
MOTOR_CONTROL.set_speed(70)

def motor_speed():
    left_detect  = int(left_sensor.value)
    right_detect = int(right_sensor.value)
    
    ## Stage 1
    if left_detect == 0 and right_detect == 0:
        MOTOR_CONTROL.forward()
   
    ## Stage 2
    if left_detect == 0 and right_detect == 1:
        MOTOR_CONTROL.right()
        sleep(0.01)

    if left_detect == 1 and right_detect == 0:
        MOTOR_CONTROL.left()
        sleep(0.01)

try:
    while True:
        motor_speed()
except KeyboardInterrupt:
    GPIO.cleanup()
    exit()
