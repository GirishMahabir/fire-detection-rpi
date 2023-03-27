from time import sleep

from CLASS.motorControl import MotorControl
from gpiozero import LineSensor
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

# Define the pins for the IR sensors
LEFT_PIN = 17
RIGHT_PIN = 27

left_sensor = LineSensor(LEFT_PIN)
right_sensor= LineSensor(RIGHT_PIN)

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)
MOTOR_CONTROL.set_speed(70)

# Initialize the history buffer with the current sensor values
history_length = 10
left_detect_history = [int(left_sensor.value)] * history_length
right_detect_history = [int(right_sensor.value)] * history_length

def motor_speed():
    # Update the history buffer with the current sensor values
    left_detect_history.pop(0)
    left_detect_history.append(int(left_sensor.value))
    right_detect_history.pop(0)
    right_detect_history.append(int(right_sensor.value))
    
    # Apply the smoothing filter to the history buffer
    left_detect_smooth = sum(left_detect_history) / len(left_detect_history)
    right_detect_smooth = sum(right_detect_history) / len(right_detect_history)
    
    ## Stage 1
    if left_detect_smooth == 0 and right_detect_smooth == 0:
        MOTOR_CONTROL.forward()
   
    ## Stage 2
    if left_detect_smooth == 0 and right_detect_smooth == 1:
        MOTOR_CONTROL.right()
        sleep(0.11)
        
    if left_detect_smooth == 1 and right_detect_smooth == 0:
        MOTOR_CONTROL.left()
        sleep(0.11)

try:
    while True:
        motor_speed()
except KeyboardInterrupt:
    GPIO.cleanup()
    exit()

