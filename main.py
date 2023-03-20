# Import necessary libraries
from gpiozero import LineSensor
from motorControl import MotorControl
from ultrasonicControl import UltrasonicData
from fireDetection import FireDetection
from handGestureDetection import HandGestureDetection

import time

# Define the pins for the IR sensors
left_pin = 17
right_pin = 27

# Define the threshold for the IR sensors
threshold = 0.5

# Create instances of the IR sensors
left_sensor = LineSensor(left_pin, threshold=threshold)
right_sensor = LineSensor(right_pin, threshold=threshold)

# Create an instance of the motor control class
motor_control = MotorControl(24, 23, 25, 26, 19, 13)

# Create an instance of the ultrasonic sensor class
ultrasonic_sensor = UltrasonicData(echo=20, trigger=21)

# Define the movement functions


def forward():
    motor_control.set_speed(100)
    motor_control.forward()


def left():
    motor_control.set_speed(100)
    motor_control.left()


def right():
    motor_control.set_speed(100)
    motor_control.right()


def stop():
    motor_control.stop()


def reverse():
    motor_control.set_speed(100)
    motor_control.reverse()


# Define the main loop
while True:
    # Read the state of the IR sensors
    left_state = left_sensor.value
    right_state = right_sensor.value

    # Determine the movement based on the IR sensor readings
    if left_state == 0 and right_state == 0:
        # Both sensors off the line, go straight
        forward()
    elif left_state == 1 and right_state == 0:
        # Left sensor on the line, turn right
        right()
    elif left_state == 0 and right_state == 1:
        # Right sensor on the line, turn left
        left()
    else:
        # Both sensors on the line, go straight
        forward()

    # Check for obstacles
    obstacle_distance = ultrasonic_sensor.get_distance()
    if obstacle_distance < 10:
        stop()
        time.sleep(1)
        reverse()
        if left_state == 0 and right_state == 0:
            forward()
        elif left_state == 1 and right_state == 0:
            right()
        elif left_state == 0 and right_state == 1:
            left()
        else:
            forward()
