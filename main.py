# Import necessary libraries
from gpiozero import LineSensor
from CLASS.motorControl import MotorControl
from CLASS.ultrasonicControl import UltrasonicData
from CLASS.fireDetection import FireDetection as fd
from CLASS.handGestureDetection import HandGestureDetection as hd
from CLASS.slackIntegration import SlackMessage as sm

import time
import toml
import cv2

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

# Create an instance of the fire detection class
fd = fd("../DATASET/Models/candle.h5")

# Create an instance of the hand gesture detection class
hd = hd()

# Load Config file for SLACK Token
config = toml.load("config.toml")
sm = sm(config['slack']['webhook_url'],
        'Fire Robot Connection Established.', 'good')
sm.send()

# Start capturing the camera feed
cap = cv2.VideoCapture(0)

# Define the main loop
while True:

    try:
        ret, frame = cap.read()
        # if frame is available, process it:
        if ret:
            fd_output = fd.process_frame(frame)
        if ret:
            hd_output = hd.detect_hand_gesture(frame)

        if fd_output:
            sm.send(sm(config['slack']['webhook_url'],
                    'Fire Detected!', 'danger'))
            motor_control.stop()
            time.sleep(10)
            continue  # Go back to the beginning of the loop

        if hd_output:
            # Open HAND, go forward faster and the closest possible to the hand while
            # being on the line.
            motor_control.set_speed(100)

        if not hd_output:
            # stop the robot for 30 seconds.
            motor_control.stop()
            time.sleep(30)
            continue

        if fd_output == None:
            motor_control.set_speed(50)

        # Read the state of the IR sensors
        left_state = left_sensor.value
        right_state = right_sensor.value

        # Determine the movement based on the IR sensor readings
        if left_state == 0 and right_state == 0:
            # Both sensors off the line, go straight
            motor_control.forward()
        elif left_state == 1 and right_state == 0:
            # Left sensor on the line, turn right
            motor_control.right()
        elif left_state == 0 and right_state == 1:
            # Right sensor on the line, turn left
            motor_control.left()
        else:
            # Both sensors on the line, go straight
            motor_control.forward()

        # Check for obstacles
        obstacle_distance = ultrasonic_sensor.get_distance()
        if obstacle_distance < 10:
            motor_control.stop()
            time.sleep(1)
            motor_control.reverse()
            if left_state == 0 and right_state == 0:
                motor_control.forward()
            elif left_state == 1 and right_state == 0:
                motor_control.right()
            elif left_state == 0 and right_state == 1:
                motor_control.left()
            else:
                motor_control.forward()

    except KeyboardInterrupt:
        # Release the camera
        cap.release()
