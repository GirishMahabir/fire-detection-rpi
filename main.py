from gpiozero import LineSensor
from CLASS.motorControl import MotorControl
from CLASS.servoControl import ServoControl
from CLASS.ultrasonicControl import UltrasonicData
from CLASS.fireDetection import FireDetection
from CLASS.handGestureDetection import HandGestureDetection
from CLASS.slackIntegration import SlackMessage
import time
import toml
import cv2
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)

# Define the pins for the IR sensors
LEFT_PIN = 17
RIGHT_PIN = 27

# Define the threshold for the IR sensors
THRESHOLD = 0.5

# Create instances of the IR sensors
LEFT_SENSOR = LineSensor(LEFT_PIN, threshold=THRESHOLD)
RIGHT_SENSOR = LineSensor(RIGHT_PIN, threshold=THRESHOLD)

def read_ir_sensors():
    left_state = LEFT_SENSOR.value
    right_state = RIGHT_SENSOR.value
    return left_state, right_state

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)

# Create an instance of the ultrasonic sensor class
ULTRASONIC_SENSOR = UltrasonicData(echo=20, trigger=21)

# Create an instance of the fire detection class
FIRE_DETECTION = FireDetection("DATASET/Models/candle.h5")

# Create an instance of the hand gesture detection class
HAND_GESTURE_DETECTION = HandGestureDetection()

# Load Config file for SLACK Token
CONFIG = toml.load("config.toml")
SLACK_MESSAGE = SlackMessage(CONFIG['slack']['webhook_url'], 'Fire Robot Connection Established.', 'good')
SLACK_MESSAGE.send()

# Start capturing the camera feed
CAP = cv2.VideoCapture(0)

def process_frame():

    SERVO_SWING = True

    def swing_servo():
        if SERVO_SWING:
            maxPW = (1.3+0.45)/1000
            minPW = (1.0-0.45)/1000
            servo = ServoControl(16, minPW, maxPW)
            servo.center()
            time.sleep(3)
            servo.right()
            time.sleep(3)
            servo.center()
            time.sleep(3)
            servo.left()
            time.sleep(3)

    while True:
        ret, frame = CAP.read()
        fd_output = FIRE_DETECTION.process_frame(frame)
        hd_output = HAND_GESTURE_DETECTION.detect_hand_gesture(frame)

        print(fd_output, hd_output)

        if fd_output:
            SERVO_SWING = False
            SLACK_MESSAGE.send("Fire Detected!", "danger")
            MOTOR_CONTROL.set_speed(0)
            time.sleep(2)
            continue
        elif hd_output:
            MOTOR_CONTROL.set_speed(100)
        elif hd_output == False:
            # closed hand, pause the robot.
            MOTOR_CONTROL.set_speed(0)
            time.sleep(5)
            MOTOR_CONTROL.set_speed(50)
            SERVO_SWING = True
        elif fd_output == False:
            MOTOR_CONTROL.set_speed(50)
            SERVO_SWING = True
        else:
            pass
        
        swing_servo()

def handle_obstacle():
    while True:
        print(f"The time is {time.time()}")

        obstacle_distance = ULTRASONIC_SENSOR.get_distance()
        left_state, right_state = read_ir_sensors()

        print(f"Obstacle Distance: {obstacle_distance}")
        print(f"Left State: {left_state}, Right State: {right_state}")

        if obstacle_distance < 10:
            MOTOR_CONTROL.set_speed(0)
            time.sleep(1)
            if ULTRASONIC_SENSOR.get_distance() < 8:
                MOTOR_CONTROL.set_speed(0)
                time.sleep(1)
                MOTOR_CONTROL.reverse()
                time.sleep(1)
                MOTOR_CONTROL.set_speed(50)
                time.sleep(1)
                
                if left_state == 1 and right_state == 1:
                    MOTOR_CONTROL.reverse()
                elif left_state == 1 and right_state == 0:
                    MOTOR_CONTROL.right()
                elif left_state == 0 and right_state == 1:
                    MOTOR_CONTROL.left()

                time.sleep(1)
        else:
            MOTOR_CONTROL.forward()
            if left_state == 1 and right_state == 1:
                MOTOR_CONTROL.forward()
            elif left_state == 1 and right_state == 0:
                MOTOR_CONTROL.right()
            elif left_state == 0 and right_state == 1:
                MOTOR_CONTROL.left()
        
        time.sleep(0.5)

# Define the main loop
try:
    threading.Thread(target=handle_obstacle).start()
    # threading.Thread(target=process_frame).start()
except KeyboardInterrupt:
    MOTOR_CONTROL.set_speed(0)
    # terminate threads
    for t in threading.enumerate():
        if t is not threading.currentThread():
            t.join()
    SLACK_MESSAGE.send("Fire Robot Connection Terminated.", "danger")
    GPIO.cleanup()
    CAP.release()

# Path: main.py