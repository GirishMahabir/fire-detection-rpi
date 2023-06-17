from gpiozero import LineSensor
from CLASS.motorControl import MotorControl
from CLASS.servoControl import ServoControl
from CLASS.ultrasonicControl import UltrasonicData
from CLASS.fireDetection import FireDetection
from CLASS.handGestureDetection import HandGestureDetection
from CLASS.slackIntegration import SlackMessage
import RPi.GPIO as GPIO
from time import sleep
import datetime as dt
import toml
import cv2
import threading

GPIO.setmode(GPIO.BCM)

PROG_TIME = dt.datetime.now().second

LEFT_SENSOR = LineSensor(17)
RIGHT_SENSOR = LineSensor(27)

GPIO.setmode(GPIO.BCM)

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)
MOTOR_NORMAL_SPEED = 60
MOOTOR_HIGH_SPEED = 65
MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
MOTOR_ON_WHEEL_TIME = 12

# Create an instance of the ultrasonic sensor class
ULTRASONIC_SENSOR = UltrasonicData(echo=20, trigger=21)

# Create an instance of the fire detection class
FIRE_DETECTION = FireDetection("DATASET/Models/candle.h5")

# Create an instance of the hand gesture detection class
HAND_GESTURE_DETECTION = HandGestureDetection()
HAND_GESTURE_STOP_SECONDS = 60

# Load Config file for SLACK Token
CONFIG = toml.load("config.toml")
# SLACK_MESSAGE = SlackMessage(
#     CONFIG['slack']['webhook_url'], 'Fire Robot Connection Established.', 'good')
# SLACK_MESSAGE.send()

# Start capturing the camera feed
CAP = cv2.VideoCapture(0)

# SERVO Control
maxPW = (1.3+0.45)/1000
minPW = (1.0-0.45)/1000
SERVO = ServoControl(16, minPW, maxPW)
SERVO_WAIT_SECONDS = 3

def detect_obs():
    global PROG_TIME
    while True:
        if ULTRASONIC_SENSOR.get_distance() <= 10:
            MOTOR_CONTROL.set_speed(0)
            sleep(5)
            MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
            detect_obs()
        elif (dt.datetime.now().second - PROG_TIME) >= MOTOR_ON_WHEEL_TIME:
                MOTOR_CONTROL.set_speed(0)
                sleep(MOTOR_ON_WHEEL_TIME)
                PROG_TIME = dt.datetime.now().second
                MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        else:
            sleep(1)
 
def motor_speed():    
    global PROG_TIME
    while True:
        left_detect  = int(LEFT_SENSOR.value)
        right_detect = int(RIGHT_SENSOR.value)

        if left_detect == 0 and right_detect == 0:
            MOTOR_CONTROL.forward()
            if (dt.datetime.now().second - PROG_TIME) >= MOTOR_ON_WHEEL_TIME:
                MOTOR_CONTROL.set_speed(0)
                sleep(MOTOR_ON_WHEEL_TIME)
                PROG_TIME = dt.datetime.now().second
                MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        if left_detect == 0 and right_detect == 1:
            MOTOR_CONTROL.right()
            sleep(0.15)
        if left_detect == 1 and right_detect == 0:
            MOTOR_CONTROL.left()
            sleep(0.15)
        
def frame_processing():
    # sleep(2)        
    if (dt.datetime.now().second - PROG_TIME) <= MOTOR_ON_WHEEL_TIME:
        ret, frame = CAP.read()
        fd_output = FIRE_DETECTION.process_frame(frame)
        hd_output = HAND_GESTURE_DETECTION.detect_hand_gesture(frame)
        print(fd_output, hd_output)
        if fd_output: # Fire detected, True
            # Fire detected, stop the robot.
            # SLACK_MESSAGE.send("Fire Detected!", "danger")
            print("FIRE DETECTED - NOTIFICATION SENT!")
        
        if hd_output == True: # Open hand detected, True
            # open hand, increase the speed of the robot.
            MOTOR_CONTROL.set_speed(MOOTOR_HIGH_SPEED)
            print("Increasing Motor speed!")

        if hd_output == False: # Closed hand detected, False
            # closed hand, pause the robot for STOP_SECONDS seconds.
            MOTOR_CONTROL.set_speed(0)
            print("Closed Hand Detected!")
            sleep(HAND_GESTURE_STOP_SECONDS)
            MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)

def camera_handling():
    while True:
        frame_processing()
        SERVO.center()
        sleep(SERVO_WAIT_SECONDS)
        frame_processing()
        SERVO.right()
        sleep(SERVO_WAIT_SECONDS)
        frame_processing()
        SERVO.center()
        sleep(SERVO_WAIT_SECONDS)
        frame_processing()
        SERVO.left()
        sleep(SERVO_WAIT_SECONDS)
        frame_processing()

def main():
    try:
        PROG_TIME = dt.datetime.now().second
        threading.Thread(target=detect_obs).start()
        threading.Thread(target=motor_speed).start()
        threading.Thread(target=camera_handling).start()
    except KeyboardInterrupt:
        SERVO.center()
        # SLACK_MESSAGE.send("Exiting..Keyboard Interrupt Detected!", "warning")
        print("EXITING _ NOTIF SENT")
        for i in threading.enumerate():
            if i is not threading.current_thread():
                i.join()

        print("Exiting Main Thread")
        GPIO.cleanup()
        CAP.release()


if __name__ == '__main__':
    main()