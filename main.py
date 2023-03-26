from gpiozero import LineSensor
from CLASS.motorControl import MotorControl
from CLASS.servoControl import ServoControl
from CLASS.ultrasonicControl import UltrasonicData
from CLASS.fireDetection import FireDetection
from CLASS.handGestureDetection import HandGestureDetection
from CLASS.slackIntegration import SlackMessage
import RPi.GPIO as GPIO
import time
import toml
import cv2
import threading

left_sensor = LineSensor(17, queue_len=20,
                         sample_rate=200, threshold=0.1)
right_sensor = LineSensor(27, queue_len=20,
                          sample_rate=200, threshold=0.1)

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)
MOTOR_NORMAL_SPEED = 60
MOOTOR_HIGH_SPEED = 70
MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)

# Create an instance of the ultrasonic sensor class
ULTRASONIC_SENSOR = UltrasonicData(echo=20, trigger=21)

# Create an instance of the fire detection class
FIRE_DETECTION = FireDetection("DATASET/Models/candle.h5")

# Create an instance of the hand gesture detection class
HAND_GESTURE_DETECTION = HandGestureDetection()
STOP_SECONDS = 10

# Load Config file for SLACK Token
CONFIG = toml.load("config.toml")
SLACK_MESSAGE = SlackMessage(
    CONFIG['slack']['webhook_url'], 'Fire Robot Connection Established.', 'good')
SLACK_MESSAGE.send()

# Start capturing the camera feed
CAP = cv2.VideoCapture(0)

# Servo Control
maxPW = (1.3+0.45)/1000
minPW = (1.0-0.45)/1000
servo = ServoControl(16, minPW, maxPW)
servo_swing = True


def motor_speed():
    while True:
        left_detect = int(left_sensor.value)
        right_detect = int(right_sensor.value)

        while ULTRASONIC_SENSOR.get_distance() > 20:
            if left_detect == 0 and right_detect == 0:
                MOTOR_CONTROL.forward()
            if left_detect == 0 and right_detect == 1:
                MOTOR_CONTROL.right()
            if left_detect == 1 and right_detect == 0:
                MOTOR_CONTROL.left()


def frame_processing():
    ret, frame = CAP.read()
    fd_output = FIRE_DETECTION.process_frame(frame)
    hd_output = HAND_GESTURE_DETECTION.detect_hand_gesture(frame)

    if fd_output == True:
        servo_swing = False
        SLACK_MESSAGE.send("Fire Detected!", "danger")
        MOTOR_CONTROL.set_speed(0)
        time.sleep(5)
    elif hd_output:
        MOTOR_CONTROL.set_speed(MOOTOR_HIGH_SPEED)
    elif hd_output == False:
        # closed hand, pause the robot.
        MOTOR_CONTROL.set_speed(0)
        time.sleep(STOP_SECONDS)
        MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        servo_swing = True
    elif fd_output == False:
        MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        servo_swing = True
    else:
        pass


def camera_handling():
    while True:
        if servo_swing:
            servo.center()
            time.sleep(2)
            frame_processing()
        if servo_swing:
            servo.right()
            time.sleep(2)
            frame_processing()
        if servo_swing:
            servo.center()
            time.sleep(2)
            frame_processing()
        if servo_swing:
            servo.left()
            time.sleep(2)
            frame_processing()
        else:
            frame_processing()


def main():
    try:
        threading.Thread(target=motor_speed).start()
        threading.Thread(target=camera_handling).start()
    except KeyboardInterrupt:
        for i in threading.enumerate():
            if i.name != 'MainThread':
                print(i.name)
                i._stop()
        print("Exiting Main Thread")
        GPIO.cleanup()
        CAP.release()


if __name__ == '__main__':
    main()
