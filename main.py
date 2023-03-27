from gpiozero import LineSensor
from CLASS.motorControl import MotorControl
from CLASS.servoControl import ServoControl
from CLASS.ultrasonicControl import UltrasonicData
# from CLASS.fireDetection import FireDetection
# from CLASS.handGestureDetection import HandGestureDetection
from CLASS.slackIntegration import SlackMessage
import RPi.GPIO as GPIO
import time
import toml
import cv2
import threading

THREAD_LOCK = threading.Lock()

left_sensor = LineSensor(17)
right_sensor = LineSensor(27)

# Create an instance of the motor control class
MOTOR_CONTROL = MotorControl(24, 23, 25, 26, 19, 13)
MOTOR_NORMAL_SPEED = 60
MOOTOR_HIGH_SPEED = 65
MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)

# Create an instance of the ultrasonic sensor class
ULTRASONIC_SENSOR = UltrasonicData(echo=20, trigger=21)

# Create an instance of the fire detection class
#FIRE_DETECTION = FireDetection("DATASET/Models/candle.h5")

# Create an instance of the hand gesture detection class
#HAND_GESTURE_DETECTION = HandGestureDetection()
HAND_GESTURE_STOP_SECONDS = 10

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
#servo = ServoControl(16, minPW, maxPW)
servo_swing = True
OBSTACLE_DETECTED = False

def detect_obs():
    while True:
        if ULTRASONIC_SENSOR.get_distance() <= 10:
            # OBSTACLE_DETECTED = True
            MOTOR_CONTROL.set_speed(0)
            time.sleep(HAND_GESTURE_STOP_SECONDS)
            detect_obs()
        else:
            MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)

        time.sleep(1)

def motor_speed():
    while True:
        left_detect  = int(left_sensor.value)
        right_detect = int(right_sensor.value)

        if left_detect == 0 and right_detect == 0:
            MOTOR_CONTROL.forward()

        if left_detect == 0 and right_detect == 1:
            MOTOR_CONTROL.right()
            time.sleep(0.15)
            
        if left_detect == 1 and right_detect == 0:
            MOTOR_CONTROL.left()
            time.sleep(0.15)

def frame_processing():
    # time.sleep(2)
    ret, frame = CAP.read()
    fd_output = FIRE_DETECTION.process_frame(frame)
    hd_output = HAND_GESTURE_DETECTION.detect_hand_gesture(frame)

    print(fd_output, hd_output)

    if fd_output == True:
        SLACK_MESSAGE.send("Fire Detected!", "danger")
        MOTOR_CONTROL.set_speed(0)
        return False        
    elif hd_output:
        MOTOR_CONTROL.set_speed(MOOTOR_HIGH_SPEED)
    elif hd_output == False:
        # closed hand, pause the robot.
        MOTOR_CONTROL.set_speed(0)
        time.sleep(HAND_GESTURE_STOP_SECONDS)
        MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        servo_swing = True
    elif fd_output == False:
        MOTOR_CONTROL.set_speed(MOTOR_NORMAL_SPEED)
        servo_swing = True
    else:
        pass


def camera_handling():
    servo_swing = True

    def swing_wait():
        servo_swing = False
        time.sleep(10)
        while frame_processing() == False:
            swing_wait()
        servo_swing = True

    while True:
        if servo_swing:
            servo.center()
            # time.sleep(5)
            while frame_processing() == False:
                swing_wait()
        if servo_swing:
            servo.right()
            # time.sleep(5)
            while frame_processing() == False:
                swing_wait()
        if servo_swing:
            servo.center()
            time.sleep(5)
            while frame_processing() == False:
                swing_wa+it()
        if servo_swing:
            servo.left()
            # time.sleep(5)
            while frame_processing() == False:
                swing_wait()
        else:
            while frame_processing() == False:
                swing_wait()

def main():
    try:
        obstacle_thread = threading.Thread(target=detect_obs).start()
        motor_thread = threading.Thread(target=motor_speed).start()
        # threading.Thread(target=camera_handling).start()
    except KeyboardInterrupt:
        servo.center()
        SLACK_MESSAGE.send("Exiting..Keyboard Interrupt Detected!", "warning")

        for i in threading.enumerate():
            # wait for all threads to finish
            if i != threading.currentThread():
                i.join()
                i.stop()
        print("Exiting Main Thread")
        GPIO.cleanup()
        CAP.release()


if __name__ == '__main__':
    main()
