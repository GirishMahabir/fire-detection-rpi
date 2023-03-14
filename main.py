from fireDetection import FireDetection
from handGesture import HandGesture
from followPath import FollowPath

import cv2

FIRE_MODEL = 'DATASET/Models/candle.h5'

while True:
    cap = cv2.VideoCapture(0)
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Initialize the fire class
    fire = FireDetection(FIRE_MODEL, frame)
    # Initialize the gesture class
    gesture = HandGesture(frame)

    if fire == "fire detected":
        # Stop robot movement and send a slack message.
        pass

    if gesture == "open":
        # Start robot movement, move fast.
        pass

    if gesture == "closed":
        # Stop robot movement.
        pass

    # follow path and move servo slowly.

    # repeat
