import cv2
import mediapipe as mp

class HandGestureIdentification:
    # Initialize the class
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()

    def detect_hand_gesture(self, frame):
        self.frame = frame
        # Convert image to RGB format
        image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Detect hands in the image
        results = self.hands.process(image)

        # Check if any hands are detected
        if results.multi_hand_landmarks:
            # Loop through all detected hands.
            for hand_landmarks in results.multi_hand_landmarks:
                # Detect open palm pointing up
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return True

                # Detect closed Hand Gesture.
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return False


if __name__ == '__main__':
    # Initialize Frame
    capture = cv2.VideoCapture(0)
    screen = HandGestureIdentification()

    try:
        while True:
            ret, frame = capture.read()
            # Detect Hand Gesture
            print(screen.detect_hand_gesture(frame))
    except KeyboardInterrupt:
        capture.release()
