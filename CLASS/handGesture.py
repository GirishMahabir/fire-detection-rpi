import cv2
import mediapipe as mp


class HandGestureIdentification:
    # Initialize the class
    def __init__(self, frame):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.frame = frame

    def detect_hand_gesture(self):

        # Convert image to RGB format
        image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Detect hands in the image
        results = self.hands.process(image)

        # Check if any hands are detected
        if results.multi_hand_landmarks:
            # Loop through all detected hands.
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw dots at the landmarks of all fingers
                for landmark in self.mp_hands.HandLandmark:
                    x, y = int(hand_landmarks.landmark[landmark].x * self.frame.shape[1]), int(
                        hand_landmarks.landmark[landmark].y * self.frame.shape[0])
                    # Draw a circle at the landmark
                    cv2.circle(self.frame, (x, y), 5, (0, 255, 0), -1)

                # Connect the dots in the hand
                connections = self.mp_hands.HAND_CONNECTIONS
                # Loop through all connections
                for connection in connections:
                    # Get the start and end points of the connection
                    x0, y0 = int(hand_landmarks.landmark[connection[0]].x * self.frame.shape[1]), int(
                        hand_landmarks.landmark[connection[0]].y * self.frame.shape[0])
                    x1, y1 = int(hand_landmarks.landmark[connection[1]].x * frame.shape[1]), int(
                        hand_landmarks.landmark[connection[1]].y * self.frame.shape[0])
                    # Draw a line between the start and end points
                    cv2.line(self.frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

                # Detect open palm pointing up
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y:
                    cv2.putText(self.frame, 'Open Palm', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Detect closed Hand Gesture.
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y:
                    cv2.putText(self.frame, 'Closed Hand', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the image
        cv2.imshow('Hand Tracking', self.frame)


if __name__ == '__main__':
    # Initialize Frame
    capture = cv2.VideoCapture(0)
    while True:
        ret, frame = capture.read()
        screen = HandGestureIdentification(frame)
        # Detect Hand Gesture
        screen.detect_hand_gesture()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            capture.release()
            cv2.destroyAllWindows()
            break
