import cv2
import mediapipe as mp

class HandGestureDetection:
    # Initialize the class
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils

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
                # Draw landmarks on the hand
                self.mp_drawing.draw_landmarks(self.frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                                               self.mp_drawing.DrawingSpec(color=(0, 0, 255)),
                                               self.mp_drawing.DrawingSpec(color=(0, 255, 0)))

                # Detect open palm pointing up
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return "Open palm pointing up"
                # Detect open palm pointing down
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y > \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y > \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return "Open palm pointing down"

                # Detect closed Hand Gesture.
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y > \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y > \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return "Closed Hand Gesture"

                # Detect fist, straight hand pointing down
                if hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.THUMB_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                        hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[
                    self.mp_hands.HandLandmark.PINKY_MCP].y:
                    return "Fist, straight hand pointing down"

        # No hand gesture detected
        return "No hand gesture"


def main():
    # Initialize Frame
    capture = cv2.VideoCapture(0)
    screen = HandGestureDetection()

    try:
        while True:
            ret, frame = capture.read()

            # Detect Hand Gesture
            gesture = screen.detect_hand_gesture(frame)

            # Draw frame with label
            cv2.putText(frame, gesture, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Hand Gesture Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        capture.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
