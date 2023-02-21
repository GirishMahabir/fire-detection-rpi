import cv2
import time
import mediapipe as mp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Initialize video capture
cap = cv2.VideoCapture(0)

while True:
    # Read frame from webcam
    ret, frame = cap.read()

    # Convert image to RGB format
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect hands in the image
    results = hands.process(image)

    # Check if any hands are detected
    if results.multi_hand_landmarks:
        # Loop through all detected hands.
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw dots at the landmarks of all fingers
            for landmark in mp_hands.HandLandmark:
                x, y = int(hand_landmarks.landmark[landmark].x * frame.shape[1]), int(
                    hand_landmarks.landmark[landmark].y * frame.shape[0])
                # Draw a circle at the landmark
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

            # Connect the dots in the hand
            connections = mp_hands.HAND_CONNECTIONS
            # Loop through all connections
            for connection in connections:
                # Get the start and end points of the connection
                x0, y0 = int(hand_landmarks.landmark[connection[0]].x * frame.shape[1]), int(
                    hand_landmarks.landmark[connection[0]].y * frame.shape[0])
                x1, y1 = int(hand_landmarks.landmark[connection[1]].x * frame.shape[1]), int(
                    hand_landmarks.landmark[connection[1]].y * frame.shape[0])
                # Draw a line between the start and end points
                cv2.line(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

            # Print all the landmarks in an easy to read format so that we can use them to detect other gestures.
            print(
                f"DATA: Thumb TIP {hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y} \
                    and MCP {hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y}")
            print(
                f"DATA: Index Finger TIP {hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y} \
                    and MCP{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y}")
            print(
                f"DATA: Middle Finger TIP {hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y} \
                    and MCP {hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y}")
            print(
                f"DATA: Ring Finger TIP {hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y} \
                    and MCP {hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y}")
            print(
                f"DATA: Pinky TIP {hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y} \
                    and MCP {hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y}")

            print("\n\n")

            # Detect open palm pointing up
            if hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y:
                cv2.putText(frame, 'Open Palm', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Detect closed Hand Gesture.
            if hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y > hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y > hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y > hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y and \
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y > hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y:
                cv2.putText(frame, 'Closed Hand', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show the image
    cv2.imshow('Hand Tracking', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
