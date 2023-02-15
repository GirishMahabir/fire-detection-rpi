import cv2
import numpy as np
from keras.models import load_model

# Load the trained model
model = load_model('DATASET/Models/fire.h5')

# Define the function to process each frame
firelist=['FIRE',"NOT A FIRE"]

def process_frame(frame):
    
    # Preprocess the frame
    resized = cv2.resize(frame, (48, 86))
    normalized = resized / 255.0
    reshaped = np.reshape(normalized, (1, 48, 86, 3))

    # Use the model to predict whether there is fire in the frame
    prediction = model.predict(reshaped)
    label=firelist[np.argmax(prediction)]
    cv2.putText(frame, label, (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return frame


# Start capturing the camera feed
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if ret:
        processed_frame = process_frame(frame)
        cv2.imshow("Fire Detection", processed_frame)

        # Press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
