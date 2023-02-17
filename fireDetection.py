import cv2
import numpy as np
from keras.models import load_model

# Load the trained model
model = load_model('DATASET/Models/fire.h5')

# Define the function to process each frame
firelist = ["NOT A FIRE", "FIRE"]

# Define the function to process each frame


def process_frame(frame):

    # Preprocess the frame
    resized = cv2.resize(frame, (86, 48))

    # Use the model to predict whether there is fire in the frame
    prediction = model.predict(np.expand_dims(resized, axis=0))
    print(
        f"Prediction is: {prediction} and the label is: {[np.argmax(prediction)]}")
    label = firelist[np.argmax(prediction)]
    # if prediction[0][0] > 1.5:
    #     label = "Fire"
    # else:
    #     label = "No Fire"
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
