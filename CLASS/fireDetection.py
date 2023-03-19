import cv2
import numpy as np
from keras.models import load_model


class FireDetection:
    def __init__(self, model, frame):
        self.frame = frame
        self.model = model
        self.firelist = ["NOT A FIRE", "FIRE"]

    def process_frame(self):
        # Preprocess the frame
        resized = cv2.resize(frame, (86, 48))

        # Use the model to predict whether there is fire in the frame
        prediction = model.predict(np.expand_dims(resized, axis=0))
        print(
            f"Prediction is: {prediction} and the label is: {[np.argmax(prediction)]}")

        label = self.firelist[np.argmax(prediction)]

        cv2.putText(frame, label, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if label == "FIRE":
            return "fire detected"

        return frame


if __name__ == "__main__":
    # Start capturing the camera feed
    cap = cv2.VideoCapture(0)
    # Load the trained model
    model = load_model('DATASET/Models/candle.h5')

    while True:
        ret, frame = cap.read()
        # Initialize the class
        fire = FireDetection(model, frame)
        if ret:
            processed_frame = fire.process_frame()
            cv2.imshow("Fire Detection", processed_frame)

            # Press q to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
